#!/usr/bin/env python3
from __future__ import annotations

import os
import time
import threading
import subprocess
import sys
from typing import List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
import pyaudio
from ultralytics import YOLO

from kitchen_assistant.realsense import ImgNode
from kitchen_assistant.utils import resolve_model_path
from kitchen_assistant.onrobot import RG

from kitchen_assistant.voice.wakeup_word import WakeupWord
from kitchen_assistant.voice.stt_openai import STT
from kitchen_assistant.voice.command_parser import parse_command, ParseResult
from kitchen_assistant.voice.runner import run_object, run_post_actions, RunResult

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

def _env_default(key: str, default: str) -> str:
    v = os.environ.get(key)
    if v is None:
        return default
    v = v.strip()
    return v if v else default


ROBOT_ID = _env_default("ROBOT_ID", "dsr01")
ROBOT_MODEL = _env_default("ROBOT_MODEL", "m0609")

J_HOME = [0, 0, 0, 0, 0, 0]
J_PERSON = [0, 0, -90, 90, 0, 180]

HOME_VEL = float(_env_default("KITCHEN_HOME_VEL", "60"))
HOME_ACC = float(_env_default("KITCHEN_HOME_ACC", "60"))

GRIPPER_NAME = _env_default("KITCHEN_GRIPPER_NAME", "rg2")
TOOLCHARGER_IP = _env_default("KITCHEN_TOOLCHARGER_IP", "192.168.1.1")
TOOLCHARGER_PORT = int(_env_default("KITCHEN_TOOLCHARGER_PORT", "502"))

PERSON_CONF_TH = float(_env_default("KITCHEN_PERSON_CONF_TH", "0.35"))


def _obj_kor(obj_id: str) -> str:
    return {
        "cup": "컵",
        "ramen": "라면",
        "scissors": "가위",
        "spatula": "뒤집개",
        "spoon": "국자",
        "knife": "칼",
        "pan": "프라이팬",
        "pot": "냄비",
        "apple": "사과",
        "orange": "오렌지",
        "bottle": "콜라",
    }.get(obj_id, obj_id)


def _run_movej(joints: List[float], name: str) -> int:
    # Run Doosan movej in a fresh process (robust against g_node/context issues)
    joints_str = ",".join(str(float(x)) for x in joints)
    cmd = [
        sys.executable, "-m", "kitchen_assistant.tools.robot_movej",
        "--joints", joints_str,
        "--vel", str(HOME_VEL),
        "--acc", str(HOME_ACC),
        "--name", name,
            "--timeout", _env_default("KITCHEN_MOVEJ_TIMEOUT", "15.0"),
    ]
    env = os.environ.copy()
    env["ROBOT_ID"] = ROBOT_ID
    env["ROBOT_MODEL"] = ROBOT_MODEL
    return subprocess.call(cmd, env=env)


class KitchenVoiceController:
    def __init__(self):
        print(f"[init] robot_id={ROBOT_ID} robot_model={ROBOT_MODEL}")
        print(f"[init] toolcharger={TOOLCHARGER_IP}:{TOOLCHARGER_PORT} gripper={GRIPPER_NAME}")

        # rclpy already inited in main()
        self.node = rclpy.create_node("kitchen_voice")
        self.img_node = ImgNode()

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.img_node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

        # Person model
        self.person_model = YOLO(resolve_model_path("yolo11n.pt"))

        # STT
        self.stt = STT(duration=float(_env_default("KITCHEN_STT_DURATION", "5.0")))

        # Gripper (release only)
        self.gripper = None
        try:
            print("[INIT] Gripper connect...")
            self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        except Exception as e:
            print("[WARN] Gripper init failed:", e)
            self.gripper = None

    def goto_home(self):
        rc = _run_movej(J_HOME, "kitchen_goto_home")
        if rc != 0:
            print(f"[WARN] goto_home failed rc={rc}")

    def goto_person_pose(self):
        rc = _run_movej(J_PERSON, "kitchen_goto_person")
        if rc != 0:
            print(f"[WARN] goto_person_pose failed rc={rc}")

    def open_gripper(self):
        if self.gripper is None:
            print("[WARN] Gripper not available.")
            return
        try:
            self.gripper.open_gripper()
        except Exception as e:
            print("[WARN] open_gripper failed:", e)

    # ---------------------------
    # Wakeword / person detection
    # ---------------------------
    def wait_wakeword(self) -> bool:
        print("[state] WAKEWORD_WAIT (say wakeword)")
        pa = pyaudio.PyAudio()
        stream = pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=48000,
            input=True,
            frames_per_buffer=4096,
        )
        detector = WakeupWord(buffer_size=4096, threshold=float(_env_default("KITCHEN_WAKE_TH", "0.30")))
        detector.set_stream(stream)
        try:
            while rclpy.ok():
                if detector.is_wakeup():
                    return True
        finally:
            try:
                stream.stop_stream()
                stream.close()
            except Exception:
                pass
            try:
                pa.terminate()
            except Exception:
                pass
        return False

    def wait_person(self) -> bool:
        print("[state] PERSON_WAIT (YOLO person)")
        while rclpy.ok():
            frame = self.img_node.get_color_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            try:
                res = self.person_model(frame, verbose=False)[0]
            except Exception:
                time.sleep(0.05)
                continue

            if res.boxes is None or len(res.boxes) == 0:
                time.sleep(0.05)
                continue

            try:
                cls_list = res.boxes.cls.tolist()
                conf_list = res.boxes.conf.tolist()
            except Exception:
                time.sleep(0.05)
                continue

            for cls, conf in zip(cls_list, conf_list):
                name = self.person_model.names.get(int(cls), str(int(cls)))
                if name == "person" and float(conf) >= PERSON_CONF_TH:
                    return True
            time.sleep(0.05)
        return False

    # ---------------------------
    # STT loops
    # ---------------------------
    def listen(self) -> str:
        return self.stt.speech2text()

    def listen_parse(self) -> ParseResult:
        text = self.listen()
        return parse_command(text)

    def wait_release(self, pending: List[str]) -> str:
        print("[state] STT_WAIT_RELEASE (say: '줘/그리퍼 열어' to release)")
        while rclpy.ok():
            pr = self.listen_parse()
            if pr.kind == "release":
                self.open_gripper()
                return "released"
            if pr.kind == "rest":
                return "rest"
            if pr.kind == "plan":
                print("[voice] 먼저 물건을 건네고(줘/그리퍼 열어) 다음 명령을 해주세요.")
            else:
                print("[voice] 못 알아들었어요. '줘' 또는 '그리퍼 열어' 라고 말해보세요.")
        return "rest"

    def run(self):
        # Startup -> home -> wakeword
        self.goto_home()

        state = "WAKEWORD"
        pending: List[str] = []

        while rclpy.ok():
            if state == "WAKEWORD":
                ok = self.wait_wakeword()
                if not ok:
                    continue
                self.goto_person_pose()
                self.wait_person()
                print("안녕하세요")
                state = "STT"
                continue

            if state == "STT":
                print("[state] STT_WAIT (say: '무엇을 가져와')")
                pr = self.listen_parse()

                if pr.kind == "rest":
                    self.goto_home()
                    pending = []
                    state = "WAKEWORD"
                    continue

                if pr.kind == "release":
                    self.open_gripper()
                    self.goto_person_pose()
                    self.wait_person()
                    state = "STT"
                    continue

                if pr.kind != "plan" or not pr.plan:
                    print("[voice] 명령을 이해하지 못했어요.")
                    continue

                pending = list(pr.plan)
                state = "EXEC"
                continue

            if state == "EXEC":
                if not pending:
                    self.goto_person_pose()
                    self.wait_person()
                    state = "STT"
                    continue

                obj = pending.pop(0)

                self.goto_home()
                print(f"[plan] execute: {obj} (remaining={len(pending)})")

                rr: RunResult = run_object(obj)

                if rr.status == "not_found":
                    print(f"{_obj_kor(obj)}이(가) 없습니다")
                    self.goto_home()
                    continue

                if rr.status != "ok":
                    if rr.rc == 101:
                        print(f"[WARN] transient error -> skip: {obj} (module={rr.failed_module})")
                        self.goto_home()
                        continue
                    print(f"[error] failed: {obj} rc={rr.rc} module={rr.failed_module}")
                    self.goto_home()
                    pending = []
                    state = "STT"
                    continue

                # Success -> wait for release
                res = self.wait_release(pending)
                if res == "rest":
                    print("[voice] rest requested. releasing object before sleeping.")
                    self.open_gripper()
                    if getattr(rr, "post_actions", None):
                        print("[voice] post actions (e.g. fridge close) ...")
                        ok_post = run_post_actions(list(rr.post_actions))
                        if not ok_post:
                            print("[WARN] post actions failed.")
                    self.goto_home()
                    pending = []
                    state = "WAKEWORD"
                    continue

                # Normal release path: run post-actions after handover (e.g., close fridge)
                if getattr(rr, "post_actions", None):
                    print("[voice] post actions (e.g. fridge close) ...")
                    ok_post = run_post_actions(list(rr.post_actions))
                    if not ok_post:
                        print("[WARN] post actions failed. aborting current plan.")
                        self.goto_home()
                        pending = []
                        state = "STT"
                        continue

                if pending:
                    continue

                self.goto_person_pose()
                self.wait_person()
                state = "STT"

    def shutdown(self):
        try:
            if getattr(self, "img_node", None):
                self.img_node.destroy_node()
        except Exception:
            pass
        try:
            if getattr(self, "node", None):
                self.node.destroy_node()
        except Exception:
            pass
        try:
            if getattr(self, "executor", None):
                self.executor.shutdown()
        except Exception:
            pass


def main():
    rclpy.init()
    ctrl: Optional[KitchenVoiceController] = None
    try:
        ctrl = KitchenVoiceController()
        ctrl.run()
    finally:
        try:
            if ctrl:
                ctrl.shutdown()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()

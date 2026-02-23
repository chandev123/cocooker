#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fridge_open_then_apple_pick.py

✅ 목적: 냉장고 문 열기 -> 사과 감지/집기


필수:
- 다른 터미널에서 bringup(real) 실행 중이어야 함.
- 같은 폴더에 T_gripper2camera.npy 존재.
"""

import time
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

import DR_init
from kitchen_assistant.utils import resolve_model_path, resolve_resource_path
from kitchen_assistant.realsense import ImgNode
from kitchen_assistant.onrobot import RG


# -----------------------------
# USER CONFIG
# -----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

# Doosan enums (as you used)
DR_BASE = 0
DR_MV_MOD_REL = 1

# Motion
VELOCITY = 60
ACC = 60

# Gripper (RG2)
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# Apple YOLO
MODEL_PATH = resolve_model_path(resolve_model_path("yolov8n.pt"))   # custom pt면 수정
TARGET_NAME = "apple"
CONF_THRES = 0.45
DETECT_HOLD_SEC = 2.0
MAX_GAP_SEC = 0.35

# pick motion tuning (mm)
PREAPPROACH_DIST = 80
PICK_PUSH_DIST  = 33
RETREAT_DIST    = 120
GRIP_WAIT = 0.7
SAFE_LIFT_Z = 60
CAM_ALIGN_TH = 0.75

HAND_EYE_FILENAME = "T_gripper2camera.npy"
AFTER_OPEN_WAIT_SEC = 0.8


def resolve_path_near_script(filename: str) -> str:
    base_dir = Path(__file__).resolve().parent
    p = Path(filename)
    if p.is_file():
        return str(p.resolve())
    p2 = base_dir / filename
    if p2.is_file():
        return str(p2.resolve())
    return filename


@dataclass
class DsrApi:
    """Doosan motion API wrapper (removes undefined-name warnings)."""
    movej: object
    movel: object
    wait: object
    get_current_posx: object
    posx: object


def fridge_open_sequence(dsr: DsrApi, gripper: RG):
    """
    fridge_rightdown_open.py 의 do_motion_demo 내용을 그대로 옮김.
    """
    print("[FRIDGE] start sequence")

    dsr.movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([852.708, 84.156, 137.864, 179.972, -90.238, -90.173]), vel=VELOCITY, acc=ACC)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([20.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
              vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
    gripper.close_gripper()
    time.sleep(0.2)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([0.0, 0.0, 0.0, 10.0, 0.0, 0.0]),
              vel=5, acc=5, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([-50.0, -10.0, 0.0, 0.0, 0.0, 0.0]),
              vel=30, acc=30, ref=DR_BASE, mod=DR_MV_MOD_REL)
    gripper.open_gripper()
    time.sleep(0.2)

    dsr.movel(dsr.posx([0.0, 0.0, 0.0, -10.0, 0.0, 0.0]),
              vel=5, acc=5, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([0.0, 0.0, 160.0, 0.0, 0.0, 0.0]),
              vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    gripper.close_gripper()
    time.sleep(0.2)

    dsr.movel(dsr.posx([35.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
              vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([0.0, 40.0, 0.0, 0.0, 0.0, 0.0]),
              vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([0.0, 0.0, -80.0, 0.0, 0.0, 0.0]),
              vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([0.0, 0.0, 0.0, -34.0, 0.0, 0.0]),
              vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([0.0, -90.0, 0.0, 0.0, 0.0, 0.0]),
              vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([0.0, 0.0, 0.0, 34.0, 0.0, 0.0]),
              vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    dsr.movel(dsr.posx([0.0, 0.0, -80.0, 0.0, 0.0, 0.0]),
              vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
    dsr.wait(0.5)

    gripper.open_gripper()
    time.sleep(0.2)

    print("[FRIDGE] done sequence")


class ApplePickCore(Node):
    def __init__(self, dsr: DsrApi, gripper: RG):
        super().__init__("apple_pick_node")
        self.dsr = dsr
        self.gripper = gripper

        print("[APPLE] init ImgNode...")
        self.img_node = ImgNode()
        rclpy.spin_once(self.img_node)
        time.sleep(1.0)

        self.intrinsics = self.img_node.get_camera_intrinsic()
        if self.intrinsics is None:
            self.get_logger().warn("Camera intrinsics not received yet. Waiting a bit...")
            for _ in range(30):
                rclpy.spin_once(self.img_node)
                time.sleep(0.05)
                self.intrinsics = self.img_node.get_camera_intrinsic()
                if self.intrinsics is not None:
                    break
        if self.intrinsics is None:
            raise RuntimeError("No camera intrinsics. Check /camera/camera/color/camera_info topic.")

        he_path = resolve_path_near_script(HAND_EYE_FILENAME)
        self.gripper2cam = np.load(he_path)
        print(f"[APPLE] hand-eye loaded: {he_path}")

        from ultralytics import YOLO
        model_path = resolve_path_near_script(MODEL_PATH)
        print(f"[APPLE] loading YOLO: {model_path}")
        self.model = YOLO(model_path)
        self.names = self.model.names

        apple_ids = [i for i, n in self.names.items() if n == TARGET_NAME]
        if not apple_ids:
            raise RuntimeError(f'YOLO model has no class named "{TARGET_NAME}". names={self.names}')
        self.apple_class_id = int(apple_ids[0])

        self.hold_start_t: Optional[float] = None
        self.last_seen_t: Optional[float] = None
        self.done_once = False

        self.get_logger().info(f"Ready. Holding condition: {TARGET_NAME} >= {DETECT_HOLD_SEC:.1f}s")

    # ---- geometry helpers ----
    def get_camera_pos(self, center_x, center_y, center_z, intrinsics):
        camera_x = (center_x - intrinsics["ppx"]) * center_z / intrinsics["fx"]
        camera_y = (center_y - intrinsics["ppy"]) * center_z / intrinsics["fy"]
        camera_z = center_z
        return (camera_x, camera_y, camera_z)

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def get_base2cam(self):
        base2gripper = self.get_robot_pose_matrix(*self.dsr.get_current_posx()[0])
        return base2gripper @ self.gripper2cam

    def transform_to_base(self, camera_coords):
        coord = np.append(np.array(camera_coords), 1)
        base2cam = self.get_base2cam()
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]

    def get_depth_value(self, x, y, depth_frame):
        h, w = depth_frame.shape
        if 0 <= x < w and 0 <= y < h:
            return depth_frame[y, x]
        return None

    # ---- approach axis ----
    def infer_approach_axis_from_camera(self):
        base2cam = self.get_base2cam()
        Rm = base2cam[:3, :3]
        v_base = Rm @ np.array([0.0, 0.0, 1.0])
        idx = int(np.argmax(np.abs(v_base)))
        axis = ["x", "y", "z"][idx]
        sign = 1 if v_base[idx] >= 0 else -1
        confidence = float(abs(v_base[idx]))
        return axis, sign, confidence, v_base

    def choose_approach_axis(self, target_xyz):
        axis, sign, conf, v_base = self.infer_approach_axis_from_camera()
        if conf >= CAM_ALIGN_TH:
            self.get_logger().info(
                f"Approach axis from camera: axis={axis} sign={sign} conf={conf:.2f} v_base={v_base}"
            )
            return axis, sign, "camera"

        cur = self.dsr.get_current_posx()[0]
        tcp_xyz = np.array(cur[:3], dtype=float)
        tgt = np.array(target_xyz[:3], dtype=float)
        d = tgt - tcp_xyz
        idx = int(np.argmax(np.abs(d)))
        axis2 = ["x", "y", "z"][idx]
        sign2 = 1 if d[idx] >= 0 else -1
        self.get_logger().info(
            f"Approach axis fallback(dominant delta): axis={axis2} sign={sign2} delta={d}"
        )
        return axis2, sign2, "delta"

    @staticmethod
    def axis_unit(axis, sign):
        if axis == "x":
            return np.array([float(sign), 0.0, 0.0])
        if axis == "y":
            return np.array([0.0, float(sign), 0.0])
        if axis == "z":
            return np.array([0.0, 0.0, float(sign)])
        raise ValueError(f"invalid axis: {axis}")

    def pick_sequence(self, target_xyz, approach_axis, approach_sign):
        self.dsr.movej([0.0, 0.0, 0.0, 0.0, 0.0, -30.0], vel=VELOCITY, acc=ACC, 
              mod=DR_MV_MOD_REL)
        current_pos = self.dsr.get_current_posx()[0]
        rx, ry, rz = current_pos[3], current_pos[4], current_pos[5]

        tgt = np.array(target_xyz[:3], dtype=float)
        a = self.axis_unit(approach_axis, approach_sign)

        if SAFE_LIFT_Z and SAFE_LIFT_Z > 0:
            lift = self.dsr.posx([float(current_pos[0]), float(current_pos[1]), float(current_pos[2] + SAFE_LIFT_Z),
                                  rx, ry, rz])
            self.dsr.movel(lift, vel=VELOCITY, acc=ACC)

        pre_xyz     = tgt - a * PREAPPROACH_DIST
        pick_xyz    = tgt + a * PICK_PUSH_DIST
        retreat_xyz = tgt - a * RETREAT_DIST

        pre     = self.dsr.posx([float(pre_xyz[0]),     float(pre_xyz[1]),     float(pre_xyz[2]),     rx, ry, rz])
        pick    = self.dsr.posx([float(pick_xyz[0]),    float(pick_xyz[1]),    float(pick_xyz[2]),    rx, ry, rz])
        retreat = self.dsr.posx([float(retreat_xyz[0]), float(retreat_xyz[1]), float(retreat_xyz[2]), rx, ry, rz])

        self.get_logger().info(
            f"Pick sequence: axis={approach_axis} sign={approach_sign} "
            f"pre={pre_xyz.tolist()} pick={pick_xyz.tolist()} retreat={retreat_xyz.tolist()}"
        )

        self.dsr.movel(pre, vel=VELOCITY, acc=ACC)
        self.dsr.movel(pick, vel=VELOCITY, acc=ACC)

        self.gripper.close_gripper()
        self.dsr.wait(GRIP_WAIT)

        self.dsr.movel(retreat, vel=VELOCITY, acc=ACC)

        self.dsr.wait(GRIP_WAIT)
        self.dsr.movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
        self.gripper.open_gripper()
        self.dsr.wait(GRIP_WAIT)

    def step(self):
        rclpy.spin_once(self.img_node, timeout_sec=0.0)

        img = self.img_node.get_color_frame()
        depth = self.img_node.get_depth_frame()
        if img is None or depth is None:
            return None

        results = self.model.predict(img, conf=CONF_THRES, verbose=False)
        if not results:
            return img

        r = results[0]
        best = None
        if r.boxes is not None and len(r.boxes) > 0:
            for b in r.boxes:
                cls = int(b.cls.item())
                if cls != self.apple_class_id:
                    continue
                conf = float(b.conf.item())
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
                if best is None or conf > best[0]:
                    best = (conf, (x1, y1, x2, y2))

        now = time.monotonic()

        if best is None:
            if self.last_seen_t is not None and (now - self.last_seen_t) > MAX_GAP_SEC:
                self.hold_start_t = None
                self.last_seen_t = None
            return img

        conf, (x1, y1, x2, y2) = best
        cx = int((x1 + x2) / 2.0)
        cy = int((y1 + y2) / 2.0)

        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)

        if self.last_seen_t is None or (now - self.last_seen_t) > MAX_GAP_SEC:
            self.hold_start_t = now
        self.last_seen_t = now

        held = now - (self.hold_start_t or now)
        cv2.putText(img, f"{TARGET_NAME} conf={conf:.2f} hold={held:.2f}s",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if held >= DETECT_HOLD_SEC:
            z = self.get_depth_value(cx, cy, depth)
            retry = 0
            while (z is None or z == 0) and retry < 10:
                rclpy.spin_once(self.img_node, timeout_sec=0.0)
                depth2 = self.img_node.get_depth_frame()
                if depth2 is not None:
                    z = self.get_depth_value(cx, cy, depth2)
                retry += 1
                time.sleep(0.03)

            if z is None or z == 0:
                self.get_logger().warn("Depth invalid. Skipping this trigger.")
                self.hold_start_t = None
                self.last_seen_t = None
                return img

            z_val = float(z)
            if z_val < 10.0:
                z_val *= 1000.0

            cam_pos = self.get_camera_pos(cx, cy, z_val, self.intrinsics)
            base_xyz = self.transform_to_base(cam_pos)

            axis, sign, method = self.choose_approach_axis(base_xyz)

            self.get_logger().info(
                f"TRIGGER: {TARGET_NAME} held {held:.2f}s -> pixel=({cx},{cy}) depth={z_val:.1f} "
                f"-> base={base_xyz} -> approach={axis}{'+' if sign > 0 else '-'} (by {method})"
            )

            self.hold_start_t = None
            self.last_seen_t = None

            self.pick_sequence(base_xyz, axis, sign)
            self.done_once = True

        return img


def main():
    rclpy.init()

    print("[MAIN] create Doosan node...")
    node = rclpy.create_node("fridge_open_then_apple_pick_v4", namespace=ROBOT_ID)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    # Doosan API import (same as your fridge/apple scripts)
    from DSR_ROBOT2 import movej, movel, wait, get_current_posx
    from DR_common2 import posx

    dsr = DsrApi(movej=movej, movel=movel, wait=wait, get_current_posx=get_current_posx, posx=posx)

    print("[MAIN] Gripper connect...")
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
    try:
        gripper.open_gripper()
        time.sleep(0.2)
    except Exception as e:
        print("[GRIPPER] open init error:", e)

    # 1) fridge open
    fridge_open_sequence(dsr, gripper)
    time.sleep(AFTER_OPEN_WAIT_SEC)

    # 2) apple detect/pick
    print("[MAIN] start apple detection loop...")
    app = ApplePickCore(dsr, gripper)

    cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL)

    timeout = float(os.environ.get('KITCHEN_DETECT_TIMEOUT_SEC', '12.0'))
    t0 = time.monotonic()
    obj_name = globals().get('TARGET_NAME', 'object')

    while True:
        if (time.monotonic() - t0) > timeout and not getattr(app, "done_once", False):
            print(f"[TIMEOUT] {obj_name} not detected for {timeout:.1f}s")
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
            sys.exit(2)

        frame = app.step()
        if getattr(app, "done_once", False):
            break
        if frame is not None:
            cv2.imshow("Webcam", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
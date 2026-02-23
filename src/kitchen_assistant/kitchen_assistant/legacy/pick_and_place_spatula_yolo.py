#!/usr/bin/env python3
import time
import os
import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from kitchen_assistant.realsense import ImgNode
from kitchen_assistant.onrobot import RG

import DR_init
from kitchen_assistant.utils import resolve_model_path, resolve_resource_path

# -----------------------------
# Image enhancement (optional)
# -----------------------------
# If you want to boost saturation/brightness BEFORE feeding into YOLO,
# set COLOR_ENHANCE=True and APPLY_ENHANCE_TO_MODEL=True.
COLOR_ENHANCE = False
APPLY_ENHANCE_TO_MODEL = True

SAT_MULT = 1.25   # 1.0=no change, try 1.15~1.40
VAL_MULT = 1.05   # 1.0=no change, try 1.00~1.15
GAMMA    = 1.00   # 1.0=no change

def enhance_bgr(img_bgr):
    """HSV saturation/value boost + optional gamma. Input/Output are BGR (cv2)."""
    if img_bgr is None:
        return img_bgr
    out = img_bgr
    if SAT_MULT != 1.0 or VAL_MULT != 1.0:
        hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[..., 1] *= float(SAT_MULT)
        hsv[..., 2] *= float(VAL_MULT)
        hsv[..., 1] = np.clip(hsv[..., 1], 0, 255)
        hsv[..., 2] = np.clip(hsv[..., 2], 0, 255)
        out = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

    if GAMMA != 1.0:
        inv = 1.0 / float(GAMMA)
        lut = (np.clip(((np.arange(256) / 255.0) ** inv) * 255.0, 0, 255)).astype(np.uint8)
        out = cv2.LUT(out, lut)

    return out


# -----------------------------
# User parameters
# -----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_BASE = 0
DR_MV_MOD_REL = 1

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

MODEL_PATH = resolve_model_path(resolve_model_path("kitchen.pt"))
TARGET_NAME = "spatula"

CONF_THRES = 0.45
DETECT_HOLD_SEC = 2.0                    # "2초 이상 감지" 조건
MAX_GAP_SEC = 0.35                       # 프레임 누락/미검출 허용 간격(넘으면 리셋)

# pick motion tuning (mm)
# ※ 이제 "Z축" 고정이 아니라, 선택된 접근축(x/y/z) 방향으로 적용됩니다.
PREAPPROACH_DIST = 80                    # 대상에서 멀리 떨어진 프리어프로치 오프셋
PICK_PUSH_DIST  = 25                    # 최종 집기 시 대상 쪽으로 더 밀어주는 거리(기존 z - PICK_DZ 의미)
RETREAT_DIST    = 120                    # 집은 뒤 반대 방향으로 빠지는 거리
GRIP_WAIT = 1.0

# (선택) 접근 전에 현재 위치에서 안전하게 위로 살짝 올림 (base +Z 방향)
SAFE_LIFT_Z = 60                         # 0이면 비활성

# 카메라 정렬로 접근축을 고를 때, 축 정렬 신뢰도 임계값 (0~1)
CAM_ALIGN_TH = 0.75

# -----------------------------
# Doosan init
# -----------------------------
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ApplePickNode(Node):
    def __init__(self):
        super().__init__("apple_pick_node")

        # Camera node (your realsense.py)
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

        # Hand-eye matrix
        # NOTE: code assumes this is T_gripper->camera (base2cam = base2gripper @ gripper2cam)
        self.gripper2cam = np.load(resolve_resource_path("T_gripper2camera.npy"))

        # Gripper
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        try:
            self.gripper.open_gripper()
        except Exception as e:
            self.get_logger().warn(f"gripper open failed (ignored): {e}")

        # YOLO
        from ultralytics import YOLO
        self.model = YOLO(MODEL_PATH)
        self.names = self.model.names  # id -> name

        self.apple_class_ids = [i for i, n in self.names.items() if n == TARGET_NAME]
        if not self.apple_class_ids:
            raise RuntimeError(f'YOLO model has no class named "{TARGET_NAME}". names={self.names}')
        self.apple_class_id = int(self.apple_class_ids[0])

        # Detection hold state
        self.hold_start_t = None
        self.last_seen_t = None
        self.last_center = None  # (cx, cy)

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
        """Returns T_base->camera using current TCP pose + hand-eye."""
        base2gripper = self.get_robot_pose_matrix(*get_current_posx()[0])
        return base2gripper @ self.gripper2cam

    def transform_to_base(self, camera_coords):
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate
        base2cam = self.get_base2cam()
        td_coord = np.dot(base2cam, coord)
        return td_coord[:3]

    def get_depth_value(self, x, y, depth_frame):
        h, w = depth_frame.shape
        if 0 <= x < w and 0 <= y < h:
            return depth_frame[y, x]
        return None

    # ---- approach axis inference ----
    def infer_approach_axis_from_camera(self):
        """
        카메라의 +Z(전방/깊이) 축이 base 좌표계의 어떤 축(x/y/z)과 가장 정렬되어 있는지 추정.
        return: (axis_char, sign, confidence, v_base)
          - axis_char: 'x'|'y'|'z'
          - sign: +1 or -1 (base +axis 방향으로 카메라 전방이 향하면 +1)
          - confidence: |dot| (0~1)
          - v_base: 카메라 전방 벡터를 base에서 본 값 (3,)
        """
        base2cam = self.get_base2cam()
        R = base2cam[:3, :3]

        # Camera forward axis: +Z in camera coordinates (RealSense/typical convention)
        v_base = R @ np.array([0.0, 0.0, 1.0])

        idx = int(np.argmax(np.abs(v_base)))
        axis = ["x", "y", "z"][idx]
        sign = 1 if v_base[idx] >= 0 else -1
        confidence = float(abs(v_base[idx]))
        return axis, sign, confidence, v_base

    def choose_approach_axis(self, target_xyz):
        """
        1) 카메라 정렬(카메라 전방축)로 접근축 결정
        2) 정렬이 애매하면(conf 낮음) 현재 TCP->타깃 벡터의 가장 큰 성분축으로 fallback
        """
        axis, sign, conf, v_base = self.infer_approach_axis_from_camera()

        if conf >= CAM_ALIGN_TH:
            self.get_logger().info(
                f"Approach axis from camera: axis={axis} sign={sign} conf={conf:.2f} v_base={v_base}"
            )
            return axis, sign, conf, "camera"

        # fallback: dominant delta axis from current TCP to target
        cur = get_current_posx()[0]
        tcp_xyz = np.array(cur[:3], dtype=float)
        tgt = np.array(target_xyz[:3], dtype=float)
        d = tgt - tcp_xyz
        idx = int(np.argmax(np.abs(d)))
        axis2 = ["x", "y", "z"][idx]
        sign2 = 1 if d[idx] >= 0 else -1
        conf2 = float(abs(d[idx]) / (np.linalg.norm(d) + 1e-9))  # relative dominance
        self.get_logger().info(
            f"Approach axis fallback(dominant delta): axis={axis2} sign={sign2} conf~={conf2:.2f} "
            f"delta={d} (camera_conf={conf:.2f} < {CAM_ALIGN_TH})"
        )
        return axis2, sign2, conf2, "delta"

    @staticmethod
    def axis_unit(axis, sign):
        if axis == "x":
            return np.array([float(sign), 0.0, 0.0])
        if axis == "y":
            return np.array([0.0, float(sign), 0.0])
        if axis == "z":
            return np.array([0.0, 0.0, float(sign)])
        raise ValueError(f"invalid axis: {axis}")

    # ---- robot action ----
    def pick_sequence(self, target_xyz, approach_axis, approach_sign):
        """
        target_xyz: base frame position (mm)
        approach_axis/sign: 선택된 접근축과 방향
          - 예) 카메라가 위에서 아래(-z)로 보고 있으면 approach_axis='z', approach_sign=-1
          - 예) 카메라가 +x 방향으로 보고 있으면 approach_axis='x', approach_sign=+1
        """
        # movej([0.0, 0.0, 0.0, 0.0, 0.0, -30.0], vel=VELOCITY, acc=ACC, 
        #       mod=DR_MV_MOD_REL)
        current_pos = get_current_posx()[0]
        rx, ry, rz = current_pos[3], current_pos[4], current_pos[5]

        tgt = np.array(target_xyz[:3], dtype=float)
        a = self.axis_unit(approach_axis, approach_sign)  # toward-object direction

        # (선택) 먼저 살짝 위로 안전 리프트
        # if SAFE_LIFT_Z and SAFE_LIFT_Z > 0:
        #     lift = posx([float(current_pos[0]), float(current_pos[1]), float(current_pos[2] + SAFE_LIFT_Z), rx, ry, rz])
        #     movel(lift, vel=VELOCITY, acc=ACC)

        pre_xyz     = tgt - a * PREAPPROACH_DIST
        pick_xyz    = tgt + a * PICK_PUSH_DIST
        retreat_xyz = tgt - a * RETREAT_DIST

        pre     = posx([float(pre_xyz[0]),     float(pre_xyz[1]),     float(pre_xyz[2]),     rx, ry, rz])
        pick    = posx([float(pick_xyz[0]),    float(pick_xyz[1]),    float(pick_xyz[2]),    rx, ry, rz])
        retreat = posx([float(retreat_xyz[0]), float(retreat_xyz[1]), float(retreat_xyz[2]), rx, ry, rz])

        self.get_logger().info(
            f"Pick sequence: axis={approach_axis} sign={approach_sign} "
            f"pre={pre_xyz.tolist()} pick={pick_xyz.tolist()} retreat={retreat_xyz.tolist()}"
        )

        # Approach -> Push -> Grip -> Retreat
        
        movel(pre, vel=VELOCITY, acc=ACC)
        movel(pick, vel=VELOCITY, acc=ACC)

        self.gripper.close_gripper()
        wait(GRIP_WAIT)
        

        # movel(retreat, vel=VELOCITY, acc=ACC)

        movel([0.0, 20.0, 20.0, 0.0, 0.0, 0.0], vel=VELOCITY, acc=ACC, 
               mod=DR_MV_MOD_REL)
        wait(0.5)

        movel([0.0, 70.0, 0.0, 0.0, 0.0, 0.0], vel=VELOCITY, acc=ACC, 
               mod=DR_MV_MOD_REL)
        wait(0.5)
        
        movel([0.0, 0.0, 300.0, 0.0, 0.0, 0.0], vel=VELOCITY, acc=ACC, 
               mod=DR_MV_MOD_REL)
        wait(0.5)
        
        movel([-650.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=VELOCITY, acc=ACC, 
               mod=DR_MV_MOD_REL)
        wait(0.5)

        movej([-70.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=30, acc=30, 
               mod=DR_MV_MOD_REL)
        wait(0.5)

        width = self.gripper.get_width()
        if width > 12 :
            print(f'집기에 성공했습니다 {width}' )
        else :
            print(f'집기에 실패했습니다. {width}')
        # drop (원래 코드 유지)
        # wait(GRIP_WAIT)
        # movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
        # self.gripper.open_gripper()
        # wait(GRIP_WAIT)


    # ---- YOLO result parsing helpers (supports BOX & OBB) ----
    def _select_best_detection(self, r):
        """
        Returns dict or None.
        dict keys:
          - conf (float)
          - cx, cy (int) : pixel center for depth
          - kind: "obb" | "box"
          - draw: data for drawing (polygon or xyxy)
        """
        best = None

        # 1) OBB (Ultralytics OBB models usually populate r.obb, not r.boxes)
        obb = getattr(r, "obb", None)
        if obb is not None and len(obb) > 0:
            has_xywhr = hasattr(obb, "xywhr") and obb.xywhr is not None
            has_xyxyxyxy = hasattr(obb, "xyxyxyxy") and obb.xyxyxyxy is not None

            for i in range(len(obb)):
                cls = int(obb.cls[i].item()) if hasattr(obb, "cls") else None
                if cls is None or cls != self.apple_class_id:
                    continue
                conf = float(obb.conf[i].item()) if hasattr(obb, "conf") else 0.0

                if has_xywhr:
                    cx_f = float(obb.xywhr[i][0].item())
                    cy_f = float(obb.xywhr[i][1].item())
                elif has_xyxyxyxy:
                    pts = obb.xyxyxyxy[i].reshape(4, 2).cpu().numpy()
                    cx_f = float(pts[:, 0].mean())
                    cy_f = float(pts[:, 1].mean())
                else:
                    if hasattr(obb, "xyxy") and obb.xyxy is not None:
                        x1, y1, x2, y2 = [float(v) for v in obb.xyxy[i].tolist()]
                        cx_f = (x1 + x2) / 2.0
                        cy_f = (y1 + y2) / 2.0
                    else:
                        continue

                cand = {
                    "conf": conf,
                    "cx": int(round(cx_f)),
                    "cy": int(round(cy_f)),
                    "kind": "obb",
                    "draw": {
                        "poly": obb.xyxyxyxy[i].reshape(4, 2).cpu().numpy().astype(np.int32)
                        if has_xyxyxyxy else None,
                        "xyxy": obb.xyxy[i].cpu().numpy() if hasattr(obb, "xyxy") and obb.xyxy is not None else None,
                    },
                }
                if best is None or conf > best["conf"]:
                    best = cand

            return best

        # 2) Normal boxes
        if r.boxes is not None and len(r.boxes) > 0:
            for b in r.boxes:
                cls = int(b.cls.item())
                if cls != self.apple_class_id:
                    continue
                conf = float(b.conf.item())
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
                cx = int(round((x1 + x2) / 2.0))
                cy = int(round((y1 + y2) / 2.0))
                cand = {
                    "conf": conf,
                    "cx": cx,
                    "cy": cy,
                    "kind": "box",
                    "draw": {"xyxy": (int(x1), int(y1), int(x2), int(y2))},
                }
                if best is None or conf > best["conf"]:
                    best = cand

        return best

    def _draw_detection(self, img, det):
        if det is None:
            return
        if det["kind"] == "obb":
            poly = det["draw"].get("poly", None)
            if poly is not None:
                cv2.polylines(img, [poly], isClosed=True, color=(0, 255, 0), thickness=2)
            else:
                xyxy = det["draw"].get("xyxy", None)
                if xyxy is not None:
                    x1, y1, x2, y2 = [int(v) for v in xyxy.tolist()]
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            x1, y1, x2, y2 = det["draw"]["xyxy"]
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cx, cy = det["cx"], det["cy"]
        cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)

    # ---- main loop step ----
    def step(self):
        rclpy.spin_once(self.img_node, timeout_sec=0.0)

        img = self.img_node.get_color_frame()
        depth = self.img_node.get_depth_frame()
        if img is None or depth is None:
            return None

        # Optional enhancement BEFORE YOLO
        img_for_yolo = enhance_bgr(img) if (COLOR_ENHANCE and APPLY_ENHANCE_TO_MODEL) else img

        # YOLO inference
        results = self.model.predict(img_for_yolo, conf=CONF_THRES, verbose=False)
        if not results:
            return img

        r = results[0]
        det = self._select_best_detection(r)

        now = time.monotonic()

        if det is None:
            # not seen -> reset if gap too large
            if self.last_seen_t is not None and (now - self.last_seen_t) > MAX_GAP_SEC:
                self.hold_start_t = None
                self.last_seen_t = None
                self.last_center = None
            return img

        conf = float(det['conf'])
        cx = int(det['cx'])
        cy = int(det['cy'])

        # clamp to image bounds for depth access
        h_img, w_img = img.shape[:2]
        cx = clamp(cx, 0, w_img - 1)
        cy = clamp(cy, 0, h_img - 1)

        # draw detection (supports OBB polygon)
        self._draw_detection(img, det)

        # hold logic
        if self.last_seen_t is None or (now - self.last_seen_t) > MAX_GAP_SEC:
            self.hold_start_t = now
        self.last_seen_t = now
        self.last_center = (cx, cy)

        held = now - (self.hold_start_t or now)
        cv2.putText(img, f"{TARGET_NAME} conf={conf:.2f} hold={held:.2f}s",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # trigger
        if held >= DETECT_HOLD_SEC:
            # get depth at center (retry a bit if invalid)
            z = self.get_depth_value(cx, cy, depth)
            retry = 0
            while (z is None or z == 0) and retry < 10:
                rclpy.spin_once(self.img_node, timeout_sec=0.0)
                depth = self.img_node.get_depth_frame()
                if depth is not None:
                    z = self.get_depth_value(cx, cy, depth)
                retry += 1
                time.sleep(0.03)

            if z is None or z == 0:
                self.get_logger().warn("Depth invalid. Skipping this trigger.")
                self.hold_start_t = None
                self.last_seen_t = None
                self.last_center = None
                return img

            # If depth topic is meters (float) instead of mm, convert
            z_val = float(z)
            if z_val < 10.0:  # heuristic: likely meters
                z_val *= 1000.0

            cam_pos = self.get_camera_pos(cx, cy, z_val, self.intrinsics)
            base_xyz = self.transform_to_base(cam_pos)

            # 접근축 결정 (카메라 정렬 기반)
            axis, sign, conf2, method = self.choose_approach_axis(base_xyz)

            self.get_logger().info(
                f"TRIGGER: {TARGET_NAME} held {held:.2f}s -> pixel=({cx},{cy}) depth={z_val:.1f} "
                f"-> base={base_xyz} -> approach={axis}{'+' if sign > 0 else '-'} (by {method})"
            )

            # reset hold state BEFORE moving (so after motion it can detect again)
            self.hold_start_t = None
            self.last_seen_t = None
            self.last_center = None

            # perform pick
            self.pick_sequence(base_xyz, axis, sign)

            self.done_once = True
            return img


        return img


if __name__ == "__main__":
    rclpy.init()

    # Doosan node init (same pattern as your test.py)
    node = rclpy.create_node("dsr_example_demo_py", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import get_current_posx, movej, movel, wait
        from DR_common2 import posx, posj
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        raise

    cv2.namedWindow("Webcam", cv2.WINDOW_NORMAL)

    app = ApplePickNode()

    J_align = [-16.229, 20.985, 116.567, 103.044, -51.537, 40.891]  # 원하는 초기 조인트각(도)로 바꾸세요
    movej(J_align, vel=VELOCITY, acc=ACC)
    wait(0.5)


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
        if app.done_once:
            break
        if frame is not None:
            cv2.imshow("Webcam", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()
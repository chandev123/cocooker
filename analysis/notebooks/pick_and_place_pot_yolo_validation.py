#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pot handle pick with Joint-6 pre-alignment WITHOUT re-detecting GRIP after rotation.

Why this version:
- You pointed out a real failure mode: after rotating J6, YOLO can miss the handle,
  so "re-detect GRIP after alignment" is brittle.
- Here we do:
  (A) detect once -> get (GRIP pixel, direction) -> compute GRIP 3D in BASE immediately
  (B) rotate J6 so the arrow becomes image-UP (open-loop; no re-detect required)
  (C) move to the cached BASE target with the new orientation and grip

Assumptions:
- The camera is wrist/tool mounted so J6 rotation causes an in-plane image rotation.
- The only thing we need from post-rotation perception is optional verification (disabled by default).
- TCP/hand-eye are consistent with your previous apple_pick_yolo pattern.

If J6 rotates the wrong way, set J6_IMAGE_SIGN = -1.0
"""

import time
import os
import sys
import math
from pathlib import Path
import pathlib

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
# User parameters
# -----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_BASE = 0
DR_MV_MOD_REL = 1

# Seg model
MODEL_FILENAME = resolve_model_path("potandhandle.pt")
CONF_THRES = 0.35

# Detection hold (seconds)
DETECT_HOLD_SEC = 1.2
MAX_GAP_SEC = 0.35

# J6 alignment (image arrow -> straight up)
ANGLE_TOL_DEG = 7.0
MAX_ALIGN_STEPS = 6          # open-loop stepping (no re-detect)
J6_STEP_CLAMP_DEG = 45.0     # max per movej in relative mode
J6_IMAGE_SIGN = +1.0         # if it rotates the wrong way, set -1.0
ALLOW_VERTICAL_BOTH = False  # True: allow up OR down if it's closer

# Motion / gripper tuning
VELOCITY, ACC = 60, 60
J6_VEL, J6_ACC = 30, 60

PREAPPROACH_DZ = 90     # mm above target
PICK_DZ = 25            # mm below target (down)
RETREAT_DZ = 140        # mm up after gripping
GRIP_WAIT = 2.0

RETURN_HOME_AFTER = True
HOME_J = [0, 0, 90, 0, 90, 0]

# Optional post-rotation verify (does NOT affect GRIP point)
VERIFY_AFTER_ROTATION = False

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"


# -----------------------------
# Doosan init
# -----------------------------
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def _normalize_angle_rad(a: float) -> float:
    """Normalize to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def overlay_mask(frame, mask_bin, color=(0, 0, 255), alpha=0.30):
    overlay = frame.copy()
    overlay[mask_bin == 1] = color
    return cv2.addWeighted(frame, 1 - alpha, overlay, alpha, 0)


def compute_grip_from_handle_mask(mask_bin: np.ndarray, offset_px: int = 35):
    """
    PCA on handle mask points -> main axis.
    Returns: grip_pt, root_pt, direction_unit (2D unit, image coords: x right, y down)

    개선점:
    - root + direction*offset 로 만든 '초기 grip'을
      손잡이의 폭 방향(법선)으로 "정중앙(centerline)"에 스냅시켜
      아래/위로 치우치는 문제를 줄임.
    """
    ys, xs = np.where(mask_bin == 1)
    if len(xs) < 120:
        return None, None, None

    pts = np.column_stack((xs, ys)).astype(np.float32)

    # PCA
    mean = pts.mean(axis=0)
    pts_c = pts - mean
    cov = np.cov(pts_c, rowvar=False)
    eigvals, eigvecs = np.linalg.eig(cov)

    direction = eigvecs[:, np.argmax(eigvals)].astype(np.float32)
    direction /= (np.linalg.norm(direction) + 1e-9)

    # endpoints along main axis
    proj = pts_c @ direction
    p_min = pts[np.argmin(proj)]
    p_max = pts[np.argmax(proj)]

    # choose root closer to image center (arrow points away from center)
    h, w = mask_bin.shape
    img_center = np.array([w / 2, h / 2], dtype=np.float32)
    if np.linalg.norm(p_min - img_center) < np.linalg.norm(p_max - img_center):
        root = p_min
        direction_unit = direction
    else:
        root = p_max
        direction_unit = -direction

    # 1) initial grip along axis
    grip0 = root + direction_unit * float(offset_px)

    # 2) "centerline snap": move along normal so that grip lies at the center of handle width
    # normal n = [-dy, dx]
    n = np.array([-direction_unit[1], direction_unit[0]], dtype=np.float32)
    n /= (np.linalg.norm(n) + 1e-9)

    # project all points to (t along axis) and (s along normal)
    # We want points whose t is near grip0's t, then take median s (center of width)
    t0 = float((grip0 - mean) @ direction_unit)

    t = pts_c @ direction_unit         # along-axis
    s = pts_c @ n                      # across-axis (width)

    # take a thin slice around t0
    band = 14.0  # pixels; tune 10~20
    idx = np.where(np.abs(t - t0) < band)[0]

    if idx.size >= 40:
        s_med = float(np.median(s[idx]))
        # snapped grip = mean + direction*t0 + n*s_med
        grip = mean + direction_unit * t0 + n * s_med
    else:
        # fallback: no slice points -> keep grip0
        grip = grip0

    return tuple(grip.astype(int)), tuple(root.astype(int)), direction_unit.astype(np.float32)


class PotHandlePickNode(Node):
    def __init__(self):
        super().__init__("pot_handle_pick_node_noredetect")

        self.base_dir = Path(__file__).resolve().parent

        # Camera node
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
            raise RuntimeError("No camera intrinsics. Check camera_info topic from ImgNode.")

        # Hand-eye (gripper->camera) : always load relative to this script folder
        he_path = pathlib.Path(resolve_resource_path("T_gripper2camera.npy"))
        self.gripper2cam = np.load(str(he_path))

        # Gripper
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        try:
            self.gripper.open_gripper()
        except Exception as e:
            self.get_logger().warn(f"gripper open failed (ignored): {e}")

        # YOLO segmentation
        from ultralytics import YOLO
        model_path = self.base_dir / MODEL_FILENAME
        self.model = YOLO(str(model_path))
        self.names = self.model.names

        # Hold state
        self.hold_start_t = None
        self.last_seen_t = None

        self.done_once = False

        self.get_logger().info("Ready. Holding condition: handle >= %.1fs" % DETECT_HOLD_SEC)
        self.get_logger().info("This version caches GRIP (base_xyz) BEFORE rotating J6. No GRIP re-detect after rotation.")
        self.get_logger().info("If J6 rotates the wrong way, set J6_IMAGE_SIGN=-1.0")

    # ---- geometry helpers ----
    def get_camera_pos(self, px: int, py: int, z_mm: float):
        intr = self.intrinsics
        cx = float(px)
        cy = float(py)
        camera_x = (cx - intr["ppx"]) * z_mm / intr["fx"]
        camera_y = (cy - intr["ppy"]) * z_mm / intr["fy"]
        camera_z = z_mm
        return (camera_x, camera_y, camera_z)

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        Rm = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, posx_before=None):
        """
        Transform a 3D point in camera frame into base frame using:
        T_base_cam = T_base_gripper(current) @ T_gripper_cam (hand-eye)
        If posx_before is given, use that pose instead of current (safer for our cache-before-rotate).
        """
        coord = np.append(np.array(camera_coords, dtype=np.float32), 1.0)
        if posx_before is None:
            base2gripper = self.get_robot_pose_matrix(*get_current_posx()[0])
        else:
            base2gripper = self.get_robot_pose_matrix(*posx_before)
        base2cam = base2gripper @ self.gripper2cam
        td_coord = base2cam @ coord
        return td_coord[:3]

    def depth_median(self, x: int, y: int, depth_frame: np.ndarray, k: int = 5):
        h, w = depth_frame.shape
        r = k // 2
        x0, x1 = max(0, x - r), min(w, x + r + 1)
        y0, y1 = max(0, y - r), min(h, y + r + 1)
        patch = depth_frame[y0:y1, x0:x1].astype(np.float32).reshape(-1)
        patch = patch[patch > 0]
        if patch.size == 0:
            return None
        z = float(np.median(patch))
        if z < 10.0:  # meters -> mm heuristic
            z *= 1000.0
        return z

    # ===========================
    # ✅ (추가) 2번 개선: handle 마스크 ROI 기반 robust depth
    # ===========================
    def depth_from_handle_roi(self, gx: int, gy: int,
                              depth_frame: np.ndarray,
                              handle_mask: np.ndarray,
                              r: int = 12):
        """
        Robust depth from handle mask ROI around GRIP.
        - gx, gy : GRIP pixel (color-aligned)
        - depth_frame : aligned depth image
        - handle_mask : merged handle binary mask
        - r : ROI half-size
        Returns depth in mm or None
        """
        if handle_mask is None:
            return None

        h, w = depth_frame.shape
        x0, x1 = max(0, gx - r), min(w, gx + r + 1)
        y0, y1 = max(0, gy - r), min(h, gy + r + 1)

        depth_roi = depth_frame[y0:y1, x0:x1].astype(np.float32)
        mask_roi = handle_mask[y0:y1, x0:x1]

        vals = depth_roi[(mask_roi == 1) & (depth_roi > 0)]
        if vals.size < 30:
            return None

        z = float(np.median(vals))

        # meters -> mm (32FC1 대응)
        if z < 10.0:
            z *= 1000.0

        return z

    # ---- perception ----
    def detect_handle(self, img: np.ndarray):
        res = self.model(img, conf=CONF_THRES)[0]
        merged_handle = None

        if res.masks is None or res.boxes is None:
            return None, None, None, None, img

        masks = res.masks.data.cpu().numpy()
        classes = res.boxes.cls.cpu().numpy().astype(int)
        confs = res.boxes.conf.cpu().numpy()

        vis = img.copy()

        for m, c, cf in zip(masks, classes, confs):
            label = str(self.names.get(int(c), int(c)))
            mask_bin = (m > 0.5).astype(np.uint8)

            if "handle" in label.lower():
                merged_handle = mask_bin if merged_handle is None else np.maximum(merged_handle, mask_bin)
                color = (0, 0, 255)
            elif "body" in label.lower() or "pot" in label.lower():
                color = (0, 255, 0)
            else:
                color = (0, 255, 255)

            vis = overlay_mask(vis, mask_bin, color=color, alpha=0.25)

        if merged_handle is None:
            return None, None, None, None, vis

        grip_pt, root_pt, direction = compute_grip_from_handle_mask(merged_handle, offset_px=35)

        if grip_pt is not None and root_pt is not None and direction is not None:
            cv2.circle(vis, root_pt, 6, (0, 255, 255), -1)
            cv2.putText(vis, "ROOT", (root_pt[0] + 6, root_pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.circle(vis, grip_pt, 7, (0, 0, 255), -1)
            cv2.putText(vis, "GRIP", (grip_pt[0] + 6, grip_pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            dir_vec = (direction * 60.0).astype(int)
            arrow_end = (root_pt[0] + int(dir_vec[0]), root_pt[1] + int(dir_vec[1]))
            cv2.arrowedLine(vis, root_pt, arrow_end, (255, 0, 255), 2, tipLength=0.2)

        return merged_handle, grip_pt, root_pt, direction, vis

    # ---- J6 alignment (open-loop, no re-detect) ----
    def compute_total_j6_delta_deg(self, direction_unit: np.ndarray) -> float:
        """
        direction_unit: 2D unit in image coords (x right, y down)
        Goal: make it point UP (0, -1) -> angle = -90 deg in atan2(y,x) convention.
        Returns TOTAL delta in degrees (can be > clamp).
        """
        dx = float(direction_unit[0])
        dy = float(direction_unit[1])
        cur = math.atan2(dy, dx)        # image angle
        desired_up = -math.pi / 2.0     # up = (0, -1)

        if ALLOW_VERTICAL_BOTH:
            desired_down = +math.pi / 2.0
            d_up = _normalize_angle_rad(desired_up - cur)
            d_dn = _normalize_angle_rad(desired_down - cur)
            delta = d_up if abs(d_up) <= abs(d_dn) else d_dn
        else:
            delta = _normalize_angle_rad(desired_up - cur)

        return float(math.degrees(delta) * float(J6_IMAGE_SIGN))

    def rotate_j6_openloop(self, total_delta_deg: float) -> None:
        """
        Apply total_delta_deg using multiple relative movej steps, without any vision re-detect.
        """
        remain = float(total_delta_deg)

        for i in range(MAX_ALIGN_STEPS):
            if abs(remain) <= ANGLE_TOL_DEG:
                break

            step = max(-J6_STEP_CLAMP_DEG, min(J6_STEP_CLAMP_DEG, remain))
            self.get_logger().info(f"[J6 openloop {i+1}/{MAX_ALIGN_STEPS}] step={step:+.1f} deg (remain={remain:+.1f})")

            movej([0.0, 0.0, 0.0, 0.0, 0.0, float(step)],
                  vel=J6_VEL, acc=J6_ACC, mod=DR_MV_MOD_REL)
            wait(0.2)

            remain -= step

        if abs(remain) > ANGLE_TOL_DEG:
            self.get_logger().warn(f"J6 openloop remaining delta still {remain:+.1f} deg (tolerance {ANGLE_TOL_DEG} deg). Increase MAX_ALIGN_STEPS or clamp.")

    # ---- robot action ----
    def pick_sequence(self, base_xyz):
        x, y, z = float(base_xyz[0]), float(base_xyz[1]), float(base_xyz[2])

        # Use current orientation AFTER J6 rotation
        current_pos = get_current_posx()[0]
        rx, ry, rz = current_pos[3], current_pos[4], current_pos[5]

        pre = posx([x, y, z + PREAPPROACH_DZ, rx, ry, rz])
        pick = posx([x, y, z - PICK_DZ,        rx, ry, rz])
        retreat = posx([x, y, z + RETREAT_DZ,  rx, ry, rz])

        movel(pre, vel=VELOCITY, acc=ACC)
        movel(pick, vel=VELOCITY, acc=ACC)

        self.gripper.close_gripper()
        wait(GRIP_WAIT)

        movel(retreat, vel=VELOCITY, acc=ACC)

        movel(posx([0.0, 0.0, 50.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, 
          ref=DR_BASE, mod=DR_MV_MOD_REL)
        wait(0.5)

        movej([-90.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=VELOCITY, acc=ACC, 
          mod=DR_MV_MOD_REL)
        wait(0.5)

        width = self.gripper.get_width()
        if width > 12 :
            print(f'집기에 성공했습니다 {width}' )
        else :
            print(f'집기에 실패했습니다. {width}')

        

    # ---- main loop ----
    def step(self):
        rclpy.spin_once(self.img_node, timeout_sec=0.0)

        img = self.img_node.get_color_frame()
        depth = self.img_node.get_depth_frame()
        if img is None or depth is None:
            return None

        # ✅ (수정) handle_mask를 받아옴
        handle_mask, grip_pt, root_pt, direction, vis = self.detect_handle(img)

        now = time.monotonic()
        seen = (direction is not None and grip_pt is not None)

        if not seen:
            if self.last_seen_t is not None and (now - self.last_seen_t) > MAX_GAP_SEC:
                self.hold_start_t = None
                self.last_seen_t = None
            return vis

        if self.last_seen_t is None or (now - self.last_seen_t) > MAX_GAP_SEC:
            self.hold_start_t = now
        self.last_seen_t = now

        held = now - (self.hold_start_t or now)
        cv2.putText(vis, f"handle hold={held:.2f}s", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        if held < DETECT_HOLD_SEC:
            return vis

        # ---- TRIGGER ----
        self.hold_start_t = None
        self.last_seen_t = None

        gx, gy = int(grip_pt[0]), int(grip_pt[1])

        # ✅ (수정) GRIP 한 점 depth가 아니라 handle 마스크 ROI에서 robust하게 depth 추출
        z_mm = self.depth_from_handle_roi(gx, gy, depth, handle_mask, r=12)
        if z_mm is None:
            self.get_logger().warn("Depth invalid in HANDLE ROI near GRIP. Skipping.")
            return vis

        # (선택) 화면에 depth 표시
        cv2.putText(vis, f"z={z_mm:.1f}mm", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Cache robot pose BEFORE rotating J6 (for consistent base->cam)
        posx_before = get_current_posx()[0]

        cam_pos = self.get_camera_pos(gx, gy, z_mm)
        base_xyz = self.transform_to_base(cam_pos, posx_before=posx_before)

        total_delta_deg = self.compute_total_j6_delta_deg(direction)

        self.get_logger().info(
            f"TRIGGER: held {held:.2f}s | GRIP px=({gx},{gy}) depth={z_mm:.1f} -> base={base_xyz} | J6_delta={total_delta_deg:+.1f}deg"
        )

        # Rotate J6 WITHOUT re-detecting GRIP afterwards
        self.rotate_j6_openloop(total_delta_deg)

        # Optional sanity-check: only verify direction, not re-pick GRIP
        if VERIFY_AFTER_ROTATION:
            rclpy.spin_once(self.img_node, timeout_sec=0.0)
            img2 = self.img_node.get_color_frame()
            if img2 is not None:
                _, _, _, dir2, _ = self.detect_handle(img2)
                if dir2 is not None:
                    dx, dy = float(dir2[0]), float(dir2[1])
                    ang = math.degrees(math.atan2(dy, dx))
                    self.get_logger().info(f"Verify: direction angle after rot = {ang:.1f} deg (target -90 deg for UP).")
                else:
                    self.get_logger().warn("Verify: handle not detected after rotation (ignored).")

        # Execute pick with cached base_xyz
        self.pick_sequence(base_xyz)

        self.done_once = True

        return vis


if __name__ == "__main__":
    rclpy.init()

    node = rclpy.create_node("dsr_example_pot_handle_pick_py", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import get_current_posx, movej, movel, wait, DR_MV_MOD_REL
        from DR_common2 import posx
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 / DR_common2 : {e}")
        raise

    cv2.namedWindow("PotHandle", cv2.WINDOW_NORMAL)

    app = PotHandlePickNode()

    J_align = [-90.962, 2.308, 56.362, -1.586, 92.434, 88.283]  # 원하는 초기 조인트각(도)로 바꾸세요
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
            cv2.imshow("PotHandle", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()
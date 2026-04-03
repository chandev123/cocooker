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
import csv

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
        self.fail_dir = "fail_images"
        os.makedirs(self.fail_dir, exist_ok=True) # 폴더 없으면 생성

        print('gripper1')
        self.gripper.open_gripper()
        wait(1.0)
        print('gripper2')
        self.gripper.close_gripper()
        wait(1.0)
        print('gripper3')
        self.gripper.open_gripper()
        print('gripper4')

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
        max_conf = 0.0  # [추가] 최대 신뢰도 저장 변수

        if res.masks is None or res.boxes is None:
            return None, None, None, None, 0.0, img  # 반환값 개수 변경 (conf 추가)

        masks = res.masks.data.cpu().numpy()
        classes = res.boxes.cls.cpu().numpy().astype(int)
        confs = res.boxes.conf.cpu().numpy()

        vis = img.copy()

        for m, c, cf in zip(masks, classes, confs):
            label = str(self.names.get(int(c), int(c)))
            mask_bin = (m > 0.5).astype(np.uint8)

            if "handle" in label.lower():
                merged_handle = mask_bin if merged_handle is None else np.maximum(merged_handle, mask_bin)
                max_conf = max(max_conf, float(cf))  # [추가] 핸들 신뢰도 갱신
                color = (0, 0, 255)
            elif "body" in label.lower() or "pot" in label.lower():
                color = (0, 255, 0)
            else:
                color = (0, 255, 255)

            vis = overlay_mask(vis, mask_bin, color=color, alpha=0.25)

        if merged_handle is None:
            return None, None, None, None, 0.0, vis

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
            
            # 화면에 신뢰도 표시
            cv2.putText(vis, f"Conf: {max_conf:.2f}", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return merged_handle, grip_pt, root_pt, direction, max_conf, vis

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
        print('pick1')
        x, y, z = float(base_xyz[0]), float(base_xyz[1]), float(base_xyz[2])

        # Use current orientation AFTER J6 rotation
        current_pos = get_current_posx()[0]
        rx, ry, rz = current_pos[3], current_pos[4], current_pos[5]

        pre = posx([x, y, z + PREAPPROACH_DZ, rx, ry, rz])
        pick = posx([x, y, z - PICK_DZ,        rx, ry, rz])
        retreat = posx([x, y, z + RETREAT_DZ,  rx, ry, rz])
        print('pick2')
        movel(pre, vel=VELOCITY, acc=ACC)
        print('pick3')
        movel(pick, vel=VELOCITY, acc=ACC)
        print('pick4')
        self.gripper.close_gripper()
        wait(GRIP_WAIT)

        # movel(retreat, vel=VELOCITY, acc=ACC)

        # movel(posx([0.0, 0.0, 50.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, 
        #   ref=DR_BASE, mod=DR_MV_MOD_REL)
        # wait(0.5)

        # movej([-90.0, 0.0, 0.0, 0.0, 0.0, 0.0], vel=VELOCITY, acc=ACC, 
        #   mod=DR_MV_MOD_REL)
        # wait(0.5)

        # width = self.gripper.get_width()
        # if width > 12 :
        #     print(f'집기에 성공했습니다 {width}' )
        # else :
        #     print(f'집기에 실패했습니다. {width}')
        # print('pick5')
                
    def save_data(self, record):
        """
        [수정] 실시간으로 CSV 파일에 데이터를 한 줄씩 추가합니다.
        저장 항목: 1.객체인식 성공여부, 2.인식 시간, 3.정확도, 4.파지 성공여부, 5.동작 시간
        """
        csv_filename = 'pot_pick_data.csv'
        file_exists = os.path.isfile(csv_filename)
        
        try:
            # 'a' 모드로 열어서 데이터를 끝에 추가 (Append)
            with open(csv_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                
                # 파일이 없거나 비어있으면 헤더(제목)부터 작성
                if not file_exists or os.stat(csv_filename).st_size == 0:
                    writer.writerow([
                        'Trial', 
                        'Detect_Success',   # 1. 객체인식 성공여부
                        'Detect_Time',      # 2. 인식까지 걸린 시간
                        'Confidence',       # 3. 인식 정확도
                        'Grasp_Success',    # 4. 파지 성공여부
                        'Motion_Time',      # 5. 파지 동작시작~파지 시간
                        'Grip_Width'        # (참고용) 그리퍼 너비
                    ])
                    
                # 데이터 작성
                writer.writerow([
                    record['trial'],
                    1,                      # Detect_Success (여기까지 왔으면 무조건 성공)
                    record['detect_time'],  # Detect_Time
                    record['conf'],         # Confidence
                    record['success'],      # Grasp_Success
                    record['motion_time'],  # Motion_Time
                    record['width']         # Grip_Width
                ])
                f.flush() # 버퍼 강제 비우기 (즉시 저장)
                os.fsync(f.fileno()) # OS 레벨 저장 동기화
                
            print(f">>> [저장 완료] Trial {record['trial']} 기록됨 (성공:{record['success']}, 시간:{record['detect_time']:.1f}s+{record['motion_time']:.1f}s)")
            
        except Exception as e:
            print(f">>> [저장 실패] CSV 쓰기 오류: {e}")

    # ---- main loop ----
    def step(self):
        rclpy.spin_once(self.img_node, timeout_sec=0.0)

        # [Time] 시도 시작
        if not hasattr(self, 'trial_start_t') or self.trial_start_t is None:
            self.trial_start_t = time.monotonic()
        
        # [Safety] 초기화
        if not hasattr(self, 'fail_dir'):
            self.fail_dir = "fail_images"
            os.makedirs(self.fail_dir, exist_ok=True)
        if not hasattr(self, 'last_fail_save_t'):
            self.last_fail_save_t = None

        img = self.img_node.get_color_frame()
        depth = self.img_node.get_depth_frame()
        if img is None or depth is None:
            return None

        # 1. 객체 인식
        handle_mask, grip_pt, root_pt, direction, conf, vis = self.detect_handle(img)

        now = time.monotonic()
        seen = (direction is not None and grip_pt is not None)
        elapsed_time = now - self.trial_start_t 

        # [Timeout] 3초 Blind Pick 로직
        VISION_TIMEOUT = 3.0
        is_blind_run = False
        
        if not seen:
            if elapsed_time > VISION_TIMEOUT:
                print(f">>> [Vision Timeout] 3초 경과! Blind Pick 모드로 진입합니다.")
                grip_pt = (320, 240) 
                root_pt = (320, 300)
                direction = np.array([0.0, -1.0])
                conf = 0.0
                handle_mask = None
                is_blind_run = True
                
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                try:
                    cv2.imwrite(f"{self.fail_dir}/blind_run_{timestamp}.jpg", img)
                except: pass
            else:
                if self.last_seen_t is not None and (now - self.last_seen_t) > MAX_GAP_SEC:
                    self.hold_start_t = None
                    self.last_seen_t = None
                
                remain = VISION_TIMEOUT - elapsed_time
                cv2.putText(vis, f"Searching... {remain:.1f}s", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                return vis

        if self.last_seen_t is None or (now - self.last_seen_t) > MAX_GAP_SEC:
            self.hold_start_t = now
        self.last_seen_t = now

        held = now - (self.hold_start_t or now)
        if not is_blind_run and held < DETECT_HOLD_SEC:
            return vis

        # ---- 동작 시작 ----
        detect_end_t = time.monotonic()
        detect_duration = detect_end_t - self.trial_start_t 

        self.hold_start_t = None
        self.last_seen_t = None

        if is_blind_run:
             z_mm = 400.0 
             gx, gy = int(grip_pt[0]), int(grip_pt[1]) 
             print(f">>> [Blind] 깊이 정보가 없어 기본값({z_mm}mm)을 사용합니다.")
        else:
            gx, gy = int(grip_pt[0]), int(grip_pt[1])
            z_mm = self.depth_from_handle_roi(gx, gy, depth, handle_mask, r=12)
        
        if z_mm is None:
            self.get_logger().warn("Depth invalid. Skipping.")
            return vis

        motion_start_t = time.monotonic()
        
        posx_before = get_current_posx()[0]
        cam_pos = self.get_camera_pos(gx, gy, z_mm)
        base_xyz = self.transform_to_base(cam_pos, posx_before=posx_before)
        total_delta_deg = self.compute_total_j6_delta_deg(direction)

        self.rotate_j6_openloop(total_delta_deg)
        
        # [Pick] 가서 집고 들어올림
        self.pick_sequence(base_xyz) 
        
        motion_end_t = time.monotonic()
        motion_duration = motion_end_t - motion_start_t 

        # -------------------------------------------------------------
        # [측정 및 놓기]
        # -------------------------------------------------------------
        width = float(self.gripper.get_width()) 
        is_success = 1 if width > 12 else 0 
        
        print(f">>> [동작 완료] Width:{width:.2f}mm 측정됨. 냄비를 놓아줍니다.")
        self.gripper.open_gripper()
        wait(0.5) 
        
        # -------------------------------------------------------------
        # [NEW] 원점 복귀(Place) 시간 측정
        # -------------------------------------------------------------
        place_start_t = time.monotonic()
        
        # 원점 위치 (기존 reset_robot_position에 있던 좌표)
        J_align = [-90.962, 2.308, 56.362, -1.586, 92.434, 88.283]
        movej(J_align, vel=VELOCITY, acc=ACC)
        wait(0.5)
        
        place_end_t = time.monotonic()
        place_duration = place_end_t - place_start_t
        # -------------------------------------------------------------

        # 결과 기록 (place_ms 추가)
        if not hasattr(self, 'history'): self.history = []
        
        record = {
            'trial': len(self.history) + 1,
            'success': is_success,          
            'detect_ms': detect_duration * 1000.0,
            'motion_ms': motion_duration * 1000.0,
            'place_ms':  place_duration * 1000.0,   # [추가] 복귀 시간
            'conf': float(conf),            
            'width': width,
            'detect_success': 0 if is_blind_run else 1
        }
        self.history.append(record)
        self.save_data_custom(record)

        # 다음 시도 준비
        TARGET_TRIALS = 100
        if len(self.history) < TARGET_TRIALS:
            print(">>> Resetting Variables...")
            # 변수 초기화
            self.trial_start_t = time.monotonic() 
            self.hold_start_t = None
            self.last_seen_t = None
            self.done_once = False 
        else:
            print(f">>> {TARGET_TRIALS} Trials Completed!")
            self.done_once = True
            
        return vis

    # ---- [Helper] 리셋 및 초기화 함수 ----
    def reset_robot_position(self):
        TARGET_TRIALS = 100
        if len(self.history) < TARGET_TRIALS:
            print(">>> Resetting Robot...")
            self.gripper.open_gripper()
            wait(0.5)
            # 초기 위치로 이동 (팔을 뺐다가 다시 오게 함)
            # J_align은 main이나 전역변수에서 가져옴
            J_align = [-90.962, 2.308, 56.362, -1.586, 92.434, 88.283]
            movej(J_align, vel=VELOCITY, acc=ACC)
            wait(0.5)
            
            # 다음 시도를 위해 시간 초기화
            self.trial_start_t = time.monotonic() 
            self.hold_start_t = None
            self.last_seen_t = None
            self.done_once = False 
        else:
            print(f">>> {TARGET_TRIALS} Trials Completed!")
            self.done_once = True

    # ---- [Helper] 데이터 저장 함수 (단위: ms) ----
    def save_data_custom(self, record):
        csv_filename = 'pot_pick_data.csv'
        file_exists = os.path.isfile(csv_filename)
        try:
            with open(csv_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                # 헤더에 단위 명시 (ms)
                if not file_exists or os.stat(csv_filename).st_size == 0:
                    writer.writerow(['Trial', 'Detect_Success', 'Detect_Time(ms)', 'Confidence', 'Grasp_Success', 'Motion_Time(ms)', 'Grip_Width'])
                
                writer.writerow([
                    record['trial'],
                    record.get('detect_success', 0), 
                    f"{record['detect_ms']:.2f}",   # 밀리초 단위이므로 소수점 2자리만 해도 충분 (예: 53.42 ms)
                    f"{record['conf']:.4f}",        
                    record['success'],
                    f"{record['motion_ms']:.2f}",   # 예: 2100.50 ms
                    f"{record['width']:.4f}"        # 너비는 정밀하게 4자리
                ])
                f.flush()
                os.fsync(f.fileno())
            
            print(f">>> [기록] Trial {record['trial']} | Time: {record['detect_ms']:.1f}ms + {record['motion_ms']:.1f}ms | Width: {record['width']:.2f}")
        except Exception as e:
            print(f">>> [에러] CSV 저장 실패: {e}")

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

    # 1. 초기 위치 이동 (시작 전 한 번만 수행)
    J_align = [-90.962, 2.308, 56.362, -1.586, 92.434, 88.283]  
    movej(J_align, vel=VELOCITY, acc=ACC)
    wait(0.5)

    # --------------------------------------------------------------------------
    # [수정] Timeout 관련 변수 제거
    # 원래 있던 t0, timeout 로직은 1회성 실행을 위한 것이므로 100회 반복 시에는 방해가 됩니다.
    # --------------------------------------------------------------------------
    # timeout = float(os.environ.get('KITCHEN_DETECT_TIMEOUT_SEC', '12.0')) 
    # t0 = time.monotonic() 
    obj_name = globals().get('TARGET_NAME', 'object')

    print(">>> 100회 반복 테스트를 시작합니다. (Press ESC to stop manually)")

    while True:
        # [수정] 타임아웃 체크 로직 주석 처리 (이게 살아있으면 12초 뒤에 꺼짐)
        # if (time.monotonic() - t0) > timeout and not getattr(app, "done_once", False):
        #     print(f"[TIMEOUT] {obj_name} not detected for {timeout:.1f}s")
        #     ... (종료 코드) ...
        #     sys.exit(2)

        # 2. step() 반복 호출
        # step() 내부에서 100회 미만일 때는 app.done_once = False를 유지하므로
        # 이 while 루프는 계속 돕니다.
        frame = app.step()

        # 3. 100회 완료 후 종료 조건
        # step() 내부에서 100회가 끝나면 app.done_once = True로 바뀝니다.
        if app.done_once:
            print(">>> 모든 테스트가 완료되었습니다. 프로그램을 종료합니다.")
            break

        if frame is not None:
            cv2.imshow("PotHandle", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC 키를 누르면 강제 종료
            print(">>> 사용자에 의해 강제 종료되었습니다.")
            # 강제 종료 시에도 현재까지의 데이터 저장 시도 (선택 사항)
            if hasattr(app, 'save_plot'):
                app.save_plot() 
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()
#!/usr/bin/env python3
import time
import rclpy
import DR_init
from kitchen_assistant.utils import resolve_model_path, resolve_resource_path

from kitchen_assistant.onrobot import RG  # 업로드된 onrobot.py 사용


# -----------------------------
# USER CONFIG
# -----------------------------
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_OFF = 0
DR_ON = 1
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1
DR_QSTOP = 2
DR_BASE = 0
DR_TOOL = 1
DR_AXIS_Z = 2
DR_FC_MOD_REL = 1

# OnRobot Tool Changer / Modbus
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# Motion default
VELOCITY = 60
ACC = 60


# -----------------------------
# PLACEHOLDER: your motion code here
# -----------------------------
def do_motion_demo(movej, movel, posx, wait, gripper):
    """
    여기 안에 movej/movel을 마음대로 채워넣으세요.
    아래는 예시라서 주석 처리해둠.
    """

    # 예시 1) 안전 포즈 이동
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)
    wait(0.5)

    movej([14.799, 9.342, 124.528, 19.738, -46.149, 74.756], vel=VELOCITY, acc=ACC)
    wait(0.5)

    movel(posx([672.708, 286.156, 137.864, 179.972, -90.238, -90.173]), vel=VELOCITY, acc=ACC)
    wait(0.5)

    gripper.close_gripper()
    time.sleep(0.2)

    movel(posx([872.708, 286.156, 137.864, 179.972, -90.238, -90.173]), vel=VELOCITY, acc=ACC)
    wait(0.5)

    movel(posx([0.0, -155.0, 0.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, 
          ref=DR_BASE, mod=DR_MV_MOD_REL)
    wait(0.5)

    movel(posx([10.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, 
          ref=DR_BASE, mod=DR_MV_MOD_REL)
    wait(0.5)

    movel(posx([-100.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, 
          ref=DR_BASE, mod=DR_MV_MOD_REL)
    wait(0.5)

    gripper.open_gripper()
    time.sleep(0.2)

    
   


    




def do_gripper_demo(gripper):
    """
    그리퍼 테스트용. 원하면 지우세요.
    """
    try:
        print("[GRIPPER] open")
        gripper.open_gripper()
        time.sleep(1.0)

        print("[GRIPPER] close")
        gripper.close_gripper()
        time.sleep(1.0)

        print("[GRIPPER] open")
        gripper.open_gripper()
        time.sleep(1.0)
    except Exception as e:
        print("[GRIPPER] error:", e)


def main():
    # -----------------------------
    # ROS2 init (python3로 실행하더라도 rclpy는 필요)
    # -----------------------------
    rclpy.init()
    node = rclpy.create_node("robot_basic_run_node", namespace=ROBOT_ID)

    # Doosan DSR init
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node

    # Doosan API import (이후 movej/movel 호출 가능)
    from DSR_ROBOT2 import movej, movel, wait
    from DR_common2 import posx

    # -----------------------------
    # OnRobot gripper init
    # -----------------------------
    print("[INIT] Gripper connect...")
    gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

    # 그리퍼 초기 상태를 열어두고 시작(원하면 삭제)
    try:
        gripper.open_gripper()
        time.sleep(0.2)
    except Exception as e:
        print("[GRIPPER] open init error:", e)

    # -----------------------------
    # Optional: gripper test
    # -----------------------------
    # do_gripper_demo(gripper)

    # -----------------------------
    # Your motion code
    # -----------------------------
    print("[RUN] start motion demo (fill in do_motion_demo)")
    do_motion_demo(movej, movel, posx, wait, gripper)

    print("[DONE]")

    # -----------------------------
    # shutdown
    # -----------------------------
    rclpy.shutdown()


if __name__ == "__main__":
    main()

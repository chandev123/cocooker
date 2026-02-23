#!/usr/bin/env python3
"""Move Doosan robot via ROS2 services (dsr_msgs2), without importing DSR_ROBOT2.

This avoids DR_init / g_node issues and matches how `ros2 service call` works.
"""
import os
import sys
import argparse
import time

import rclpy


def _env_default(key: str, default: str) -> str:
    v = os.environ.get(key)
    if v is None:
        return default
    v = v.strip()
    return v if v else default


def _parse_joints(s: str):
    parts = [p.strip() for p in s.replace("[", "").replace("]", "").split(",") if p.strip() != ""]
    if len(parts) != 6:
        raise ValueError(f"Expected 6 joints, got {len(parts)} from: {s}")
    return [float(p) for p in parts]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joints", default=None, help="comma-separated 6 joint degrees, e.g. '0,0,0,0,0,0'")
    ap.add_argument("--home", action="store_true", help="call MoveHome instead of MoveJoint")
    ap.add_argument("--vel", type=float, default=float(_env_default("KITCHEN_HOME_VEL", "60")))
    ap.add_argument("--acc", type=float, default=float(_env_default("KITCHEN_HOME_ACC", "60")))
    ap.add_argument("--timeout", type=float, default=float(_env_default("KITCHEN_MOVEJ_TIMEOUT", "15.0")))
    ap.add_argument("--name", default="kitchen_movej")
    args = ap.parse_args()

    robot_id = _env_default("ROBOT_ID", "dsr01")

    srv_move_joint = f"/{robot_id}/motion/move_joint"
    srv_move_home  = f"/{robot_id}/motion/move_home"

    rclpy.init()
    node = None
    try:
        node = rclpy.create_node(args.name)

        if args.home:
            from dsr_msgs2.srv import MoveHome
            cli = node.create_client(MoveHome, srv_move_home)
            node.get_logger().info(f"[move] waiting service: {srv_move_home}")
            t0 = time.time()
            while not cli.wait_for_service(timeout_sec=1.0):
                if time.time() - t0 > args.timeout:
                    raise RuntimeError(f"Service not available: {srv_move_home}")
            req = MoveHome.Request()
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut)
            if fut.result() is None:
                raise RuntimeError(f"MoveHome call failed: {fut.exception()}")
            return 0

        if args.joints is None:
            raise ValueError("--joints is required unless --home is used")

        from dsr_msgs2.srv import MoveJoint
        cli = node.create_client(MoveJoint, srv_move_joint)
        node.get_logger().info(f"[move] waiting service: {srv_move_joint}")
        t0 = time.time()
        while not cli.wait_for_service(timeout_sec=1.0):
            if time.time() - t0 > args.timeout:
                raise RuntimeError(f"Service not available: {srv_move_joint}")

        joints = _parse_joints(args.joints)

        req = MoveJoint.Request()
        req.pos = joints
        req.vel = float(args.vel)
        req.acc = float(args.acc)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0

        node.get_logger().info(f"[move] MoveJoint pos={joints} vel={req.vel} acc={req.acc}")
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut)
        if fut.result() is None:
            raise RuntimeError(f"MoveJoint call failed: {fut.exception()}")
        return 0

    except Exception as e:
        try:
            if node:
                node.get_logger().error(str(e))
            else:
                print(e, file=sys.stderr)
        except Exception:
            print(e, file=sys.stderr)
        return 1
    finally:
        try:
            if node:
                node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

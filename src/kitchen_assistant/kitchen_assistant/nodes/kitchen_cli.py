#!/usr/bin/env python3
import sys
import subprocess

# Each legacy script creates/destroys its own rclpy context.
# If we run them in-process (runpy), DSR_ROBOT2's cached service clients
# can keep a dead context and crash on the next command.
# So we run each command in a fresh Python process: `python3 -m <module>`.

COMMANDS = {
    # kitchen items
    "cup": "kitchen_assistant.legacy.pick_and_place_cup_yolo",
    "컵": "kitchen_assistant.legacy.pick_and_place_cup_yolo",

    "ramen": "kitchen_assistant.legacy.pick_and_place_ramen_yolo",
    "라면": "kitchen_assistant.legacy.pick_and_place_ramen_yolo",

    "scissors": "kitchen_assistant.legacy.pick_and_place_scissors_yolo",
    "가위": "kitchen_assistant.legacy.pick_and_place_scissors_yolo",
    "주방가위": "kitchen_assistant.legacy.pick_and_place_scissors_yolo",

    "spatula": "kitchen_assistant.legacy.pick_and_place_spatula_yolo",
    "뒤집개": "kitchen_assistant.legacy.pick_and_place_spatula_yolo",

    "spoon": "kitchen_assistant.legacy.pick_and_place_spoon_yolo",
    "국자": "kitchen_assistant.legacy.pick_and_place_spoon_yolo",

    "knife": "kitchen_assistant.legacy.pick_and_place_knife",
    "칼": "kitchen_assistant.legacy.pick_and_place_knife",

    "pan": "kitchen_assistant.legacy.pick_and_place_pan_yolo",
    "프라이팬": "kitchen_assistant.legacy.pick_and_place_pan_yolo",
    "팬": "kitchen_assistant.legacy.pick_and_place_pan_yolo",

    "pot": "kitchen_assistant.legacy.pick_and_place_pot_yolo",
    "냄비": "kitchen_assistant.legacy.pick_and_place_pot_yolo",
}

# fridge sequences (open -> pick -> close)
FRIDGE_SEQ = {
    "apple": (
        "kitchen_assistant.legacy.fridge_rightdown_open",
        "kitchen_assistant.legacy.pick_and_place_apple_yolo",
        "kitchen_assistant.legacy.fridge_rightdown_close",
    ),
    "사과": (
        "kitchen_assistant.legacy.fridge_rightdown_open",
        "kitchen_assistant.legacy.pick_and_place_apple_yolo",
        "kitchen_assistant.legacy.fridge_rightdown_close",
    ),
    "orange": (
        "kitchen_assistant.legacy.fridge_leftdown_open",
        "kitchen_assistant.legacy.pick_and_place_orange_yolo",
        "kitchen_assistant.legacy.fridge_leftdown_close",
    ),
    "오렌지": (
        "kitchen_assistant.legacy.fridge_leftdown_open",
        "kitchen_assistant.legacy.pick_and_place_orange_yolo",
        "kitchen_assistant.legacy.fridge_leftdown_close",
    ),
    "bottle": (
        "kitchen_assistant.legacy.fridge_leftup_open",
        "kitchen_assistant.legacy.pick_and_place_bottle_yolo",
        "kitchen_assistant.legacy.fridge_leftup_close",
    ),
    "콜라": (
        "kitchen_assistant.legacy.fridge_leftup_open",
        "kitchen_assistant.legacy.pick_and_place_bottle_yolo",
        "kitchen_assistant.legacy.fridge_leftup_close",
    ),
    "페트병": (
        "kitchen_assistant.legacy.fridge_leftup_open",
        "kitchen_assistant.legacy.pick_and_place_bottle_yolo",
        "kitchen_assistant.legacy.fridge_leftup_close",
    ),
}

def _normalize(s: str) -> str:
    return s.strip()

def _pick_key(cmd: str):
    # fuzzy: if sentence contains one of keys
    for k in FRIDGE_SEQ.keys():
        if k in cmd:
            return k
    for k in COMMANDS.keys():
        if k in cmd:
            return k
    return None

def _run_module(mod: str) -> int:
    cmd = [sys.executable, "-m", mod]
    print(f"[CLI] exec: {' '.join(cmd)}")
    return subprocess.call(cmd)

def main():
    print("kitchen_cli: type object name (e.g., cup/컵, ramen/라면, apple/사과, pan/프라이팬).")
    print("exit / quit to stop.\n")

    while True:
        try:
            cmd = input("command> ")
        except EOFError:
            break

        cmd = _normalize(cmd)
        if not cmd:
            continue
        if cmd.lower() in ("exit", "quit", "q"):
            break

        key = _pick_key(cmd)
        if key is None:
            print(f"Unknown command: {cmd}")
            continue

        if key in FRIDGE_SEQ:
            for mod in FRIDGE_SEQ[key]:
                print(f"\n--- run: {mod} ---")
                rc = _run_module(mod)
                if rc != 0:
                    print(f"[CLI] module failed (rc={rc}): {mod}")
                    break
        else:
            mod = COMMANDS[key]
            print(f"\n--- run: {mod} ---")
            rc = _run_module(mod)
            if rc != 0:
                print(f"[CLI] module failed (rc={rc}): {mod}")

    print("bye")

if __name__ == "__main__":
    main()

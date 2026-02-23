from __future__ import annotations

import os
import sys
import subprocess
from dataclasses import dataclass, field
from typing import List, Optional

# Legacy modules are executed in separate python processes (python -m <module>).
# Return codes convention:
#   0 = success
#   2 = not_found (e.g. detect timeout)
#   other = failure

COMMANDS = {
    "cup": "kitchen_assistant.legacy.pick_and_place_cup_yolo",
    "ramen": "kitchen_assistant.legacy.pick_and_place_ramen_yolo",
    "scissors": "kitchen_assistant.legacy.pick_and_place_scissors_yolo",
    "spatula": "kitchen_assistant.legacy.pick_and_place_spatula_yolo",
    "spoon": "kitchen_assistant.legacy.pick_and_place_spoon_yolo",
    "knife": "kitchen_assistant.legacy.pick_and_place_knife",
    "pan": "kitchen_assistant.legacy.pick_and_place_pan_yolo",
    "pot": "kitchen_assistant.legacy.pick_and_place_pot_yolo",
}

FRIDGE_SEQ = {
    "apple": (
        "kitchen_assistant.legacy.fridge_rightdown_open",
        "kitchen_assistant.legacy.pick_and_place_apple_yolo",
        "kitchen_assistant.legacy.fridge_rightdown_close",
    ),
    "orange": (
        "kitchen_assistant.legacy.fridge_leftdown_open",
        "kitchen_assistant.legacy.pick_and_place_orange_yolo",
        "kitchen_assistant.legacy.fridge_leftdown_close",
    ),
    "bottle": (
        "kitchen_assistant.legacy.fridge_leftup_open",
        "kitchen_assistant.legacy.pick_and_place_bottle_yolo",
        "kitchen_assistant.legacy.fridge_leftup_close",
    ),
}

@dataclass
class RunResult:
    status: str  # ok | not_found | fail
    rc: int
    failed_module: str = ""
    post_actions: List[str] = field(default_factory=list)

def _is_transient_error(output: str) -> bool:
    # Typical intermittent Doosan/ROS2 state issues
    if "IndexError: list index out of range" in output and ("get_current_posx" in output or "DSR_ROBOT2.py" in output):
        return True
    if "RCLError" in output and "context is invalid" in output:
        return True
    if "failed to check service availability" in output and "context is invalid" in output:
        return True
    return False


def _env_int(key: str, default: int) -> int:
    try:
        return int(os.environ.get(key, str(default)).strip())
    except Exception:
        return default


def _env_float(key: str, default: float) -> float:
    try:
        return float(os.environ.get(key, str(default)).strip())
    except Exception:
        return default


# Retry policy (transient errors only)
TRANSIENT_RETRY = _env_int("KITCHEN_TRANSIENT_RETRY", 2)
TRANSIENT_DELAY = _env_float("KITCHEN_TRANSIENT_DELAY", 0.3)

TRANSIENT_RETRY_FRIDGE = _env_int("KITCHEN_TRANSIENT_RETRY_FRIDGE", max(3, TRANSIENT_RETRY))
TRANSIENT_DELAY_FRIDGE = _env_float("KITCHEN_TRANSIENT_DELAY_FRIDGE", 0.5)


def _run_module_once(mod: str) -> tuple[int, str]:
    """Run a legacy module and stream its stdout/stderr to our console while keeping a tail for error detection."""
    cmd = [sys.executable, "-m", mod]
    print(f"[runner] exec: {' '.join(cmd)}")

    # Merge stderr into stdout to avoid deadlocks and to capture traceback lines.
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    tail: List[str] = []
    assert proc.stdout is not None
    for line in proc.stdout:
        # Stream to console
        print(line, end="")
        tail.append(line)
        if len(tail) > 300:
            tail = tail[-300:]

    rc = proc.wait()
    return int(rc), "".join(tail)


def _run_module(mod: str, *, is_fridge: bool = False) -> int:
    """Run module with automatic retry on transient errors.

    Return codes:
      0  success
      2  not_found (propagated)
      101 transient failure after retries (so caller can skip and continue plan)
      other non-zero: real failure
    """
    retries = TRANSIENT_RETRY_FRIDGE if is_fridge else TRANSIENT_RETRY
    delay = TRANSIENT_DELAY_FRIDGE if is_fridge else TRANSIENT_DELAY

    for attempt in range(retries + 1):
        rc, out = _run_module_once(mod)

        if rc == 0 or rc == 2:
            return rc

        if _is_transient_error(out):
            if attempt < retries:
                print(f"[runner] transient error detected -> retry {attempt+1}/{retries} after {delay:.1f}s: {mod}")
                import time as _time
                _time.sleep(delay)
                continue
            print(f"[runner] transient error persists -> give up (rc=101): {mod}")
            return 101

        return rc

    return 101


def run_post_actions(mods: List[str]) -> bool:
    for m in mods:
        rc = _run_module(m, is_fridge=True)
        if rc != 0:
            print(f"[runner] post_action failed rc={rc}: {m}")
            return False
    return True

def run_object(obj_id: str) -> RunResult:

    if obj_id in FRIDGE_SEQ:
        open_mod, pick_mod, close_mod = FRIDGE_SEQ[obj_id]

        # 1) OPEN
        rc_open = _run_module(open_mod, is_fridge=True)
        if rc_open == 2:
            return RunResult(status="not_found", rc=rc_open, failed_module=open_mod)
        if rc_open != 0:
            return RunResult(status="fail", rc=rc_open, failed_module=open_mod)

        # 2) PICK (timeout)
        rc_pick = _run_module(pick_mod)

        if rc_pick == 0:
            # SUCCESS:
            # Close should happen AFTER handover/release -> defer close to post_actions
            return RunResult(status="ok", rc=0, post_actions=[close_mod])

        # NOT_FOUND or FAIL:
        # We will NOT enter release state, so close immediately (best-effort)
        rc_close = _run_module(close_mod, is_fridge=True)
        if rc_close != 0:
            return RunResult(status="fail", rc=rc_close, failed_module=close_mod)

        if rc_pick == 2:
            return RunResult(status="not_found", rc=rc_pick, failed_module=pick_mod)
        return RunResult(status="fail", rc=rc_pick, failed_module=pick_mod)

    mod = COMMANDS.get(obj_id, "")
    if not mod:
        return RunResult(status="fail", rc=99, failed_module="unknown_object")

    rc = _run_module(mod)
    if rc == 0:
        return RunResult(status="ok", rc=0)
    if rc == 2:
        return RunResult(status="not_found", rc=rc, failed_module=mod)
    return RunResult(status="fail", rc=rc, failed_module=mod)

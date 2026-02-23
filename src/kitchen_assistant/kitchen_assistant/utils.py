\
from __future__ import annotations
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def _pkg_share_dir(pkg_name: str) -> Path:
    try:
        return Path(get_package_share_directory(pkg_name))
    except Exception:
        # when running directly from source tree (not installed)
        return Path(__file__).resolve().parent

def resolve_model_path(filename: str, pkg_name: str = "kitchen_assistant") -> str:
    """
    Resolve model path from (1) absolute (2) CWD (3) env KITCHEN_MODEL_DIR (4) package share models/
    """
    p = Path(filename)
    if p.is_file():
        return str(p.resolve())
    if p.is_absolute():
        return str(p)

    # CWD
    p2 = Path.cwd() / filename
    if p2.is_file():
        return str(p2.resolve())

    # env dir
    env_dir = os.environ.get("KITCHEN_MODEL_DIR", "")
    if env_dir:
        p3 = Path(env_dir) / filename
        if p3.is_file():
            return str(p3.resolve())

    # package models/
    share = _pkg_share_dir(pkg_name)
    p4 = share / pkg_name / "models" / filename
    if p4.is_file():
        return str(p4.resolve())
    p5 = share / "models" / filename
    if p5.is_file():
        return str(p5.resolve())

    return filename  # let caller fail with clear error

def resolve_resource_path(filename: str, pkg_name: str = "kitchen_assistant") -> str:
    """
    Resolve resource file path (e.g. T_gripper2camera.npy) from:
      (1) env KITCHEN_RESOURCE_<NAME> (2) absolute/cwd (3) ~/ (4) package share
    """
    key = "KITCHEN_RESOURCE_" + filename.upper().replace(".", "_").replace("-", "_")
    env = os.environ.get(key, "")
    if env and Path(env).is_file():
        return str(Path(env).resolve())

    p = Path(filename)
    if p.is_file():
        return str(p.resolve())
    if p.is_absolute():
        return str(p)

    p2 = Path.cwd() / filename
    if p2.is_file():
        return str(p2.resolve())

    p3 = Path.home() / filename
    if p3.is_file():
        return str(p3.resolve())

    share = _pkg_share_dir(pkg_name)
    p4 = share / pkg_name / "models" / filename
    if p4.is_file():
        return str(p4.resolve())
    p5 = share / "models" / filename
    if p5.is_file():
        return str(p5.resolve())

    return filename

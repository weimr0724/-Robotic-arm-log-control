import os
import json

# Default calibration parameters used when calibration.json is missing or invalid.
DEFAULT_CAL = {
    "base": {"x_ratio": 0.50, "y_margin": 30},
    "visual_zero_deg": {"a1": 0, "a2": 0, "a3": 0},
    "link_lengths_px": {"l1": 160, "l2": 120, "l3": 90},
    "view_mode_default": "SIDE",
    "ui": {"show_world_axes": True, "show_robot_axes": True, "show_ee_trace": True},
}

# Calibration file path (relative to project root).
CAL_PATH = "calibration.json"


def clamp(x, lo=0, hi=180):
    """Clamp a numeric value into a safe range [lo, hi]."""
    try:
        x = float(x)
    except Exception:
        return lo
    return max(lo, min(hi, x))


def safe_mkdir(path: str):
    """Create a directory if it does not exist."""
    if not path:
        return
    os.makedirs(path, exist_ok=True)


def load_calibration(path: str = CAL_PATH, default: dict = DEFAULT_CAL) -> dict:
    """Load calibration config from JSON; fall back to default on failure."""
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return default
    return default


def save_calibration(cal: dict, path: str = CAL_PATH):
    """Save calibration config to JSON. Fail silently to avoid runtime interruption."""
    try:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(cal, f, ensure_ascii=False, indent=2)
    except Exception:
        pass
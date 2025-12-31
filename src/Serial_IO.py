from .utils import clamp


def send_T(ser, a1, a2, a3):
    """Send target joint angles to the controller using the 'T' command format."""
    if ser is None:
        return
    msg = f"T,{clamp(a1)},{clamp(a2)},{clamp(a3)}\n"
    try:
        ser.write(msg.encode("utf-8"))
    except Exception:
        pass


def parse_feedback_line(line: str):
    """
    Parse a feedback line from the controller.

    Expected format example:
      F,90,90,90

    Returns:
      (a1, a2, a3) as floats, or None if the line is invalid.
    """
    try:
        line = line.strip()
        if not line:
            return None

        if line.startswith("F"):
            parts = line.split(",")
            if len(parts) >= 4:
                a1 = float(parts[1])
                a2 = float(parts[2])
                a3 = float(parts[3])
                return (a1, a2, a3)

        return None
    except Exception:
        return None

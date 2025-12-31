import os, sys, time, json, csv
import numpy as np

from .utils import load_calibration, save_calibration, DEFAULT_CAL, CAL_PATH
from .Serial_IO import send_T, parse_feedback_line
from .ui_kinematics import CameraPanel, RunLogger, fk_points_side, draw_virtual_robot


Button = CameraPanel.Button



# ---------- Optional libs ----------
HAS_SERIAL = True
try:
    import serial
except Exception:
    HAS_SERIAL = False

HAS_CV2 = True
try:
    import cv2
except Exception:
    HAS_CV2 = False

import pygame

# =========================
# CONFIG 
# =========================
PORT = "COM4"
BAUD = 115200

ENABLE_SERIAL_DRIVE = True   # ✅ UI=False
ENABLE_CAMERA = True         # No Opencv = False

W, H = 1280, 820
FPS = 60

CAM_W, CAM_H = 360, 270
CAM_FPS_LIMIT = 25
CAM_INDEX_CANDIDATES = [0, 1, 2, 3]


# Motion / Marker 
A3_DEFAULT = 90
MANUAL_STEP_A3_PER_WHEEL = 2

# Motion
MOTION_DIFF_THRESH = 25      # More STABLE MEANS THE VALUE IS BIG（更稳）
MOTION_MIN_AREA = 900        
MOTION_DOWNSCALE = 0.5       # Avoid


A1_MIN, A1_MAX = 0, 180
A2_MIN, A2_MAX = 0, 180


DEADBAND_PX = 6              
EMA_ALPHA = 0.25             
RATE_LIMIT_DEG = 2           

# Marker
TRACK_COLOR = "green"        # "green" or "red"
TRACE_MAX = 600

# Log
LOG_DIR = "logs"

# 2D
DEFAULT_CAL = {
    "base": {"x_ratio": 0.50, "y_margin": 30},
    "visual_zero_deg": {"a1": 0, "a2": 0, "a3": 0},
    "link_lengths_px": {"l1": 160, "l2": 120, "l3": 90},
    "view_mode_default": "SIDE",
    "ui": {"show_world_axes": True, "show_robot_axes": True, "show_ee_trace": True}
}
CAL_PATH = "calibration.json"




# =========================
# UI Button
# =========================
class Button:
    def __init__(self, rect, text):
        self.rect = pygame.Rect(rect)
        self.text = text
    def draw(self, screen, font, active=True):
        color = (60, 60, 60) if active else (35, 35, 35)
        pygame.draw.rect(screen, color, self.rect, border_radius=10)
        pygame.draw.rect(screen, (120, 120, 120), self.rect, 2, border_radius=10)
        txt = font.render(self.text, True, (240, 240, 240))
        screen.blit(txt, txt.get_rect(center=self.rect.center))
    def hit(self, pos):
        return self.rect.collidepoint(pos)


# =========================
# Draw helpers (FK)
# =========================
def draw_panel_border(screen, x, y, w, h, title, font):
    pygame.draw.rect(screen, (35, 35, 35), (x, y, w, h), border_radius=14)
    pygame.draw.rect(screen, (120, 120, 120), (x, y, w, h), 2, border_radius=14)
    screen.blit(font.render(title, True, (220, 220, 220)), (x + 12, y + 10))

def draw_world_axes(screen, origin, axis_len=90, mono=None):
    ox, oy = origin
    pygame.draw.line(screen, (90, 90, 90), (ox, oy), (ox + axis_len, oy), 2)
    pygame.draw.line(screen, (90, 90, 90), (ox, oy), (ox, oy - axis_len), 2)
    pygame.draw.circle(screen, (90, 90, 90), (ox, oy), 3)
    if mono:
        screen.blit(mono.render("Xw", True, (120,120,120)), (ox + axis_len + 6, oy - 10))
        screen.blit(mono.render("Yw", True, (120,120,120)), (ox - 20, oy - axis_len - 22))

def draw_robot_axes(screen, origin, theta_rad, axis_len=80, mono=None):
    ox, oy = origin
    xr = (ox + axis_len*np.cos(theta_rad), oy - axis_len*np.sin(theta_rad))
    yr = (ox + axis_len*np.cos(theta_rad + np.pi/2), oy - axis_len*np.sin(theta_rad + np.pi/2))
    pygame.draw.line(screen, (0, 220, 255), (ox, oy), (int(xr[0]), int(xr[1])), 2)
    pygame.draw.line(screen, (0, 220, 255), (ox, oy), (int(yr[0]), int(yr[1])), 2)
    if mono:
        screen.blit(mono.render("Xr", True, (0,220,255)), (int(xr[0]) + 6, int(xr[1]) - 10))
        screen.blit(mono.render("Yr", True, (0,220,255)), (int(yr[0]) + 6, int(yr[1]) - 10))

def fk_points_side(angles_deg, link_lengths_px, base_xy):
    a1, a2, a3 = angles_deg
    L1, L2, L3 = link_lengths_px
    bx, by = base_xy

    r1 = np.deg2rad(a1)
    r2 = np.deg2rad(a2)
    r3 = np.deg2rad(a3)

    t1 = r1
    t2 = r1 + r2
    t3 = r1 + r2 + r3

    p0 = (bx, by)
    p1 = (bx + L1*np.cos(t1), by - L1*np.sin(t1))
    p2 = (p1[0] + L2*np.cos(t2), p1[1] - L2*np.sin(t2))
    p3 = (p2[0] + L3*np.cos(t3), p2[1] - L3*np.sin(t3))
    return p0, p1, p2, p3, t1

def draw_virtual_robot(screen, x, y, w, h, angles_actual, cal, view_mode, ee_trace, font, mono):
    draw_panel_border(screen, x, y, w, h, f"Virtual Robot (ACTUAL) [{view_mode}]", font)

    bx = int(x + w * float(cal["base"]["x_ratio"]))
    by = int(y + h - int(cal["base"]["y_margin"]))
    base = (bx, by)

    vz = cal["visual_zero_deg"]
    a1, a2, a3 = angles_actual
    a1v = a1 + float(vz["a1"])
    a2v = a2 + float(vz["a2"])
    a3v = a3 + float(vz["a3"])

    ll = cal["link_lengths_px"]
    link_lengths = (int(ll["l1"]), int(ll["l2"]), int(ll["l3"]))

    ui = cal.get("ui", {})
    if ui.get("show_world_axes", True):
        draw_world_axes(screen, base, axis_len=90, mono=mono)

    p0, p1, p2, p3, theta = fk_points_side((a1v, a2v, a3v), link_lengths, base)

    if ui.get("show_robot_axes", True):
        draw_robot_axes(screen, base, theta, axis_len=80, mono=mono)

    # EE trace
    if ui.get("show_ee_trace", True) and len(ee_trace) >= 2:
        pts = [(int(px), int(py)) for (px, py) in ee_trace]
        clipped = [(px_, py_) for (px_, py_) in pts if x+6 <= px_ <= x+w-6 and y+6 <= py_ <= y+h-6]
        if len(clipped) >= 2:
            pygame.draw.lines(screen, (80,255,120), False, clipped, 2)

    def ip(p): return (int(p[0]), int(p[1]))
    pygame.draw.line(screen, (0,220,255), ip(p0), ip(p1), 6)
    pygame.draw.line(screen, (0,220,255), ip(p1), ip(p2), 6)
    pygame.draw.line(screen, (0,220,255), ip(p2), ip(p3), 6)

    for pt in [p0, p1, p2, p3]:
        pygame.draw.circle(screen, (240,240,240), ip(pt), 7)
        pygame.draw.circle(screen, (20,20,20), ip(pt), 7, 2)
    pygame.draw.circle(screen, (80,255,120), ip(p3), 10, 2)

    screen.blit(mono.render(f"ACTUAL(raw)={tuple(angles_actual)}", True, (200,200,200)), (x+12, y+h-56))
    screen.blit(mono.render(f"VISUAL(map)=({int(a1v)},{int(a2v)},{int(a3v)})", True, (160,160,160)), (x+12, y+h-32))
    return (float(p3[0]), float(p3[1]))


# =========================
# MAIN
# =========================
def main():
    # --- init serial (safe) ---
    ser = None
    serial_err = None
    if ENABLE_SERIAL_DRIVE:
        if not HAS_SERIAL:
            serial_err = "pyserial not installed"
        else:
            try:
                ser = serial.Serial(PORT, BAUD, timeout=0.0)
                time.sleep(1.0)
            except Exception as e:
                serial_err = str(e)
                ser = None

    cal = load_calibration(CAL_PATH)
    view_mode = cal.get("view_mode_default", "SIDE")

    pygame.init()
    pygame.display.set_caption("Industrial Digital Twin v6 (Motion/Marker -> A1A2, A3 wheel)")
    screen = pygame.display.set_mode((W, H))
    clock = pygame.time.Clock()

    font = pygame.font.SysFont(None, 28)
    big  = pygame.font.SysFont(None, 40)
    mono = pygame.font.SysFont("consolas", 22)

    # buttons
    btn_home  = Button((40, 740, 160, 55), "HOME")
    btn_reset = Button((220, 740, 160, 55), "RESET TRACE")
    btn_mode  = Button((400, 740, 160, 55), "MODE: MOTION")
    btn_send  = Button((580, 740, 160, 55), "SEND: ON" if ENABLE_SERIAL_DRIVE else "SEND: OFF")
    btn_quit  = Button((760, 740, 160, 55), "QUIT")

    # states
    target = [90, 90, A3_DEFAULT]
    actual = [90, 90, A3_DEFAULT]
    fb_status = "NO_FB"
    rx_buf = ""
    last_fb_ts = 0.0

    # camera
    cam = CameraPanel(CAM_W, CAM_H, fps_limit=CAM_FPS_LIMIT, candidates=CAM_INDEX_CANDIDATES)

    # traces
    ee_trace = []
    cam_trace = []
    motion_trace = []
    last_ee = None

    # vision control
    vision_on = True         # Fault is open state（close =  V）
    source = "VISION" if vision_on else "IDLE"
    mode = "MOTION"          # MOTION or MARKER
    strategy = "B0_RAW"

    # motion smooth state
    sm_cx, sm_cy = None, None
    last_center = None
    last_sent = time.time()

    # logger
    logger = RunLogger()
    print("[LOG] CSV path =", logger.path)


    def read_feedback():
        nonlocal rx_buf, actual, fb_status, last_fb_ts
        if ser is None:
            return
        try:
            data = ser.read(256)
            if data:
                rx_buf += data.decode("utf-8", errors="ignore")
                while "\n" in rx_buf:
                    line, rx_buf = rx_buf.split("\n", 1)
                    fb = parse_feedback_line(line)
                    if fb:
                        a1, a2, a3, st = fb
                        actual = [a1, a2, a3]
                        fb_status = st
                        last_fb_ts = time.time()
        except Exception:
            pass

    def link_text():
        if not ENABLE_SERIAL_DRIVE:
            return "LINK: SERIAL OFF", (160,160,160)
        if ser is None:
            return "LINK: SERIAL FAIL", (255,80,80)
        if last_fb_ts == 0:
            return "LINK: WAITING...", (255,180,120)
        if (time.time() - last_fb_ts) > 1.2:
            return "LINK: FEEDBACK LOST", (255,80,80)
        return "LINK: CONNECTED", (120,220,120)

    def push_ee_trace(pt):
        nonlocal last_ee
        if pt is None:
            return
        if last_ee is None:
            ee_trace.append(pt); last_ee = pt
        else:
            dx = pt[0] - last_ee[0]
            dy = pt[1] - last_ee[1]
            if dx*dx + dy*dy >= 4:
                ee_trace.append(pt)
                last_ee = pt
        if len(ee_trace) > 900:
            ee_trace.pop(0)

    def compute_angles_from_center(cx, cy):
        # cx in [0, CAM_W], cy in [0, CAM_H]
        # map to A1 (left-right), A2 (up-down)
        a1 = A1_MIN + (A1_MAX - A1_MIN) * (cx / max(1, CAM_W))
        a2 = A2_MAX - (A2_MAX - A2_MIN) * (cy / max(1, CAM_H))  # up => larger
        return clamp(a1, A1_MIN, A1_MAX), clamp(a2, A2_MIN, A2_MAX)

    def smooth_and_limit(new_a1, new_a2):
        # rate limit relative to current target
        cur_a1, cur_a2 = target[0], target[1]
        da1 = max(-RATE_LIMIT_DEG, min(RATE_LIMIT_DEG, new_a1 - cur_a1))
        da2 = max(-RATE_LIMIT_DEG, min(RATE_LIMIT_DEG, new_a2 - cur_a2))
        return clamp(cur_a1 + da1), clamp(cur_a2 + da2)

    running = True
    try:
        while running:
            # read hardware feedback
            read_feedback()

            # camera update
            cam.update()

            # ----- Vision Control -----
            dx = ""
            dy = ""
            if vision_on and ENABLE_CAMERA and HAS_CV2 and cam.ok:
                center = None
                if mode == "MOTION":
                    center = cam.motion_center
                else:
                    center = cam.marker_center

                if center is not None:
                    cx, cy = center

                    # deadband vs last_center
                    if last_center is None:
                        last_center = (cx, cy)
                    if abs(cx - last_center[0]) < DEADBAND_PX and abs(cy - last_center[1]) < DEADBAND_PX:
                        pass
                    else:
                        last_center = (cx, cy)

                        # EMA smooth on pixel center
                        if sm_cx is None:
                            sm_cx, sm_cy = float(cx), float(cy)
                        else:
                            sm_cx = (1-EMA_ALPHA)*sm_cx + EMA_ALPHA*cx
                            sm_cy = (1-EMA_ALPHA)*sm_cy + EMA_ALPHA*cy

                        # to angles
                        a1_new, a2_new = compute_angles_from_center(sm_cx, sm_cy)
                        a1_new, a2_new = smooth_and_limit(a1_new, a2_new)
                        target[0], target[1] = a1_new, a2_new

                        dx = int(sm_cx)
                        dy = int(sm_cy)

                        # send to robot
                        if ENABLE_SERIAL_DRIVE and ser is not None:
                            send_T(ser, target[0], target[1], target[2])

                    # trace for drawing (rot90)
                    rx, ry = rot90_coord(int(cx), int(cy), CAM_W, CAM_H)
                    cam_trace.append((rx, ry))
                    if len(cam_trace) > TRACE_MAX:
                        cam_trace.pop(0)

            source = "VISION" if vision_on else "IDLE"

            # ----- log -----
            logger.log(strategy, source, mode, tuple(target), tuple(actual), dx, dy)

            # ----- draw -----
            screen.fill((22,22,22))
            screen.blit(big.render("Industrial Digital Twin v6", True, (240,240,240)), (40, 20))

            lt, lc = link_text()
            screen.blit(font.render(lt, True, lc), (40, 70))
            screen.blit(font.render(f"FB_STATUS: {fb_status}", True, (200,200,200)), (260, 70))

            if serial_err:
                screen.blit(font.render(f"Serial error: {serial_err}", True, (255,120,120)), (40, 105))

            screen.blit(font.render(f"BASELINE: {strategy}   SOURCE: {source}   VISION({mode}): {'ON' if vision_on else 'OFF'}", True, (180,180,180)), (40, 105))
            screen.blit(font.render(f"CSV: {logger.path}", True, (160,160,160)), (40, 135))

            err = (target[0]-actual[0], target[1]-actual[1], target[2]-actual[2])
            screen.blit(font.render(f"TARGET: {tuple(target)}", True, (180,180,180)), (40, 155))
            screen.blit(font.render(f"ACTUAL: {tuple(actual)}", True, (0,220,255)), (320, 155))
            screen.blit(font.render(f"ERROR : {err}", True, (255,180,120)), (600, 155))

            # left robot
            ee = draw_virtual_robot(screen, 40, 190, 820, 500, tuple(actual), cal, view_mode, ee_trace, font, mono)
            push_ee_trace(ee)

            # right camera
            draw_panel_border(screen, 900, 190, 340, 320, "Camera Monitoring (Motion/Marker)", font)
            cam_x = 900 + (340 - CAM_W)//2
            cam_y = 190 + 45

            if ENABLE_CAMERA and HAS_CV2 and cam.ok and cam.last_frame is not None:
                screen.blit(cam.last_frame, (cam_x, cam_y))
                pygame.draw.rect(screen, (120,120,120), (cam_x, cam_y, CAM_W, CAM_H), 1)

                # trace
                if len(cam_trace) >= 2:
                    pts = [(cam_x + x, cam_y + y) for (x, y) in cam_trace]
                    col = (255,160,80) if mode == "MOTION" else (80,255,120)
                    pygame.draw.lines(screen, col, False, pts, 2)

                # status text
                screen.blit(mono.render("Orange=motion   Green=marker", True, (180,180,180)), (910, 470))
                screen.blit(mono.render("Keys: V on/off, M motion/marker, C clear", True, (160,160,160)), (910, 495))
            else:
                msg = "CAM OFF (pip install opencv-python)" if not HAS_CV2 else "CAM NOT FOUND"
                screen.blit(mono.render(msg, True, (255,80,80)), (910, 330))
                screen.blit(mono.render("Tip: plug webcam, try index 0/1/2", True, (160,160,160)), (910, 355))

            # buttons row
            btn_mode.text = f"MODE: {mode}"
            btn_send.text = "SEND: ON" if ENABLE_SERIAL_DRIVE else "SEND: OFF"
            for b in [btn_home, btn_reset, btn_mode, btn_send, btn_quit]:
                b.draw(screen, font, active=True)

            # footer hints
            hint = "Buttons: HOME/RESET/MODE/SEND/QUIT | Keys: V=Vision ON/OFF, M=Mode, C=Clear trace, Wheel=A3, S=Save calib, ESC=Quit"
            screen.blit(font.render(hint, True, (160,160,160)), (40, 805))

            pygame.display.flip()

            # ----- events -----
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

                    elif event.key == pygame.K_v:
                        vision_on = not vision_on

                    elif event.key == pygame.K_m:
                        mode = "MARKER" if mode == "MOTION" else "MOTION"

                    elif event.key == pygame.K_c:
                        cam_trace = []

                    elif event.key == pygame.K_s:
                        ok = save_calibration(cal, CAL_PATH)
                        fb_status = "CAL_SAVED" if ok else "CAL_SAVE_FAIL"

                if event.type == pygame.MOUSEWHEEL:
                    target[2] = clamp(target[2] + event.y * MANUAL_STEP_A3_PER_WHEEL)
                    if ENABLE_SERIAL_DRIVE and ser is not None:
                        send_T(ser, target[0], target[1], target[2])

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    pos = event.pos
                    if btn_home.hit(pos):
                        target = [90, 90, target[2]]
                        if ENABLE_SERIAL_DRIVE and ser is not None:
                            send_T(ser, target[0], target[1], target[2])
                        fb_status = "HOME_SENT"

                    elif btn_reset.hit(pos):
                        cam_trace = []
                        ee_trace = []
                        fb_status = "TRACE_RESET"

                    elif btn_mode.hit(pos):
                        mode = "MARKER" if mode == "MOTION" else "MOTION"

                    elif btn_send.hit(pos):
                        
                        nonlocal_enable = None  # just to keep structure stable

                    elif btn_quit.hit(pos):
                        running = False

            clock.tick(FPS)

    except Exception as e:
        
        print("FATAL ERROR:", repr(e))
        import traceback
        traceback.print_exc()

    # cleanup
    try:
        cam.close()
    except Exception:
        pass
    try:
        logger.close()
    except Exception:
        pass
    try:
        if ser is not None:
            ser.close()
    except Exception:
        pass
    pygame.quit()


if __name__ == "__main__":
    main()


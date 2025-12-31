import os, time, csv
import pygame

from .utils import safe_mkdir


# ===== UI / Camera settings =====
# Set to False to run without any camera dependency.
ENABLE_CAMERA = False

# OpenCV availability flag (used by CameraPanel). If you do not need camera, keep False.
HAS_CV2 = False

# Directory used to store runtime CSV logs.
LOG_DIR = "logs"


# =========================
# Camera Panel + Motion/Marker
# =========================
class CameraPanel:
    def __init__(self, w, h, fps_limit=25, candidates=None):
        self.w, self.h = int(w), int(h)
        self.fps_limit = max(1, int(fps_limit))
        self.candidates = candidates or [0]
        self.cap = None
        self.ok = False
        self.last_frame = None   # pygame surface (rotated)
        self.raw_bgr = None      # bgr for cv
        self.last_grab = 0.0

        self.prev_gray = None
        self.motion_center = None   # (cx, cy) original coord
        self.marker_center = None   # (cx, cy) original coord

        if ENABLE_CAMERA and HAS_CV2:
            self._open_first_available()

    def _open_first_available(self):
        for idx in self.candidates:
            cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
            if cap is not None and cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
                self.cap = cap
                self.ok = True
                return
        self.ok = False
        self.cap = None

    def close(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        self.cap = None
        self.ok = False

    def update(self):
        if (not ENABLE_CAMERA) or (not HAS_CV2) or (not self.ok) or (self.cap is None):
            return

        now = time.time()
        if now - self.last_grab < (1 / self.fps_limit):
            return
        self.last_grab = now

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.ok = False
            return

        frame = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_AREA)
        self.raw_bgr = frame.copy()

    def rot90_coord(cx, cy, w, h):
        # np.rot90 CCW: x' = cy, y' = (w - 1 - cx)
        return (cy, (w - 1 - cx))
    
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
# CSV Logger
# =========================
class RunLogger:
    def __init__(self):
        safe_mkdir(LOG_DIR)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(LOG_DIR, f"run_{ts}.csv")
        self.f = open(self.path, "w", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "t","strategy","source","mode",
            "target_a1","target_a2","target_a3",
            "actual_a1","actual_a2","actual_a3",
            "err_a1","err_a2","err_a3",
            "dx","dy"
        ])
        self.flush_every = 30   # 每30行强制写盘一次
        self.n = 0
        self.f.flush()

    def log(self, strategy, source, mode, target, actual, dx, dy):
        t = int(time.time())
        err = (target[0]-actual[0], target[1]-actual[1], target[2]-actual[2])
        self.w.writerow([
            t, strategy, source, mode,
            target[0], target[1], target[2],
            actual[0], actual[1], actual[2],
            err[0], err[1], err[2],
            dx, dy
        ])
        self.n += 1
        if self.n % self.flush_every == 0:
            try:
                self.f.flush()
            except Exception:
                pass

    def close(self):
        try:
            self.f.flush()
        except Exception:
            pass
        try:
            self.f.close()
        except Exception:
            pass

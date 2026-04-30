"""MPU6050 live visualizer.

Reads CSV stream from the Arduino over serial and renders an orientation cube
plus a numeric readout. Run:

    python visualizer.py /dev/ttyACM0 115200
"""
from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass, field

import pygame
import serial
from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST, GL_LINES,
    GL_LINE_LOOP, GL_MODELVIEW, GL_PROJECTION, GL_QUADS, glBegin, glClear,
    glClearColor, glColor3f, glDisable, glEnable, glEnd, glLoadIdentity,
    glMatrixMode, glOrtho, glPopMatrix, glPushMatrix, glRotatef, glVertex2f,
    glVertex3f,
)
from OpenGL.GLU import gluLookAt, gluPerspective


@dataclass
class ImuState:
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    temp_c: float = 0.0
    ch: tuple = (0, 0, 0, 0, 0, 0)
    t_ms: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_from_csv(self, line: str) -> bool:
        parts = line.strip().split(",")
        if len(parts) != 17:
            return False
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            return False
        with self.lock:
            (self.ax, self.ay, self.az,
             self.gx, self.gy, self.gz,
             self.roll, self.pitch, self.yaw,
             self.temp_c,
             c1, c2, c3, c4, c5, c6, t) = vals
            self.ch = (int(c1), int(c2), int(c3), int(c4), int(c5), int(c6))
            self.t_ms = int(t)
        return True

    def snapshot(self):
        with self.lock:
            return (self.ax, self.ay, self.az,
                    self.gx, self.gy, self.gz,
                    self.roll, self.pitch, self.yaw,
                    self.temp_c, self.ch, self.t_ms)


def serial_reader(port: str, baud: int, state: ImuState, stop: threading.Event):
    while not stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                while not stop.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore")
                    if line.startswith("#") or line.startswith("ax,"):
                        continue
                    state.update_from_csv(line)
        except serial.SerialException as exc:
            print(f"[serial] {exc} — retrying in 2s", file=sys.stderr)
            time.sleep(2)


def draw_cube():
    # Face vertices for a 2x1x0.4 "drone body" so orientation is unambiguous.
    sx, sy, sz = 1.0, 0.6, 0.15
    faces = [
        ((1, 0, 0), [(+sx, -sy, -sz), (+sx, +sy, -sz), (+sx, +sy, +sz), (+sx, -sy, +sz)]),
        ((0.6, 0, 0), [(-sx, -sy, -sz), (-sx, -sy, +sz), (-sx, +sy, +sz), (-sx, +sy, -sz)]),
        ((0, 1, 0), [(-sx, +sy, -sz), (-sx, +sy, +sz), (+sx, +sy, +sz), (+sx, +sy, -sz)]),
        ((0, 0.6, 0), [(-sx, -sy, -sz), (+sx, -sy, -sz), (+sx, -sy, +sz), (-sx, -sy, +sz)]),
        ((0, 0, 1), [(-sx, -sy, +sz), (+sx, -sy, +sz), (+sx, +sy, +sz), (-sx, +sy, +sz)]),
        ((0.7, 0.7, 0.7), [(-sx, -sy, -sz), (-sx, +sy, -sz), (+sx, +sy, -sz), (+sx, -sy, -sz)]),
    ]
    glBegin(GL_QUADS)
    for color, verts in faces:
        glColor3f(*color)
        for v in verts:
            glVertex3f(*v)
    glEnd()


def draw_axes():
    glBegin(GL_LINES)
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(2, 0, 0)
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 2, 0)
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 2)
    glEnd()


def render_text(font, surface, lines, x=10, y=10, color=(230, 230, 230)):
    for i, line in enumerate(lines):
        img = font.render(line, True, color)
        surface.blit(img, (x, y + i * 18))


def _quad(x0, y0, x1, y1):
    glBegin(GL_QUADS)
    glVertex2f(x0, y0); glVertex2f(x1, y0)
    glVertex2f(x1, y1); glVertex2f(x0, y1)
    glEnd()


def _rect_outline(x0, y0, x1, y1):
    glBegin(GL_LINE_LOOP)
    glVertex2f(x0, y0); glVertex2f(x1, y0)
    glVertex2f(x1, y1); glVertex2f(x0, y1)
    glEnd()


def _stick_box(cx, cy, size, dot_x, dot_y, throttle=False):
    """Draw a stick box centered at (cx, cy). dot_x/dot_y are -1..+1; for
    throttle, dot_y is 0..1 (no center spring)."""
    half = size / 2
    glColor3f(0.15, 0.15, 0.18); _quad(cx - half, cy - half, cx + half, cy + half)
    glColor3f(0.4, 0.4, 0.45);   _rect_outline(cx - half, cy - half, cx + half, cy + half)
    # Center crosshair (or throttle midline for non-centering stick).
    glColor3f(0.3, 0.3, 0.35)
    glBegin(GL_LINES)
    if throttle:
        glVertex2f(cx - half, cy); glVertex2f(cx + half, cy)
    else:
        glVertex2f(cx - half, cy); glVertex2f(cx + half, cy)
        glVertex2f(cx, cy - half); glVertex2f(cx, cy + half)
    glEnd()
    # Dot.
    px = cx + dot_x * half * 0.92
    py = cy - dot_y * half * 0.92  # screen-y inverted: stick up = lower y
    glColor3f(0.95, 0.85, 0.25)
    _quad(px - 6, py - 6, px + 6, py + 6)


def draw_sticks_hud(width, height, ch):
    """Render two stick boxes in screen-space ortho. AETR convention:
    ch[0]=roll, ch[1]=pitch, ch[2]=throttle, ch[3]=yaw."""
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, width, height, 0, -1, 1)
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)

    size = 160
    margin = 30
    left_cx  = margin + size / 2
    right_cx = width - margin - size / 2
    cy       = height - margin - size / 2

    # Left stick: yaw (x, ch4) + throttle (y, ch3, 0..1).
    yaw_x      = max(-1.0, min(1.0, (ch[3] - 1500) / 500.0)) if ch[3] else 0.0
    throt_norm = max( 0.0, min(1.0, (ch[2] - 1000) / 1000.0)) if ch[2] else 0.0
    _stick_box(left_cx, cy, size, yaw_x, throt_norm * 2 - 1, throttle=True)

    # Right stick: roll (x, ch1) + pitch (y, ch2).
    roll_x  = max(-1.0, min(1.0, (ch[0] - 1500) / 500.0)) if ch[0] else 0.0
    pitch_y = max(-1.0, min(1.0, (ch[1] - 1500) / 500.0)) if ch[1] else 0.0
    _stick_box(right_cx, cy, size, roll_x, pitch_y, throttle=False)

    glEnable(GL_DEPTH_TEST)
    glPopMatrix()
    glMatrixMode(GL_PROJECTION); glPopMatrix()
    glMatrixMode(GL_MODELVIEW)


# Each view = (label, eye, up). World frame: X right, Y forward, Z up.
# Drone is centered at the origin and stays level when IMU reads zero — only
# the camera moves. center is always (0,0,0).
VIEWS = {
    pygame.K_1: ("perspective", (3.5, -4.5, 3.0), (0, 0, 1)),
    pygame.K_2: ("top-down",    (0.0,  0.0, 6.0), (0, 1, 0)),
    pygame.K_3: ("front",       (0.0, -6.0, 0.0), (0, 0, 1)),
}


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    pygame.init()
    width, height = 1000, 640
    pygame.display.set_mode((width, height), pygame.OPENGL | pygame.DOUBLEBUF)
    pygame.display.set_caption("MPU6050 visualizer")

    glEnable(GL_DEPTH_TEST)
    glClearColor(0.08, 0.08, 0.10, 1.0)
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, width / height, 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)

    state = ImuState()
    stop = threading.Event()
    reader = threading.Thread(
        target=serial_reader, args=(port, baud, state, stop), daemon=True
    )
    reader.start()

    view_name, view_eye, view_up = VIEWS[pygame.K_1]

    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key in VIEWS:
                    view_name, view_eye, view_up = VIEWS[event.key]

        ax, ay, az, gx, gy, gz, roll, pitch, yaw, temp_c, ch, t_ms = state.snapshot()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluLookAt(view_eye[0], view_eye[1], view_eye[2],
                  0.0, 0.0, 0.0,
                  view_up[0], view_up[1], view_up[2])
        draw_axes()
        # IMU orientation, applied yaw (Z) → pitch (Y) → roll (X).
        glRotatef(yaw, 0, 0, 1)
        glRotatef(pitch, 0, 1, 0)
        glRotatef(roll, 1, 0, 0)
        draw_cube()
        draw_sticks_hud(width, height, ch)
        pygame.display.flip()

        # AETR convention: CH1=Aileron, CH2=Elevator, CH3=Throttle, CH4=Rudder.
        pygame.display.set_caption(
            f"[{view_name}]  "
            f"r={roll:+5.0f} p={pitch:+5.0f} y={yaw:+5.0f}  T={temp_c:.0f}C  "
            f"A={ch[0]:4d} E={ch[1]:4d} T={ch[2]:4d} R={ch[3]:4d} "
            f"X1={ch[4]:4d} X2={ch[5]:4d}  [1/2/3 view  Esc quit]"
        )
        clock.tick(60)

    stop.set()
    pygame.quit()


if __name__ == "__main__":
    main()

"""MPU6050 live visualizer.

Reads CSV stream from the Arduino over serial and renders an orientation cube
plus a numeric readout. Run:

    python visualizer.py /dev/ttyACM0 115200
"""
from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass, field

import pygame
import serial
from OpenGL.GL import (
    GL_BLEND, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_DEPTH_TEST,
    GL_LINEAR, GL_LINES, GL_LINE_LOOP, GL_MODELVIEW, GL_ONE_MINUS_SRC_ALPHA,
    GL_PROJECTION, GL_QUADS, GL_RGBA, GL_SRC_ALPHA, GL_TEXTURE_2D,
    GL_TEXTURE_MAG_FILTER, GL_TEXTURE_MIN_FILTER, GL_UNSIGNED_BYTE, glBegin,
    glBindTexture, glBlendFunc, glClear, glClearColor, glColor3f,
    glDisable, glEnable, glEnd, glGenTextures, glLoadIdentity, glMatrixMode,
    glOrtho, glPopMatrix, glPushMatrix, glRotatef, glTexCoord2f, glTexImage2D,
    glTexParameteri, glVertex2f, glVertex3f,
)
from OpenGL.GLU import gluLookAt, gluPerspective


DISPLAY_HYST_US = 10  # ignore RX/motor jitter smaller than this for display only


def _hyst(prev: int, new: int, tol: int) -> int:
    """Hysteresis filter: keep `prev` unless `new` differs by >= tol.
    Pure display-side smoothing — the firmware's flight control still uses
    full-resolution values."""
    return new if abs(new - prev) >= tol else prev


@dataclass
class ImuState:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    temp_c: float = 0.0
    ch: tuple = (0, 0, 0, 0, 0, 0)
    armed: bool = False
    motors: tuple = (1000, 1000, 1000, 1000)
    t_ms: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_from_csv(self, line: str) -> bool:
        parts = line.strip().split(",")
        if len(parts) != 16:
            return False
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            return False
        # Defensive: if firmware ever streams a NaN (shouldn't happen, but
        # we have observed it with an MPU glitch), don't let it corrupt the
        # cube rotation or the live readout.
        for i in range(len(vals)):
            if not math.isfinite(vals[i]):
                vals[i] = 0.0
        with self.lock:
            (self.roll, self.pitch, self.yaw, self.temp_c,
             c1, c2, c3, c4, c5, c6,
             armed, m1, m2, m3, m4, t) = vals
            new_ch = (int(c1), int(c2), int(c3), int(c4), int(c5), int(c6))
            new_m  = (int(m1), int(m2), int(m3), int(m4))
            self.ch     = tuple(_hyst(p, n, DISPLAY_HYST_US)
                                for p, n in zip(self.ch, new_ch))
            self.motors = tuple(_hyst(p, n, DISPLAY_HYST_US)
                                for p, n in zip(self.motors, new_m))
            self.armed  = (int(armed) != 0)
            self.t_ms   = int(t)
        return True

    def snapshot(self):
        with self.lock:
            return (self.roll, self.pitch, self.yaw,
                    self.temp_c, self.ch,
                    self.armed, self.motors, self.t_ms)


def serial_reader(port: str, baud: int, state: ImuState, stop: threading.Event):
    while not stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                while not stop.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="ignore")
                    if line.startswith("#") or line.startswith("roll,"):
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


def _make_text_texture(font, text, color=(235, 235, 235)):
    """Render `text` to a pygame surface and upload as an OpenGL RGBA texture.
    Returns (texture_id, width_px, height_px). Background is fully transparent
    (font.render with antialias produces per-pixel alpha)."""
    surface = font.render(text, True, color)
    w, h = surface.get_size()
    data = pygame.image.tostring(surface, "RGBA", True)
    tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
    glBindTexture(GL_TEXTURE_2D, 0)
    return tex, w, h


def _draw_textured_quad(tex_id, x, y, w, h):
    """Blit the text texture at (x, y), size (w, h), in screen-space ortho."""
    glEnable(GL_TEXTURE_2D)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glBindTexture(GL_TEXTURE_2D, tex_id)
    glColor3f(1, 1, 1)
    glBegin(GL_QUADS)
    # ortho is set with y-flip via glOrtho(0, w, h, 0); texture coords stay normal.
    glTexCoord2f(0, 1); glVertex2f(x,     y)
    glTexCoord2f(1, 1); glVertex2f(x + w, y)
    glTexCoord2f(1, 0); glVertex2f(x + w, y + h)
    glTexCoord2f(0, 0); glVertex2f(x,     y + h)
    glEnd()
    glDisable(GL_TEXTURE_2D)
    glDisable(GL_BLEND)
    glBindTexture(GL_TEXTURE_2D, 0)


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


def draw_motors_hud(width, height, motors, armed, label_textures, value_cache):
    """Render the four motors as labeled squares laid out like the X-quad
    seen from above:

           [M4]   [M1]      front (drone nose)
           [M3]   [M2]      rear

    Each square outline is fixed size; an inner filled square scales in BOTH
    dimensions with the motor command (1000..2000 us -> 0..1). Color goes
    green -> yellow -> red with output level. Disarmed: gray inner square.
    Text labels (M1..M4) are drawn above each square as OpenGL textures
    pre-rendered from a pygame font in init_motor_labels()."""
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, width, height, 0, -1, 1)
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)

    sq, gap   = 80, 36                # square size and spacing between motors
    label_h   = 22                    # text room above each square for "Mn"
    value_h   = 18                    # text room below each square for "1234"
    grid_w    = sq * 2 + gap
    grid_h    = (sq + label_h + value_h) * 2 + gap
    cx        = width / 2
    cy_top    = 14                    # top of the whole grid in screen px

    # Cell layout mirrors the physical X-quad seen from above:
    #   row 0 (top of grid)    = front of drone -> M4 (FL),  M1 (FR)
    #   row 1 (bottom of grid) = rear of drone  -> M3 (RL),  M2 (RR)
    col_left  = cx - sq - gap / 2
    col_right = cx + gap / 2
    row_top    = cy_top + label_h
    row_bottom = cy_top + label_h + sq + value_h + gap + label_h

    cells = [
        ("M4", col_left,  row_top,    motors[3]),
        ("M1", col_right, row_top,    motors[0]),
        ("M3", col_left,  row_bottom, motors[2]),
        ("M2", col_right, row_bottom, motors[1]),
    ]

    for name, x, y, m_us in cells:
        # Label above the square.
        tex_id, tw, th = label_textures[name]
        _draw_textured_quad(tex_id, x + (sq - tw) / 2, y - label_h, tw, th)

        # Outer outline.
        glColor3f(0.45, 0.45, 0.50)
        _rect_outline(x, y, x + sq, y + sq)

        # Inner filled square sized by motor output.
        norm = max(0.0, min(1.0, (m_us - 1000) / 1000.0))
        if not armed:
            r, g, b = 0.32, 0.32, 0.34
        elif norm < 0.5:
            r, g, b = 0.20 + norm * 1.0, 0.85, 0.20
        else:
            r, g, b = 0.95, 0.85 - (norm - 0.5) * 1.4, 0.10
        inner = sq * (0.20 + 0.78 * norm)
        ix    = x + (sq - inner) / 2
        iy    = y + (sq - inner) / 2
        glColor3f(r, g, b)
        _quad(ix, iy, ix + inner, iy + inner)

        # Live us value below the square.
        text     = str(m_us)
        _, vw, _ = value_cache.get(text)
        value_cache.draw(text, x + (sq - vw) / 2, y + sq + 2)

    glEnable(GL_DEPTH_TEST)
    glPopMatrix()
    glMatrixMode(GL_PROJECTION); glPopMatrix()
    glMatrixMode(GL_MODELVIEW)


def init_motor_labels(font):
    """Pre-render M1..M4 text labels as GL textures. Call once after the
    GL context exists (i.e., after pygame.display.set_mode)."""
    return {name: _make_text_texture(font, name) for name in ("M1", "M2", "M3", "M4")}


class TextCache:
    """Lazy GL texture cache for short strings (e.g. live motor pulse widths).
    Each unique string is rendered + uploaded once and reused across frames.
    Memory budget is tiny: ~1000 unique us values × ~3 KB each = ~3 MB."""

    def __init__(self, font, color=(220, 220, 220)):
        self.font  = font
        self.color = color
        self._cache = {}

    def get(self, text):
        entry = self._cache.get(text)
        if entry is None:
            entry = _make_text_texture(self.font, text, self.color)
            self._cache[text] = entry
        return entry

    def draw(self, text, x, y):
        tex_id, w, h = self.get(text)
        _draw_textured_quad(tex_id, x, y, w, h)
        return w, h


def draw_sticks_hud(width, height, ch, stick_labels, value_cache):
    """Render two stick boxes in screen-space ortho. AETR convention:
    ch[0]=roll, ch[1]=pitch, ch[2]=throttle, ch[3]=yaw.

    Each box gets two axis labels (above + left) and a two-line live readout
    BELOW showing the actual RX pulse-width µs the firmware is seeing —
    useful for debugging arming thresholds and stick-range issues."""
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
    glOrtho(0, width, height, 0, -1, 1)
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity()
    glDisable(GL_DEPTH_TEST)

    size = 160
    margin = 30
    left_cx  = margin + size / 2
    right_cx = width - margin - size / 2
    cy       = height - margin - size / 2

    # Headroom: trim noise near mechanical stick extremes by mapping a slightly
    # narrower band onto full HUD range. Outside the band saturates.
    #   centered sticks (roll/pitch/yaw): 1500 ± 400  -> ±1.0
    #   throttle:                         1100..1700  ->  0..1
    CENTER_HALF = 400.0
    THROT_LO, THROT_HI = 1100.0, 1700.0

    def _centered(v):
        return 0.0 if not v else max(-1.0, min(1.0, (v - 1500) / CENTER_HALF))

    def _throttle(v):
        if not v:
            return 0.0
        return max(0.0, min(1.0, (v - THROT_LO) / (THROT_HI - THROT_LO)))

    # Left stick: yaw (x, ch4) + throttle (y, ch3).
    _stick_box(left_cx, cy, size,
               _centered(ch[3]), _throttle(ch[2]) * 2 - 1, throttle=True)

    # Right stick: roll (x, ch1) + pitch (y, ch2).
    _stick_box(right_cx, cy, size,
               _centered(ch[0]), _centered(ch[1]), throttle=False)

    # Axis labels around each box, plus live µs values below.
    def _decorate_box(cx_box, vert_label, vert_value, horiz_label, horiz_value):
        # vertical-axis label, centered above the box.
        tex, w, h = stick_labels[vert_label]
        _draw_textured_quad(tex, cx_box - w / 2,
                            cy - size / 2 - h - 4, w, h)
        # horizontal-axis label, vertically centered to the LEFT of the box.
        tex, w, h = stick_labels[horiz_label]
        _draw_textured_quad(tex, cx_box - size / 2 - w - 6,
                            cy - h / 2, w, h)
        # Live RX values below the box (two lines, vertical axis on top).
        v_top = f"{vert_label[0].upper()}={vert_value}"
        v_bot = f"{horiz_label[0].upper()}={horiz_value}"
        _, wt, ht = value_cache.get(v_top)
        _, wb, hb = value_cache.get(v_bot)
        y_top = cy + size / 2 + 4
        y_bot = y_top + ht + 2
        value_cache.draw(v_top, cx_box - wt / 2, y_top)
        value_cache.draw(v_bot, cx_box - wb / 2, y_bot)

    _decorate_box(left_cx,  "throttle", ch[2], "yaw",  ch[3])
    _decorate_box(right_cx, "pitch",    ch[1], "roll", ch[0])

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
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 460800

    pygame.init()
    width, height = 1000, 640
    pygame.display.set_mode((width, height), pygame.OPENGL | pygame.DOUBLEBUF)
    pygame.display.set_caption("MPU6050 visualizer")

    glEnable(GL_DEPTH_TEST)
    glClearColor(0.08, 0.08, 0.10, 1.0)
    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, width / height, 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)

    # Pre-render motor labels as GL textures (must happen after set_mode so
    # the GL context exists).
    label_font   = pygame.font.SysFont("monospace", 18, bold=True)
    value_font   = pygame.font.SysFont("monospace", 14)
    stick_font   = pygame.font.SysFont("monospace", 14, bold=True)
    motor_labels = init_motor_labels(label_font)
    value_cache  = TextCache(value_font, color=(200, 200, 210))
    stick_labels = {
        name: _make_text_texture(stick_font, name, color=(180, 200, 230))
        for name in ("throttle", "yaw", "pitch", "roll")
    }

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

        roll, pitch, yaw, temp_c, ch, armed, motors, t_ms = state.snapshot()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        # Shift the camera look-at point along the view's up axis so the cube
        # renders in the lower half of the screen — clear of the motor HUD.
        cube_drop = 2.0
        gluLookAt(view_eye[0], view_eye[1], view_eye[2],
                  view_up[0] * cube_drop,
                  view_up[1] * cube_drop,
                  view_up[2] * cube_drop,
                  view_up[0], view_up[1], view_up[2])
        draw_axes()
        # IMU orientation, applied yaw (Z) → pitch (Y) → roll (X).
        glRotatef(yaw, 0, 0, 1)
        glRotatef(pitch, 0, 1, 0)
        glRotatef(roll, 1, 0, 0)
        draw_cube()
        draw_motors_hud(width, height, motors, armed, motor_labels, value_cache)
        draw_sticks_hud(width, height, ch, stick_labels, value_cache)
        pygame.display.flip()

        # AETR convention: CH1=Aileron, CH2=Elevator, CH3=Throttle, CH4=Rudder.
        state_str = "ARMED  " if armed else "disarm "
        pygame.display.set_caption(
            f"{state_str}[{view_name}]  "
            f"r={roll:+5.0f} p={pitch:+5.0f} y={yaw:+5.0f}  "
            f"M=[{motors[0]} {motors[1]} {motors[2]} {motors[3]}]  "
            f"T={temp_c:.0f}C  [1/2/3 view  Esc]"
        )
        clock.tick(60)

    stop.set()
    pygame.quit()


if __name__ == "__main__":
    main()

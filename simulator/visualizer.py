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
    GL_MODELVIEW, GL_PROJECTION, GL_QUADS, glBegin, glClear, glClearColor,
    glColor3f, glEnable, glEnd, glLoadIdentity, glMatrixMode, glRotatef,
    glTranslatef, glVertex3f,
)
from OpenGL.GLU import gluPerspective


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
    t_ms: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_from_csv(self, line: str) -> bool:
        parts = line.strip().split(",")
        if len(parts) != 10:
            return False
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            return False
        with self.lock:
            (self.ax, self.ay, self.az,
             self.gx, self.gy, self.gz,
             self.roll, self.pitch, self.yaw, t) = vals
            self.t_ms = int(t)
        return True

    def snapshot(self) -> tuple[float, ...]:
        with self.lock:
            return (self.ax, self.ay, self.az,
                    self.gx, self.gy, self.gz,
                    self.roll, self.pitch, self.yaw, self.t_ms)


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

    font = pygame.font.SysFont("monospace", 14, bold=True)
    text_surface = pygame.Surface((300, 220), pygame.SRCALPHA)

    state = ImuState()
    stop = threading.Event()
    reader = threading.Thread(
        target=serial_reader, args=(port, baud, state, stop), daemon=True
    )
    reader.start()

    clock = pygame.time.Clock()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        ax, ay, az, gx, gy, gz, roll, pitch, yaw, t_ms = state.snapshot()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -6.0)
        # Tilt the world a bit so we see the body, not edge-on.
        glRotatef(20, 1, 0, 0)
        draw_axes()
        # Apply IMU orientation. Order: yaw (Z) → pitch (Y) → roll (X).
        glRotatef(yaw, 0, 0, 1)
        glRotatef(pitch, 0, 1, 0)
        glRotatef(roll, 1, 0, 0)
        draw_cube()

        # Overlay text by drawing to a pygame surface — only works in
        # 2D mode, so we render to a separate window? Pygame OpenGL
        # backends don't blit easily, so we just print at low rate.
        pygame.display.flip()
        if pygame.time.get_ticks() % 250 < 20:
            sys.stdout.write(
                f"\rt={t_ms:>7d}ms  acc=({ax:+.2f},{ay:+.2f},{az:+.2f})g  "
                f"gyro=({gx:+7.2f},{gy:+7.2f},{gz:+7.2f})dps  "
                f"rpy=({roll:+6.1f},{pitch:+6.1f},{yaw:+6.1f})  "
            )
            sys.stdout.flush()
        clock.tick(60)

    stop.set()
    pygame.quit()


if __name__ == "__main__":
    main()

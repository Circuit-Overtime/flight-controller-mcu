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
    t_ms: int = 0
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update_from_csv(self, line: str) -> bool:
        parts = line.strip().split(",")
        if len(parts) != 11:
            return False
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            return False
        with self.lock:
            (self.ax, self.ay, self.az,
             self.gx, self.gy, self.gz,
             self.roll, self.pitch, self.yaw,
             self.temp_c, t) = vals
            self.t_ms = int(t)
        return True

    def snapshot(self) -> tuple[float, ...]:
        with self.lock:
            return (self.ax, self.ay, self.az,
                    self.gx, self.gy, self.gz,
                    self.roll, self.pitch, self.yaw,
                    self.temp_c, self.t_ms)


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

        ax, ay, az, gx, gy, gz, roll, pitch, yaw, temp_c, t_ms = state.snapshot()

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
        pygame.display.flip()

        pygame.display.set_caption(
            f"MPU6050 [{view_name}]  "
            f"roll={roll:+6.1f}°  pitch={pitch:+6.1f}°  yaw={yaw:+6.1f}°  "
            f"temp={temp_c:5.1f}°C   [1]persp [2]top [3]front  Esc=quit"
        )
        clock.tick(60)

    stop.set()
    pygame.quit()


if __name__ == "__main__":
    main()

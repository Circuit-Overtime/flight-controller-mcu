# flight-controller-mcu

DIY drone flight controller. Stage 1: bring up MPU6050 IMU on Arduino Mega and
visualize it live. Later stages: port to Teensy 4.x, then a custom PCB running
our own drone OS.

## Layout

- `mcu/imu.ino` — Arduino sketch. Raw-I2C MPU6050 driver, on-boot calibration,
  CSV stream over USB serial.
- `simulator/visualizer.py` — pygame + PyOpenGL viewer that reads the serial
  CSV and renders an orientation cube.

## Wiring (Arduino Mega 2560)

| MPU6050 | Mega    |
| ------- | ------- |
| VCC     | 5V      |
| GND     | GND     |
| SDA     | 20 (SDA)|
| SCL     | 21 (SCL)|
| AD0     | GND     |

I2C address: `0x68`. No external pull-ups needed for short wiring (the GY-521
breakout has 4.7k pull-ups on board).

## Flashing

Open `mcu/imu.ino` in the Arduino IDE, select **Tools → Board → Arduino Mega
2560**, pick the right port, hit upload. Only `Wire.h` is used — no library
install needed.

On boot the sketch prints `# calibrating - keep IMU still and level`, samples
2000 readings to compute offsets, then streams:

```
ax,ay,az,gx,gy,gz,roll,pitch,yaw,t_ms
```

at 100 Hz. Accel in g, gyro in deg/s, angles in degrees.

## Running the visualizer

```bash
cd simulator
python -m venv .venv && source .venv/bin/activate
pip install pygame PyOpenGL pyserial
python visualizer.py /dev/ttyACM0 115200
```

The Mega usually shows up as `/dev/ttyACM0` or `/dev/ttyUSB0` on Linux. Pass
the path as the first argument; default baud is 115200.

Press **Esc** to quit.

## Roadmap

- [x] Raw-I2C driver + calibration + CSV stream
- [x] pygame/OpenGL orientation viewer
- [ ] Complementary filter (fuse accel + gyro for drift-free roll/pitch)
- [ ] Madgwick / Mahony AHRS
- [ ] Magnetometer (HMC5883L or QMC5883L) for absolute yaw
- [ ] Port to Teensy 4.1
- [ ] ESC + motor mixing
- [ ] Custom flight-controller PCB

# F450 flight-controller system specification

## 1. Project status and scope

This aircraft is an experimental, Arduino Mega–based F450 X-quad intended for
a low, manually throttled, self-levelled first-hover test. It has not yet been
flight-qualified. Prop-off receiver, IMU, axis-sign, failsafe, and motor-response
tests remain mandatory before fitting propellers.

Altitude hold is outside the first-flight scope. The available BMP180 pressure
sensor remains disconnected until basic attitude control has been validated.

## 2. Airframe

| Property | Specification |
| --- | --- |
| Frame | F450, carbon-fibre/aluminium construction |
| Geometry | X quadcopter |
| Motor-to-motor diagonal | 450 mm |
| Centre-to-motor distance | 225 mm |
| Payload | None |
| Intended mode | Self-levelled roll/pitch, yaw-rate control, manual throttle |
| Intended first test | Very low hover in a clear outdoor test area |

The battery is mounted centrally and distributed along the aircraft's
longitudinal axis. The reported centre of gravity is near the geometric centre,
but this must be verified with the complete aircraft assembled.

## 3. Mass estimate

| Component | Unit mass | Quantity | Total |
| --- | ---: | ---: | ---: |
| Frame | 330 g | 1 | 330 g |
| A2212 motors | 60 g | 4 | 240 g |
| Nylon propellers | 8 g | 4 | 32 g |
| Battery | 300 g | 1 | 300 g |
| Electronics and wiring | approximately 100 g | 1 set | approximately 100 g |
| **Estimated all-up mass** | | | **approximately 1002 g** |

The complete aircraft must be weighed on a scale before flight. The estimate
does not establish thrust margin.

## 4. Coordinate system and IMU installation

- The physical front of the aircraft is aligned with the MPU6050's positive X
  axis.
- The MPU6050 breakout is mounted flat and rigidly on the central breadboard.
- Gyroscope X is used for roll rate, Y for pitch rate, and Z for yaw rate.
- Positive-Y direction and all measured attitude signs must be verified using
  the prop-off axis-sign log before powered lift.
- Rigid mounting transfers frame, motor, and propeller vibration directly to
  the sensor; logged gyro and accelerometer noise must be reviewed before
  increasing derivative gain.

## 5. Motor layout, direction, and outputs

Aircraft viewed from above, with the front in the positive-X direction:

```text
                    FRONT (+X)

             M4                     M1
        front-left, CW        front-right, CCW
             pin 46                 pin 52


             M3                     M2
         rear-left, CCW        rear-right, CW
             pin 48                 pin 44

                     REAR
```

| Motor | Position | Rotation viewed from above | Mega signal pin |
| --- | --- | --- | ---: |
| M1 | Front-right | Counter-clockwise (CCW) | 52 |
| M2 | Rear-right | Clockwise (CW) | 44 |
| M3 | Rear-left | Counter-clockwise (CCW) | 48 |
| M4 | Front-left | Clockwise (CW) | 46 |

Each propeller must have the correct handedness for its motor direction and
must push air downward. Motor direction, propeller type, and motor numbering
must all be physically rechecked before fitting propellers.

The X-quad mixer is:

```text
M1 = throttle - roll - pitch + yaw
M2 = throttle - roll + pitch - yaw
M3 = throttle + roll + pitch + yaw
M4 = throttle + roll - pitch - yaw
```

## 6. Propulsion and power

| Item | Specification |
| --- | --- |
| Motors | Four A2212 10T brushless motors |
| Motor KV | 1400 KV; motor-label confirmation required because 14000 KV was initially reported |
| Propellers | 10 × 4.5 inch, two-blade nylon |
| ESCs | Four SparkFun 30 A ESCs |
| ESC calibration | All four throttle-calibrated together |
| Current BEC wiring | Four BEC 5 V outputs paralleled; no observed heating, datasheet compatibility unverified |
| ESC signalling | Standard Servo PWM at 50 Hz |
| Disarmed command | 1000 us |
| Reliable continuous-spin floor | 1200 us on all four motors |
| Maximum configured command | 1800 us |
| ESC boot-low hold | 2500 ms |
| Battery | 3S, 11.1 V nominal, 3200 mAh, 40C |
| Battery connector | XT60 |

Battery chemistry still requires confirmation. The original notes identify it
as Li-ion, but the low-voltage policy must not be finalized until Li-ion versus
LiPo is confirmed.

## 7. Flight electronics and wiring

### Flight computer

- Arduino Mega 2560
- Current implementation uses the Arduino Servo library for the ESC outputs.
- Pin 52 is also the Mega SPI SCK pin, so M1 currently prevents simultaneous
  use of hardware SPI.

### Power-source topology

The present build ties all four ESC BEC 5 V outputs to the same 5 V rail. That
arrangement has operated without observed abnormal current or heating. However,
no exact ESC datasheet has been provided that permits their regulator outputs
to share current, so electrical compatibility remains unverified.

Conservative validation wiring:

- All ESC grounds remain connected to the common ground.
- All four ESC signal wires remain connected to their assigned Mega pins.
- For USB-powered bench logging with the flight battery connected, disconnect
  and individually insulate all four ESC red/BEC wires so USB is the only
  source driving the Mega 5 V rail.
- For battery-only operation without USB, use one verified 5 V BEC output or a
  dedicated regulator sized for the Mega, receiver, IMU, and indicators. Leave
  the other ESC BEC outputs disconnected and insulated.
- Do not connect the 3S battery directly to the Mega 5 V pin.

The exact ESC label/datasheet and measured BEC output are still required to
close this power-system verification item. USB and powered BEC outputs must not
be combined on the 5 V rail until their isolation behavior is established.

### MPU6050

| Property | Setting |
| --- | --- |
| Interface | I2C at 400 kHz |
| Address | 0x68 |
| Bus timeout | 3000 us with TWI reset |
| Output rate | 200 Hz |
| Digital low-pass filter | Configuration 3, approximately 44 Hz |
| Gyroscope range | ±250 degrees/s |
| Accelerometer range | ±2 g |
| Boot calibration | 2000 stationary samples |

The BMP180 is not connected for first flight.

### FlySky FS-R6B receiver

| Channel | Function | Mega pin |
| --- | --- | --- |
| CH1 | Roll | A8 / PCINT16 |
| CH2 | Pitch | A9 / PCINT17 |
| CH3 | Throttle | A10 / PCINT18 |
| CH4 | Yaw | A11 / PCINT19 |
| CH5 | Unused | A12 / PCINT20 |
| CH6 | Unused | A13 / PCINT21 |

- PWM capture uses the Port K pin-change interrupt.
- Only pulses from 800 through 2200 us are accepted.
- Invalid pulses are rejected and counted; they are never applied to control.
- A channel returns zero only before its first valid pulse. Thereafter the last
  valid value is retained with a timestamp.
- If any flight-critical channel supplies no valid pulse for 250 ms, the
  aircraft disarms immediately.
- The transmitter-off test must establish whether the receiver stops pulses,
  outputs a configured failsafe value, or holds the last command indefinitely.

### Status LEDs

| LED | Mega pin | Purpose |
| --- | ---: | --- |
| STARTUP | 24 | Boot-complete and RX-failsafe status |
| CALIB | 26 | Calibration, armed state, and IMU fault |
| TEMP | 28 | MPU6050 high-temperature warning |
| BATTERY | 30 | Low-battery warning |

### Battery measurement

The firmware expects a 10 kΩ / 3.3 kΩ divider feeding Mega A0, using the 5 V
ADC reference. It samples at 5 Hz and currently warns below 10.0 V. The divider
installation, ADC calibration, and battery chemistry must be confirmed before
the warning threshold is treated as valid. Low voltage currently warns through
an LED; it does not automatically disarm in flight.

## 8. Receiver mapping and arming

| Property | Current setting |
| --- | --- |
| Centred-stick reference | 1500 us |
| Centred-stick deadband | ±15 us |
| Effective roll/pitch/yaw span | 1500 ± 300 us |
| Measured raw roll | 1092 / 1490 / 1882 us (left/centre/right means) |
| Measured raw pitch | 1125 / 1430 / 1799 us (down/centre/up means) |
| Measured raw throttle | 1124 / 1466 / 1808 us (low/midpoint/high means) |
| Measured raw yaw | 1052 / 1462 / 1830 us (left/centre/right means) |
| Configured throttle input span | 1120-1700 us; upper cap reserves PID authority |
| Maximum commanded tilt | ±25 degrees |
| Maximum commanded yaw rate | ±120 degrees/s |

Arming requires throttle at or below 1160 us and yaw at or above 1800 us for
1500 ms. Disarming requires throttle at or below 1160 us and yaw at or below
1200 us for 1500 ms. Roll, pitch, and yaw setpoints remain zero for 1000 ms
after arming so the pilot can release the yaw gesture.

## 9. Estimation and control

- Flight-control and MPU update rate: 200 Hz.
- Roll and pitch: accelerometer/gyro complementary filter with alpha 0.98.
- Yaw: integrated gyro for display; yaw control uses gyro rate because there is
  no magnetometer or absolute-heading reference.
- Roll and pitch use cascaded control:
  - outer proportional angle loop produces a rate setpoint;
  - inner PID rate loop produces a PWM correction.
- Yaw uses a direct rate PID.
- PID derivative is taken from measurement to avoid derivative kick.
- Integrators are clamped and reset whenever disarmed.
- Mixer outputs are independently clamped to 1200-1800 us while armed.

Current pre-tuning gains:

| Controller | Kp | Ki | Kd |
| --- | ---: | ---: | ---: |
| Roll angle | 3.0 | 0 | 0 |
| Pitch angle | 3.0 | 0 | 0 |
| Roll rate | 0.50 | 0.30 | 0.005 |
| Pitch rate | 0.50 | 0.30 | 0.005 |
| Yaw rate | 1.50 | 0.50 | 0 |

Rate PID integral output is limited to ±100 us and total per-axis PID output to
±300 us. These gains are initial estimates only and must be revised from
measured logs.

## 10. Safety behavior

- ESCs receive 1000 us throughout startup and whenever disarmed.
- Four consecutive scheduled IMU read failures, approximately 20 ms at 200 Hz,
  force disarm, reset every PID, set motor telemetry to 1000 us, and require a
  new arm gesture after sensor recovery.
- Invalid PID inputs reset the affected controller rather than allowing
  NaN/Infinity to reach the mixer.
- RX timeout disarms as soon as the 250 ms channel-validity window expires.
- Motor commands shown while disarmed are predicted mixer outputs for bench
  analysis; physical ESC outputs remain at 1000 us.
- Low battery and high MPU temperature currently provide warnings only.
- There is no altitude hold, GPS recovery, magnetometer heading hold, or
  autonomous landing.

## 11. Telemetry and logging

| Property | Setting |
| --- | --- |
| Baud rate | 460800 |
| Stream rate | 50 Hz |
| Format | Extended CSV |

Telemetry includes attitude, temperature, corrected and raw RX values, armed
state, motor commands, calibrated accelerometer and gyro data, control
setpoints, PID corrections, loop timing, failsafe state, battery voltage, RX
alive bits, and rejected-pulse counters.

Use `simulator/capture_telemetry.py` and follow `TUNING.md` to collect:

1. stationary sensor/RX baseline;
2. complete receiver ranges;
3. transmitter-loss behavior;
4. roll, pitch, and yaw sign verification.

## 12. Physical-build requirements before first flight

1. Replace or secure the solderless breadboard and loose jumper connections.
   Flight-critical IMU, RX, ESC-signal, ground, and power connections require
   vibration-resistant soldered or locking connections with strain relief.
2. Confirm only an electrically valid power arrangement feeds the Mega; do not
   parallel multiple ESC BEC outputs unless their design explicitly permits it.
3. Confirm battery chemistry and voltage-divider installation.
4. Confirm M1-M4 positions, all four motor directions, and propeller handedness.
5. Verify the centre of gravity with the complete aircraft.
6. Measure actual all-up mass.
7. Review all four required prop-off logs.
8. Transmitter-loss detection is verified. Repeat an armed prop-off loss test
   later to observe the physical disarm transition under armed state.
9. Perform motor-correction tests with propellers removed: the motors on the
   physically lowered side must receive more command.
10. Establish thrust margin and hover throttle before attempting PID tuning in
    free flight.

The aircraft is not cleared for an indoor first hover.

## 13. Verified receiver results

After correcting the RX timestamp-order race, a 427-row stationary hardware
capture produced:

- RX alive mask 15 in every row;
- zero failsafe rows;
- zero rejected pulses on CH1-CH4;
- 5054 us mean control-loop interval.

This verifies the A8-A11 mapping and normal receiver health.

A subsequent transmitter-off capture showed 702 healthy rows at alive mask 15,
one staggered-expiry row at mask 8, then 579 consecutive rows at mask 0 with
failsafe asserted. No pulses were rejected. The raw values remained frozen at
their last valid measurements while their timestamps expired, confirming that
the FS-R6B stops refreshing usable PWM and the firmware detects link loss.

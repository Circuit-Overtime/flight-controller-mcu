# Flight-controller hardware and airframe record

This file records the hardware configuration used when choosing safety limits,
motor mixing, and initial PID gains. Update measured values here whenever the
airframe changes.

## Airframe

| Item | Specification |
| --- | --- |
| Frame | F450 X-quad, carbon-fibre/aluminium construction |
| Motor-to-motor diagonal | 450 mm |
| Centre-to-motor distance | 225 mm |
| Frame mass | 330 g |
| Estimated electronics mass | approximately 100 g |
| Centre of gravity | Battery centred along the longitudinal (Y) axis; reported near the geometric centre |
| Intended first test | Low, manually throttled, self-levelled hover |

## Propulsion

| Position | Motor | Rotation viewed from above | Signal pin |
| --- | --- | --- | --- |
| M1, front-right | A2212 10T, 1400 KV | CCW | Mega 52 |
| M2, rear-right | A2212 10T, 1400 KV | CW | Mega 44 |
| M3, rear-left | A2212 10T, 1400 KV | CCW | Mega 48 |
| M4, front-left | A2212 10T, 1400 KV | CW (confirmation required) | Mega 46 |

- Motor mass: 60 g each; 240 g total.
- Propellers: 10 x 4.5 inch, two-blade nylon, 8 g each; 32 g total.
- ESCs: four SparkFun 30 A units, throttle-calibrated together.
- Present power wiring: all four ESC BEC 5 V outputs are tied to the common
  5 V rail. The builder reports no abnormal current or heating in prior use,
  but parallel compatibility has not been verified from an exact ESC datasheet.
- Reliable continuous-spin threshold: approximately 1200 us on each motor.
- Reported stable command range: approximately 1200-1800 us.

## Power

| Item | Specification |
| --- | --- |
| Battery | 3S, 11.1 V nominal, 3200 mAh, 40C |
| Connector | XT60 |
| Battery mass | 300 g |

Battery chemistry (LiPo versus cylindrical Li-ion pack) still requires
confirmation. Firmware voltage thresholds must match the confirmed chemistry.

## Flight electronics

| Item | Specification |
| --- | --- |
| Flight computer | Arduino Mega 2560 |
| IMU | MPU6050 breakout |
| IMU mounting | Flat and rigid on the central breadboard |
| IMU orientation | Sensor +X aligned with the physical front of the aircraft |
| Receiver | FlySky FS-R6B, individual PWM channels |
| Receiver order | CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw |
| Pressure sensor | BMP180 available but intentionally disconnected for first flight |

Rigid IMU mounting can transmit motor/propeller vibration directly into the
gyro and accelerometer. Stationary and powered-motor logs must be checked before
raising derivative gain.

## Mass estimate

| Component group | Mass |
| --- | ---: |
| Frame | 330 g |
| Four motors | 240 g |
| Four propellers | 32 g |
| Battery | 300 g |
| Electronics | approximately 100 g |
| **Estimated all-up mass** | **approximately 1002 g** |

The estimated value is suitable only for initial modelling. Record the complete
aircraft's measured scale weight before flight.

## Items requiring confirmation

1. Confirm the motor marking is 1400 KV, not 14000 KV.
2. Confirm M4 rotates clockwise when viewed from above.
3. Confirm battery chemistry.
4. Measure the complete all-up mass on a scale.
5. Identify the exact ESC/BEC model, rated BEC voltage/current, and whether its
   manufacturer explicitly allows BEC outputs to operate in parallel.
6. Establish a documented method that prevents USB and powered ESC BEC outputs
   from driving the Mega 5 V rail against one another.
7. Measure each motor's individual continuous-spin threshold after the final
   propellers and power system are installed.

## Verified captures

- RX health after timestamp-race fix: 427/427 rows reported alive mask 15,
  zero failsafe rows, and zero rejected pulses on CH1-CH4.
- Transmitter-off test: the FS-R6B stopped refreshing flight-channel PWM.
  Firmware transitioned from mask 15 through one staggered mask-8 row to mask
  0, asserted failsafe, and retained failsafe for all remaining 579 rows. The
  operator reported switching the transmitter off at about 10 s, while stale
  PWM was first observed roughly 6-7 s later. This estimated receiver-side
  delay requires a cue-timed repeat and is not accepted for flight.

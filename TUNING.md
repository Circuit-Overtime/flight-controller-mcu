# First-flight logging and tuning procedure

PID values cannot be selected reliably from motor KV and airframe mass alone.
The hardware record in `BOM.md` provides conservative limits; measured logs
provide sensor noise, receiver range, timing jitter, axis signs, and the
airframe response needed to tune the controller.

## Telemetry

Firmware streams extended CSV at 50 Hz and 460800 baud. The first 16 fields
remain compatible with `visualizer.py`; appended fields contain:

- calibrated accelerometer and gyro measurements;
- angle, rate, yaw, and throttle setpoints;
- roll/pitch rate setpoints and PID outputs;
- measured control-loop time;
- failsafe and battery state;
- unsmoothed receiver pulse widths;
- receiver alive bits and rejected-pulse counters.

`rx_alive_mask` uses bit 0 for CH1 through bit 5 for CH6. With only CH1-CH4
wired, the expected healthy mask is decimal 15. A missing critical bit means
that channel has not delivered a valid pulse within 250 ms. Values outside
800-2200 us are rejected, counted, and never applied to control.

## Capture command

```bash
mkdir -p logs
python simulator/capture_telemetry.py /dev/ttyACM0 460800 60 logs/stationary.csv
```

Opening the serial port may reset the Mega. The tool waits up to 30 seconds for
boot calibration and then records the requested duration after receiving the
CSV header. Firmware event lines are written beside the CSV as
`stationary.events.txt`.

## Required prop-off captures

Keep all propellers removed for every test in this section.

1. **Stationary baseline — 60 seconds**
   - Place the complete aircraft on a rigid, level surface.
   - Install the flight battery in its flight position.
   - Turn on the transmitter and leave all controls untouched.
   - Do not touch the table or airframe during capture.

2. **Receiver ranges — 60 seconds**
   - Move only one control at a time, slowly.
   - For roll, pitch, and yaw: hold minimum for 3 seconds, centre for 3
     seconds, then maximum for 3 seconds.
   - For throttle: hold minimum and maximum for 3 seconds each.
   - Return every control to its normal resting position.

3. **Receiver loss — 30 seconds**
   - Start with the transmitter on and receiver healthy.
   - Switch the transmitter off approximately 10 seconds into the capture.
   - Confirm `failsafe=1`, an RX alive-mask bit clears, and armed output cannot
     persist. Perform this test without propellers.

4. **Axis-sign test — 60 seconds**
   - Begin level.
   - Lift and lower the right side, then the left side.
   - Lift and lower the nose, then the tail.
   - Rotate clockwise and counter-clockwise while keeping the frame level.
   - Pause level for several seconds between movements.

Use distinct output names such as `stationary.csv`, `rx-ranges.csv`,
`rx-loss.csv`, and `axis-signs.csv`. Do not combine the tests; separate
files make phase boundaries and faults unambiguous.

## Review gates before powered lift

The logs must establish all of the following before changing PID gains:

1. No unexplained rejected RX pulses or loss of critical alive-mask bits.
2. Correct minimum, centre, and maximum for CH1-CH4.
3. Gyro bias and stationary noise small enough for derivative control.
4. Accelerometer magnitude close to 1 g while stationary.
5. Roll, pitch, and yaw signs match the mixer and physical airframe.
6. Control-loop timing remains close to 5000 us without large periodic stalls.
7. Receiver loss produces a disarm after the configured 250 ms validity window.

Only after these gates pass should motor-on response tests be designed. Never
run a propeller-equipped vibration or PID test on a loose indoor airframe.

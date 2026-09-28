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

## Guided receiver endpoint capture

Do not enter receiver limits from specifications or visual estimates. With
propellers removed, use the interactive tool to record exactly 100 fresh
samples at every requested physical stick position:

```bash
python simulator/guided_rx_capture.py /dev/ttyACM0 460800 100 logs/rx-guided.csv
```

For each phase, set and hold the requested position before pressing Enter. The
tool then clears queued serial input and captures exactly 100 new rows. It
records raw receiver widths together with the simultaneous IMU, failsafe,
timing, setpoint, PID, and motor data. It also prints immediate per-channel
mean, standard deviation, minimum, maximum, and RX-alive status.

The guided phases are:

1. centred roll/pitch/yaw with throttle lowest;
2. roll fully left and fully right;
3. pitch fully down and fully up;
4. throttle midpoint and highest;
5. yaw fully left and fully right with throttle at midpoint, preventing the
   low-throttle yaw-right arming gesture;
6. a final centred/low position to measure centre drift.

Firmware stick endpoints, centres, deadband, direction, and arming thresholds
must not be changed until this capture has been reviewed. The tool aborts
immediately if telemetry reports that the controller became armed.

## Guided IMU alignment capture

After receiver health passes, capture the real MPU mounting and axis signs:

```bash
python simulator/guided_imu_capture.py /dev/ttyACM0 460800 100 logs/imu-guided.csv
```

The tool requests repeated level holds plus right/left roll, nose-down/nose-up
pitch, and clockwise/counter-clockwise yaw motions. Static and motion phases are
labelled in the CSV. Each phase contains exactly 100 fresh samples.

For a movement phase, place the aircraft in the stated start position, press
Enter, immediately perform one smooth motion, and hold the final position until
capture completes. Do not move the transmitter controls. The tool aborts if the
controller arms, receiver failsafe activates, or any flight channel becomes
stale.

## Required prop-off captures

Keep all propellers removed for every test in this section.

1. **Stationary baseline — 60 seconds**
   - Place the complete aircraft on a rigid, level surface.
   - Install the flight battery in its flight position.
   - Turn on the transmitter and leave all controls untouched.
   - Do not touch the table or airframe during capture.

2. **Receiver ranges**
   - Use the guided receiver endpoint capture above.

3. **Receiver loss — 30 seconds**
   - Start with the transmitter on and receiver healthy.
   - Run the timed loss capture:
     ```bash
     python simulator/capture_telemetry.py /dev/ttyACM0 460800 30 \
       logs/rx-loss-repeat.csv --tx-off-after 10
     ```
   - Switch the transmitter off at the printed cue. The cue is recorded in the
     event file so receiver and firmware detection latency can be measured.
   - Confirm `failsafe=1`, an RX alive-mask bit clears, and armed output cannot
     persist. Perform this test without propellers.

4. **Axis-sign test — 60 seconds**
   - Use the guided IMU alignment capture above.

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
7. Receiver loss produces a disarm promptly. The firmware's 250 ms validity
   window begins only after the receiver stops refreshing PWM; receiver-side
   hold or failsafe latency must be measured separately from that window.

Normal RX continuity has passed on the current hardware. The first transmitter-
off capture eventually asserted firmware failsafe, but the approximate 6-7 s
delay between the operator's switch-off and stale PWM is not cleared for flight.
Repeat with the timed cue above, then configure the radio system's native
failsafe if supported. An armed, prop-off loss test remains a later physical-
output check.

Only after these gates pass should motor-on response tests be designed. Never
run a propeller-equipped vibration or PID test on a loose indoor airframe.

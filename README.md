# SOP for using pulse area control v3.15

Files: `redpitaya_pid_3.15.bit`, `run_pulse_control.py`, `pulse_control.py`  
Order: wire up → load FPGA → configure → measurement only → calibrate → feedback

## 1. Wiring


| Red Pitaya | Connect to | Purpose |
|---|---|---|
| `IN1` | Pick-off photodiode output | Actual optical pulse measurement (ADC). Check polarity and keep within the configured ADC range. |
| `OUT2` | **Control input** of the relevant actuator / test setup | Programmable DAC waveform; feedback changes its effective HIGH level. Confirm the destination and voltage scaling before connecting. |
| `DIO0_P` | External TTL/reference trigger | Starts each reference window in `GROUP_SOURCE="external"`. |
| `DIO1_P` | External device trigger input or oscilloscope | Programmable GPIO pulse output. |
| `DIO2_P` | Oscilloscope/logic analyser | `reference_window_active` marker. Use to check which pulses are actually inside the measurement window. |

**Important:** The code fixes the Red Pitaya pin functions, but not the lab-side wiring (e.g. whether OUT2 drives the AOM controller directly or through another stage). Confirm the exact actuator input before connecting. GPIO pins are not BNC analogue inputs: use the correct expansion connector, ground and logic-level interface. Do not feed analogue or out-of-range signals into the GPIO.

For an oscilloscope check, monitor the trigger, DIO2 marker, photodiode waveform and OUT2 (as channels permit). Verify timing and voltage before enabling feedback.

## 2. Copy files and load the bitstream

Copy the three files to the Red Pitaya (e.g. `/root/`). Use exactly the same PC08 driver and runner pair; do not mix them with older PC07 files.

```bash
# From your computer; replace <RP_IP> with the board IP
scp redpitaya_pid_3.15.bit 'run_pulse_control.py' 'pulse_control.py' root@<RP_IP>:/root/
ssh root@<RP_IP>
cd /root

# Stop any old control/GUI process before reprogramming.
# Confirm the bitstream is present, then load it:
ls -lh redpitaya_pid_3.15.bit
cat /root/redpitaya_pid_3.15.bit > /dev/xdevcfg

# Check that FPGA and PC08 Python agree
python3 run_pulse_control.py show
```

`/dev/xdevcfg` must exist on the board's OS. If it does not, **do not** substitute a guessed device path; check how this Red Pitaya image programs the PL. If `show` reports an FPGA version mismatch, stop and correct the bitstream/driver pairing.

## 3. Edit `run_pulse_control.py`

Only change the configuration block at the top, above `Sections below do not need any changes`. Keep `pulse_control.py` unchanged for routine runs. Use `nano run_pulse_control.py` or edit on the PC and copy the file back.

**Trigger/reference**

```python
GROUP_SOURCE = "external"         # DIO0 external trigger; or "dac2" for each DAC2 period
EXTERNAL_TRIGGER_FREQUENCY_HZ = 1_000_000.0
EXTERNAL_TRIGGER_MIN_HIGH_POINTS = 5
EXTERNAL_TRIGGER_DELAY_NS = 650.0

REFERENCE_TARGET_START_INDEX = 0
REFERENCE_TARGET_PULSE_COUNT = 2
```

`DIO2_P` marks the reference window; pulse indices start from its rising edge. Here, pulses **0 and 1** are summed. In external mode, adjust `EXTERNAL_TRIGGER_DELAY_NS` until the intended pulses fall inside the window; the 5 ns steps are applied **after trigger qualification**. In `dac2` mode, the DAC2 period boundary defines the reference; external delay settings are bypassed.

**GPIO and DAC2 outputs**

```python
GPIO_ENABLED = True
GPIO_FREQUENCY_HZ = 1_000_000.0
GPIO_PULSES = [(0.000, 0.010)]

DAC2_ENABLED = True
DAC2_FREQUENCY_HZ = 25_000.0
DAC2_LOW_VOLTAGE_V = 0.0
DAC2_HIGH_VOLTAGE_V = 0.500
# Edit DAC2_SEQUENCE below these lines for the required pulse pattern.
```

GPIO pulse windows are fractions of the GPIO period. `DAC2_SEQUENCE` is `(normalised level, fraction of DAC2 period)`, with at most 32 segments; **all fractions must sum to 1**. At 25 kHz, one DAC2 period is 40 µs. DAC2 runs on the 125 MHz clock (8 ns resolution); GPIO/trigger timing uses 200 MHz (5 ns). The runner's voltage values are *signed DAC values*, not necessarily the physical voltage: on the noted modified board `-1 / 0 / +1 V` corresponds to `0 / 1 / 2 V` physical. Verify on the scope.

**ADC measurement**

```python
TRIGGER_FRACTION = 0.25
TARGET_HEIGHT_V = 0.100
PRE_BASELINE_SAMPLES = 8
POST_BASELINE_SAMPLES = 8
MIN_PULSE_SAMPLES = 3
MAX_PULSE_SAMPLES = 16384
ADC_SATURATION_LIMIT_V = 0.95
PULSE_POLARITY = "positive"     # "negative" if pulses go down

EXPECTED_PULSES_PER_GROUP = 0
MIN_VALID_PULSES_PER_GROUP = 1
```

Trigger threshold is approximately `baseline + TRIGGER_FRACTION × TARGET_HEIGHT_V` for positive pulses. For a different pulse height/noise level, set these using the **measured IN1 waveform**, not the nominal OUT2 voltage. Baseline windows must fit between pulses; reduce them for closely spaced pulses. Set `EXPECTED_PULSES_PER_GROUP` only when the full-group expected count is known.

**Leave feedback OFF for the first run:**

```python
FEEDBACK_SOURCE = "selected"
FEEDBACK_ENABLED = False
TARGET_FEEDBACK_AREA_V_US = None
USE_FIRST_VALID_FEEDBACK_AS_TARGET = True
AREA_TO_DAC_GAIN_V_PER_V_US = None
```

`selected` controls the sum of the selected reference pulses; `average` uses the full-group mean. Their target areas are not interchangeable. `FEEDBACK_ENABLED` alone is **not** the switch used in this SOP: the explicit `feedback-on` command enables the live loop after configuration.

## 4. Apply and check outputs

```bash
python3 run_pulse_control.py apply
python3 run_pulse_control.py show
```

`apply` writes the edited configuration, leaves measurement and feedback OFF, and enables the configured GPIO/DAC2 outputs. Check the oscilloscope first: OUT2 amplitude/pattern, DIO1 timing, DIO0 trigger and the expected detector waveform on IN1. If the output is wrong, fix it **before** starting measurement.

Optional isolated DAC checks (these replace the normal running DAC2 sequence, so run `apply` again afterwards):

```bash
python3 run_pulse_control.py dac2-dc --counts 0
python3 run_pulse_control.py dac2-square --low-counts 0 --high-counts 1000 --frequency 100 --duty 0.5
python3 run_pulse_control.py apply
```

## 5. Measurement only (open loop)

```bash
python3 run_pulse_control.py measurement-on
python3 run_pulse_control.py show
python3 run_pulse_control.py snapshot-once
python3 run_pulse_control.py monitor --interval 0.2 --count 20
```

Check the following in `show`/snapshot:

- Measurement enabled/config valid; source is the requested `external` or `dac2`.
- In external mode, the accepted trigger count increases. The DIO2 window lines up with the intended pulses.
- `reference_valid=1`, the expected selected pulse IDs/count appear, and the reported `reference_sum` is sensible.
- Invalid pulses, trigger overruns, incomplete references and FIFO overflows are not accumulating. If they are, fix timing, pulse threshold/baselines, selection or input waveform before moving on.

For a raw detector trace with the reference marker:

```bash
python3 run_pulse_control.py waveform-capture --csv waveform.csv
```

Collect measurement-only data (this command starts its own session and returns to measurement OFF afterwards):

```bash
python3 run_pulse_control.py sample --duration-s 10 --interval-s 0.1 --csv measurement.csv
```

Each row is one sparse, selected-reference snapshot. Check `snapshot_complete`, `reference_valid`, `reference_sum_area_v_us` and the individual `target*_pulse_area_v_us` columns. A 0.1 s sampling interval is the **CSV readout cadence**, not the FPGA feedback update period.

## 6. Calibrate the area-to-DAC gain

Keep feedback OFF. Take a measurement-only dataset at one DAC2 HIGH setting, then change **only** `DAC2_HIGH_VOLTAGE_V`, run `apply` again and repeat under the same optical/trigger conditions. Example:

```python
# First run
DAC2_HIGH_VOLTAGE_V = 0.500

# Second run: edit this one value, re-apply and re-measure
DAC2_HIGH_VOLTAGE_V = 0.520
```

For each run, record the mean of **valid and complete** selected-reference sums, `A1` and `A2`, in V·µs. Calculate

```text
AREA_TO_DAC_GAIN_V_PER_V_US = abs((V2 - V1) / (A2 - A1))
```

Use the absolute magnitude for the gain and set `DAC_CORRECTION_SIGN` from the observed direction. For the normal case where increasing OUT2 increases pulse area, the configured sign `-1.0` makes a positive area error reduce OUT2. If the measured slope has the opposite sign, reverse it. A negligible/noisy `A2-A1` is **not** a valid calibration: use a resolvable but safe DAC step and repeat.

Select the area target in the **same units and aggregation** as `FEEDBACK_SOURCE`: for `selected`, it is the **sum** of the selected pulses (not a per-pulse mean). Either enter a fixed target or intentionally auto-latch the first valid reference; do not mix these approaches.

## 7. Feedback (closed loop)

Stop the open-loop session first, then edit the feedback section:

```python
FEEDBACK_SOURCE = "selected"
TARGET_FEEDBACK_AREA_V_US = None     # Or set the chosen measured sum in V*us
USE_FIRST_VALID_FEEDBACK_AS_TARGET = True

AREA_ERROR_DEADBAND_V_US = None
AREA_ERROR_DEADBAND_FRACTION = 0.005
KP = 0.20
KI_PER_S = 2.0
KD = 0.0
AREA_TO_DAC_GAIN_V_PER_V_US = ...  # Replace with measured, positive gain
DAC_CORRECTION_SIGN = -1.0         # Confirm from the calibration slope
MAX_DAC_STEP_PER_UPDATE_V = 0.020
DAC_HIGH_VOLTAGE_MIN_V = DAC2_LOW_VOLTAGE_V + 0.010
DAC_HIGH_VOLTAGE_MAX_V = 1.000
MAX_INTEGRAL_AREA_TERM_FRACTION = 0.25
```

Set sensible DAC clamps for the connected device. Start with conservative gains and check the correction direction before increasing them. `KI_PER_S` is per second; for an external reference source the script derives the update interval from `EXTERNAL_TRIGGER_FREQUENCY_HZ`.

```bash
python3 run_pulse_control.py measurement-off
python3 run_pulse_control.py apply
python3 run_pulse_control.py measurement-on
python3 run_pulse_control.py snapshot-once
python3 run_pulse_control.py feedback-on
python3 run_pulse_control.py show
python3 run_pulse_control.py monitor --interval 0.2 --count 50
```

Check `feedback enabled/config`, target latch (if auto-target), error, P/I/D, pending/active correction and effective OUT2 HIGH. Confirm experimentally that correction **reduces** an imposed area error; watch for DAC clamps, oscillation and drift. If the response runs away, switch feedback OFF immediately and re-check sign/gain and loop settings.

To take a bounded closed-loop CSV instead of leaving the session running:

```bash
python3 run_pulse_control.py feedback-off
python3 run_pulse_control.py sample --duration-s 10 --interval-s 0.1 --csv feedback.csv --feedback
```

`sample --feedback` resets/restarts a fresh measurement/feedback session, logs sparse snapshots and switches feedback/measurement OFF on exit. Do not use it if the experiment needs the existing live PID target/integrator state preserved. Compare `measurement.csv` and `feedback.csv` using valid, complete rows and the same optical conditions.

## 8. Stop / reset

```bash
python3 run_pulse_control.py feedback-off
python3 run_pulse_control.py measurement-off
python3 run_pulse_control.py outputs-off
python3 run_pulse_control.py show
```

`measurement-off` also resets the active reference window/state; `outputs-off` disables GPIO and DAC2. **Do not use `clear-state` casually while feedback is running:** that command switches feedback OFF and clears the controller state. Re-run `apply` after changing parameters, after isolated DAC tests or after reloading the FPGA.

## Quick fault checks

| Symptom | Check first |
|---|---|
| `show` fails/version mismatch | Correct 3.15 bitstream + PC08 `pulse_control.py`/runner pairing; correct device/programming method. |
| No accepted triggers | DIO0 wiring, common ground, logic level, pulse width vs `EXTERNAL_TRIGGER_MIN_HIGH_POINTS`, correct `GROUP_SOURCE`. |
| Trigger counts rise but invalid/missing references | DIO2 timing, external delay, reference pulse indices, ADC polarity/threshold/baseline windows. |
| Waveform clipped | IN1 voltage, ADC saturation setting and detector gain; do not treat clipped area as valid. |
| Feedback won't enable | `AREA_TO_DAC_GAIN_V_PER_V_US` must be set; measurement must already be ON; check `show` config/selected source. |
| Correction goes the wrong way / saturates | Gain units, correction sign, DAC physical voltage mapping and HIGH clamps. Disable feedback before troubleshooting. |

**Note:** Values shown above are the defaults in the supplied `run_pulse_control(3).py`, not independently verified settings for every optical configuration. Re-check timing and voltage whenever the connected experiment changes.


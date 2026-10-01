# Red Pitaya Pulse Control SOP

**Files:** `redpitaya_pid_3.15.bit`, `run_pulse_control.py`, `pulse_control.py`  
**Order:** wire up → load FPGA → configure → measurement only → calibrate → feedback
## 1. Wiring

| Red Pitaya | Connect to | Purpose |
|---|---|---|
| `IN1` | Pick-off photodiode | ADC |
| `OUT2` |Input of the  actuator | Programmable DAC waveform |
| `DIO0_P` | External trigger | External triggering when `GROUP_SOURCE="external"`. |
| `DIO1_P` | GPIO | GPIO pulse output. |
| `DIO2_P` | Oscilloscope| `reference_window_active` marker, showing measurement window. |

## 2. Load the bitstream and py
scp redpitaya_pid_3.15.bit run_pulse_control.py pulse_control.py
cat /root/redpitaya_pid_3.15.bit > /dev/xdevcfg

## 3. Edit `run_pulse_control.py`

Only need to change the configuration block at the top, above `Sections below do not need any changes`. Keep `pulse_control.py` unchanged for routine runs.
For the "measurement" round, only need to change sections for "Trigger","DAC2 outputs","ADC measurement"

## 4. Apply and check outputs

```bash
python3 run_pulse_control.py apply
python3 run_pulse_control.py show
```

## 5. Measurement only (open loop)

```bash
python3 run_pulse_control.py sample --duration-s 10 --interval-s 0.1 --csv measurement.csv
```
Check the reference window and select the pulse you want to use for measurement here:
The setup for this check is:
- photodiode output split to **Red Pitaya `IN1`** and the **oscilloscope**;
- `DIO2_P` also connected to the **oscilloscope**;
Then start measurement:

```bash
python3 run_pulse_control.py sample --duration-s 1000 --interval-s 1 --csv measurement.csv
```
Observe something like this：
![sample window](image.png)


## 6. Calibrate the area-to-DAC gain

Keep feedback OFF. Take a measurement-only at one DAC2 HIGH setting, then change **only** `DAC2_HIGH_VOLTAGE_V`, run `apply` again and repeat under the same optical/trigger conditions. Example:

```text
AREA_TO_DAC_GAIN_V_PER_V_US = abs((V2 - V1) / (A2 - A1))
```

## 7. Feedback (closed loop)

Stop the open-loop session first, then edit the feedback section:

```python
FEEDBACK_SOURCE = "selected"
TARGET_FEEDBACK_AREA_V_US = xxx

AREA_ERROR_DEADBAND_FRACTION = 0.005

AREA_TO_DAC_GAIN_V_PER_V_US = xxx
```

And

```bash
python3 run_pulse_control.py sample --duration-s 10 --interval-s 0.1 --csv feedback.csv --feedback
```

## 8. Stop / reset

```bash
python3 run_pulse_control.py feedback-off
python3 run_pulse_control.py measurement-off
python3 run_pulse_control.py outputs-off
```

`measurement-off` also resets the active reference window/state; `outputs-off` disables GPIO and DAC2. 
**Do not use `clear-state` casually while feedback is running:** that command switches feedback OFF and clears the controller state. Re-run `apply` after changing parameters.


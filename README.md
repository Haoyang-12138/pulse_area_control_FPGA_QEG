# Pulse Area Control on Red Pitaya FPGA — QEG
A real-time optical pulse-area measurement and feedback-control system developed for the QEG experimental platform.
This project provides two complementary approaches for pulse-area measurement and feedback control:
1. **Host-computer control using a Jupyter notebook** — pulse measurements are acquired from any experimental hardware, while pulse-area calculation and PID feedback are performed on the host computer: compensate for slow drift on the scale of seconds.
2. **FPGA-based control using a Red Pitaya** — pulse generation, measurement, and PID feedback are directly in the FPGA for lower-latency closed-loop control: latency 16 clock cycles (108 ns)

## Repository Structure

```text
pulse_area_control_FPGA_QEG/
├── redpitaya_3.0.ipynb
│   └── Host-computer pulse measurement and PID-control workflow
│
├── pulse_control_pid_v3.11/
│   ├── *.sv
│   ├── pulse_control_cdc_v3.xdc
│   ├── pulse_control.py
│   └── README_FPGA_Pulse_Control_v3_13.md
│       └── FPGA source, control software, and detailed documentation
│
└── README.md
```

---

# 1. Host-Computer / Jupyter Notebook Control

`redpitaya_3.0.ipynb` provides a flexible workflow for pulse measurement and feedback control, with the control calculations performed on the host computer.

## Main Features

### Hardware-independent structure

The notebook separates:

* a **general data input/output and feedback-control section**, and
* a **Red Pitaya-specific control section**.

This allows the general measurement and feedback framework to be adapted to different experimental hardware.

Possible input/output devices include:

* oscilloscopes;
* Red Pitaya;
* AWGs;
* RFSoCs;
* other ADC/DAC hardware.

The Red Pitaya therefore does not have to be used as both the ADC and DAC.

### Independent pulse generation, measurement, and feedback

The notebook allows the following functions to be operated separately:

* pulse/output generation;
* pulse measurement;
* feedback control.

This is useful for debugging individual parts of the experimental system before running the complete closed loop.

### Runtime parameter control

Important experimental and feedback parameters can be changed directly from the notebook, including:

* target pulse area;
* target/output amplitude or Vpp;
* `Kp`;
* `Ki`;
* `Kd`;
* pulse-detection parameters;
* acquisition parameters;
* feedback settings.

### Measurement and analysis

The notebook can be used to:

* acquire pulse waveforms;
* calculate pulse area;
* monitor pulse-to-pulse fluctuations;
* compare open-loop and closed-loop behaviour;
* record experimental data;
* generate diagnostic plots.

## Conceptual Workflow

```text
ADC / oscilloscope / Red Pitaya / RFSoC
                │
                ▼
        acquire pulse waveform
                │
                ▼
       calculate pulse area
                │
                ▼
       calculate area error
                │
                ▼
       host-computer PID
                │
                ▼
      DAC / AWG / control output
                │
                └──────────────► next pulse
```

This implementation is particularly useful for development, debugging, hardware comparison, and experiments where the required feedback bandwidth does not require the complete control loop to run inside the FPGA.

---

# 2. Red Pitaya FPGA Pulse-Area Control

The `pulse_control_pid_v3.11/` directory contains the FPGA implementation together with the corresponding Python control interface.

The purpose of this implementation is to move the time-critical parts of the feedback loop directly into the FPGA.

## Conceptual Workflow

```text
external trigger / pulse sequence
              │
              ▼
        ADC pulse detection
              │
              ▼
     pulse-area measurement
              │
              ▼
       pulse-area error
              │
              ▼
         PID controller
              │
              ▼
     corrected DAC output
              │
              └──────────────► experiment
```

## Main Features

### Runtime register map

Important parameters can be changed through FPGA registers without regenerating the bit file.

Configurable quantities include:

* target pulse area;
* output amplitude / Vpp;
* `Kp`;
* `Ki`;
* `Kd`;
* pulse-detection thresholds;
* pulse-validity limits;
* pulse/group timing;
* DAC waveform parameters;
* external-trigger delay;
* group source;
* measurement enable/disable;
* feedback enable/disable.

This makes it possible to tune the experiment while keeping the same FPGA bitstream loaded.

### External trigger

The FPGA supports an external trigger for synchronising pulse grouping and measurement with the experimental sequence.

The trigger path includes synchronisation into the FPGA clock domain and a programmable trigger delay.

This allows the measurement window to be aligned with the actual optical pulse arrival time.

### Pulse generation, measurement, and feedback in one bitstream

The same FPGA design supports:

* pulse/waveform generation;
* pulse detection;
* pulse-area measurement;
* pulse grouping;
* PID feedback;
* corrected DAC output.

These functions can also be enabled or tested separately when required.

### Low-latency FPGA feedback

The pulse-area measurement and PID calculation are performed directly in FPGA logic.

This removes the need to transfer every pulse measurement to the host computer before calculating the correction, allowing substantially lower feedback latency than the notebook-based implementation.

### Python runtime interface

The accompanying Python software is used to control and monitor the FPGA.

It can be used to:

* read and write FPGA registers;
* change target area and PID parameters;
* enable or disable measurement;
* enable or disable feedback;
* configure the DAC waveform;
* configure the external trigger;
* inspect PID state;
* inspect pulse measurements;
* monitor FIFO and diagnostic information;
* record measurements;
* save data to CSV;
* generate diagnostic plots.

Plotting and data analysis are therefore performed on the Python side, while the time-critical measurement and feedback operations remain in the FPGA.

---

# FPGA Source Modules

The FPGA implementation contains modules responsible for:

* pulse detection;
* pulse-area measurement;
* pulse grouping and aggregation;
* PID feedback calculation;
* DAC waveform generation;
* GPIO sequence generation;
* external-trigger synchronisation;
* programmable trigger delay;
* clock-domain crossing;
* asynchronous FIFOs;
* runtime control/status registers;
* debug and latency markers.

Detailed implementation information is available in:

[`pulse_control_pid_v3.11/README_FPGA_Pulse_Control_v3_13.md`](pulse_control_pid_v3.11/README_FPGA_Pulse_Control_v3_13.md)

This includes the FPGA architecture, register-map details, version history, implementation notes, and hardware regression procedure.

---

# Two Control Approaches

The two implementations are intended to complement each other.

|                        | Jupyter / Host Computer          | Red Pitaya FPGA               |
| ---------------------- | -------------------------------- | ----------------------------- |
| Pulse acquisition      | External hardware / Red Pitaya   | Red Pitaya ADC                |
| Pulse-area calculation | Host computer                    | FPGA                          |
| PID calculation        | Host computer                    | FPGA                          |
| Output hardware        | Flexible                         | Red Pitaya DAC                |
| Hardware flexibility   | High                             | Red Pitaya-specific           |
| Feedback latency       | Higher                           | Lower                         |
| Parameter tuning       | Runtime                          | Runtime register map          |
| Plotting / logging     | Jupyter                          | Python interface              |
| Main purpose           | Development and flexible testing | Real-time closed-loop control |

The host-computer implementation is useful for developing and testing pulse-area feedback algorithms with flexible hardware.

Once the measurement and control strategy has been established, the FPGA implementation allows the same basic feedback concept to operate with much lower latency.

---

# Development Status

This repository contains active experimental research code.

The FPGA source package includes work derived from a physically tested **v3.12** baseline together with later **v3.13** modifications, including:

* external-trigger support;
* programmable external-trigger delay;
* revised `Ki` implementation.

The current v3.13 source should be treated as a candidate version until the corresponding Vivado timing checks and hardware regression tests have been completed.

For version-specific verification status, refer to the detailed FPGA README.

---

# Intended Application

The system was developed to investigate and stabilise optical control pulses by directly measuring pulse area and correcting the control waveform in closed loop.

The overall concept is:

```text
generate pulse
     │
     ▼
optical system
     │
     ▼
photodetector
     │
     ▼
measure pulse area
     │
     ▼
compare with target
     │
     ▼
PID correction
     │
     ▼
update next pulse
```

This architecture allows pulse stability and control-system imperfections to be characterised first using a flexible host-computer implementation and subsequently corrected using a low-latency FPGA feedback loop.

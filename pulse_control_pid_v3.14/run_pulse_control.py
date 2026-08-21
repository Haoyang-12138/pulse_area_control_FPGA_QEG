#!/usr/bin/env python3
"""
PC07 dual reference-source runtime.

Pins are unchanged:
  DIO0_P = qualified external trigger INPUT
  DIO1_P = programmable GPIO OUTPUT
  DIO2_P = reference_window_active OUTPUT marker

REFERENCE_SOURCE / GROUP_SOURCE may be:
  "external" : qualified + delayed DIO0_P starts each reference window.
  "dac2"     : each DAC2 period boundary starts each reference window.
  "manual"   : debug/idle only.

Examples:
  python3 run_pulse_control.py apply
  python3 run_pulse_control.py show
  python3 run_pulse_control.py measurement-on
  python3 run_pulse_control.py snapshot-once
  python3 run_pulse_control.py sample --duration-s 10 --interval-s 0.1 --csv refs.csv
  python3 run_pulse_control.py sample --duration-s 10 --interval-s 0.1 --csv refs.csv --feedback
"""

import argparse
import csv
import sys
import time

from pulse_control import PulseControl


ADC_VOLTS_PER_COUNT = 1.0 / 8192.0
DAC_COUNTS_PER_VOLT = 8192.0

# ---------------------------------------------------------------------------
# 1. What starts each PID reference window.
# ---------------------------------------------------------------------------
# "external": qualified + delayed DIO0_P event.
# "dac2":     DAC2 period boundary. External min-HIGH/delay are bypassed.
# "manual":   debug/idle only; sparse automatic sampling is not intended for it.
GROUP_SOURCE = "external"

# External-source settings below are used only when GROUP_SOURCE == "external".
EXTERNAL_TRIGGER_FREQUENCY_HZ = 1000000.0
# DIO0 long-pulse qualifier: one point = 5 ns at 200 MHz.
EXTERNAL_TRIGGER_MIN_HIGH_POINTS = 40
# Delay is applied AFTER the long-trigger qualification, 5 ns resolution.
# Calibrate this using scope: DIO0 + DIO2 + photodiode.
EXTERNAL_TRIGGER_DELAY_NS = 650.0

# ---------------------------------------------------------------------------
# 2. Which measured optical pulses are used for PID.
# ---------------------------------------------------------------------------
# Pulse indices are relative to DIO2/reference start and start at 0.
# Example START=2, COUNT=3 means pulse 2 + pulse 3 + pulse 4 are summed.
REFERENCE_TARGET_START_INDEX = 0
REFERENCE_TARGET_PULSE_COUNT = 14

# ---------------------------------------------------------------------------
# 3. Existing programmable GPIO on DIO1_P, 200 MHz / 5 ns resolution.
# ---------------------------------------------------------------------------
GPIO_ENABLED = True
GPIO_FREQUENCY_HZ = 1000000.0
GPIO_PULSES = [
    (0.000, 0.010),
]

# ---------------------------------------------------------------------------
# 4. OUT2 DAC2 sequence, now 1..32 segments, 125 MHz / 8 ns resolution.
# ---------------------------------------------------------------------------
DAC2_ENABLED = True
DAC2_FREQUENCY_HZ = 25000.0
DAC2_LOW_VOLTAGE_V = 0.0
DAC2_HIGH_VOLTAGE_V = 0.500

# Each pair is (normalized level in [0,1], fraction of one DAC2 period).
# Up to 32 entries; fractions must sum to 1.0.
DAC2_SEQUENCE = [
    (1.0, 10.0 / 40.0),     # 10 us UP
    (0.0,  5.0 / 40.0),     # 5 us DOWN

    (1.0, 0.25 / 40.0),     # 0.25 us UP
    (0.0, 0.25 / 40.0),     # 0.25 us DOWN
    (1.0, 0.25 / 40.0),
    (0.0, 0.25 / 40.0),
    (1.0, 0.25 / 40.0),
    (0.0, 0.25 / 40.0),
    (1.0, 0.25 / 40.0),
    (0.0, 0.25 / 40.0),
    (1.0, 0.25 / 40.0),
    (0.0, 0.25 / 40.0),   
    (1.0, 0.25 / 40.0),     
    (0.0, 0.25 / 40.0),     
    (1.0, 0.25 / 40.0),
    (0.0, 0.25 / 40.0),
    (1.0, 0.25 / 40.0),
    (0.0, 0.25 / 40.0),
    (1.0, 0.25 / 40.0),
    (0.0, 0.25 / 40.0),
    (1.0, 0.25 / 40.0),
    (0.0, 0.25 / 40.0),   

    (0.0,  5.0 / 40.0),     # 5 us DOWN
    (1.0,  1.0 / 40.0),     
    (0.0,  4.0 / 40.0),
    (1.0,  1.0 / 40.0),     
    (0.0,  4.0 / 40.0),
    (1.0,  1.0 / 40.0),     
    (0.0,  4.0 / 40.0),
]

# ---------------------------------------------------------------------------
# 5. ADC IN1 pulse measurement.
# ---------------------------------------------------------------------------
TRIGGER_FRACTION = 0.25
TARGET_HEIGHT_V = 0.100
PRE_BASELINE_SAMPLES = 16
POST_BASELINE_SAMPLES = 16
MIN_PULSE_SAMPLES = 3
MAX_PULSE_SAMPLES = 16384
ADC_SATURATION_LIMIT_V = 0.95
PULSE_POLARITY = "positive"

# Legacy full trigger-to-trigger group diagnostics only; PID no longer uses the
# old group mean. Leave these permissive unless you specifically want warnings.
EXPECTED_PULSES_PER_GROUP = 0
MIN_VALID_PULSES_PER_GROUP = 1

# ---------------------------------------------------------------------------
# 6. PID. Input is REFERENCE SUM, not old group mean.
# ---------------------------------------------------------------------------
FEEDBACK_ENABLED = False

TARGET_REFERENCE_SUM_V_US = None
USE_FIRST_VALID_REFERENCE_AS_TARGET = True

AREA_ERROR_DEADBAND_V_US = None
AREA_ERROR_DEADBAND_FRACTION = 0.005

KP = 0.20
KI_PER_S = 0.0
KD = 0.0

# Keep None until calibrated; --feedback/FEEDBACK_ENABLED requires this.
AREA_TO_DAC_GAIN_V_PER_V_US = None
DAC_CORRECTION_SIGN = -1.0
MAX_DAC_STEP_PER_UPDATE_V = 0.020
DAC_HIGH_VOLTAGE_MIN_V = DAC2_LOW_VOLTAGE_V + 0.010
DAC_HIGH_VOLTAGE_MAX_V = 1.000
MAX_INTEGRAL_AREA_TERM_FRACTION = 0.25

# PC06 physical DIO2 ignores this legacy marker bit and directly follows
# reference_window_active. Keep this only for the old internal debug marker.
LEGACY_DEBUG_MARKER_ENABLED = False

# Sparse logging defaults. One CSV row = one complete PID reference snapshot.
CSV_FILENAME = "pc06_reference_samples.csv"
SNAPSHOT_TIMEOUT_S = 0.2


# ===========================================================================
# Helpers
# ===========================================================================

def open_control():
    return PulseControl(
        adc_volts_per_count=ADC_VOLTS_PER_COUNT,
        dac_counts_per_volt=DAC_COUNTS_PER_VOLT,
    )


def apply_configuration(control):
    if GROUP_SOURCE not in ("external", "dac2", "manual"):
        raise ValueError("GROUP_SOURCE must be 'external', 'dac2', or 'manual'")

    if GROUP_SOURCE == "external":
        if EXTERNAL_TRIGGER_FREQUENCY_HZ <= 0.0:
            raise ValueError("EXTERNAL_TRIGGER_FREQUENCY_HZ must be positive")
    elif GROUP_SOURCE == "dac2":
        if not DAC2_ENABLED:
            raise RuntimeError(
                "GROUP_SOURCE='dac2' requires DAC2_ENABLED=True because "
                "the reference trigger is the physical DAC2 period boundary"
            )
        if DAC2_FREQUENCY_HZ <= 0.0:
            raise ValueError("DAC2_FREQUENCY_HZ must be positive")

    # Safe configuration state: source is manual while registers/tables settle;
    # feedback/measurement/logging are all stopped.
    control.enable_feedback(False)
    control.enable_measurement(False)
    control.enable_continuous_fifo_logging(False)
    control.set_group_source("manual")
    control.set_outputs(gpio_enabled=False, dac2_enabled=False)

    gpio_info = None
    if GPIO_ENABLED:
        gpio_info = control.configure_gpio(
            GPIO_FREQUENCY_HZ,
            GPIO_PULSES,
            enable=False,
            apply=True,
        )

    dac2_info = control.configure_dac2(
        DAC2_FREQUENCY_HZ,
        DAC2_LOW_VOLTAGE_V,
        DAC2_HIGH_VOLTAGE_V,
        DAC2_SEQUENCE,
        enable=False,
        apply=True,
    )

    measurement_info = control.configure_measurement(
        trigger_fraction=TRIGGER_FRACTION,
        target_height_v=TARGET_HEIGHT_V,
        pre_baseline_samples=PRE_BASELINE_SAMPLES,
        post_baseline_samples=POST_BASELINE_SAMPLES,
        min_pulse_samples=MIN_PULSE_SAMPLES,
        max_pulse_samples=MAX_PULSE_SAMPLES,
        adc_saturation_limit_v=ADC_SATURATION_LIMIT_V,
        pulse_polarity=PULSE_POLARITY,
        expected_pulses_per_group=EXPECTED_PULSES_PER_GROUP,
        min_valid_pulses_per_group=MIN_VALID_PULSES_PER_GROUP,
        group_source=GROUP_SOURCE,
        external_trigger_delay_ns=EXTERNAL_TRIGGER_DELAY_NS,
        external_trigger_min_high_points=EXTERNAL_TRIGGER_MIN_HIGH_POINTS,
        external_trigger_frequency_hz=(
            EXTERNAL_TRIGGER_FREQUENCY_HZ if GROUP_SOURCE == "external" else None
        ),
        reference_target_start_index=REFERENCE_TARGET_START_INDEX,
        reference_target_pulse_count=REFERENCE_TARGET_PULSE_COUNT,
        enable=False,
    )

    feedback_info = None
    if AREA_TO_DAC_GAIN_V_PER_V_US is not None:
        feedback_info = control.configure_feedback(
            target_reference_sum_v_us=TARGET_REFERENCE_SUM_V_US,
            use_first_valid_reference_as_target=USE_FIRST_VALID_REFERENCE_AS_TARGET,
            area_error_deadband_v_us=AREA_ERROR_DEADBAND_V_US,
            area_error_deadband_fraction=AREA_ERROR_DEADBAND_FRACTION,
            kp=KP,
            ki_per_s=KI_PER_S,
            kd=KD,
            area_to_dac_gain_v_per_v_us=AREA_TO_DAC_GAIN_V_PER_V_US,
            dac_correction_sign=DAC_CORRECTION_SIGN,
            max_dac_step_per_update_v=MAX_DAC_STEP_PER_UPDATE_V,
            dac_high_voltage_min_v=DAC_HIGH_VOLTAGE_MIN_V,
            dac_high_voltage_max_v=DAC_HIGH_VOLTAGE_MAX_V,
            max_integral_area_term_fraction=MAX_INTEGRAL_AREA_TERM_FRACTION,
            # External source has no hardware period register, so Python
            # supplies dt. DAC2 source is inferred by pulse_control.py directly
            # from the active DAC2 period register.
            controller_dt_s=(
                (1.0 / EXTERNAL_TRIGGER_FREQUENCY_HZ)
                if GROUP_SOURCE == "external"
                else None
            ),
            enable=False,
        )
    elif FEEDBACK_ENABLED:
        raise RuntimeError(
            "FEEDBACK_ENABLED=True but AREA_TO_DAC_GAIN_V_PER_V_US is None"
        )

    control.enable_marker(LEGACY_DEBUG_MARKER_ENABLED)
    control.clear_flags()
    control.clear_fifos()
    control.clear_state()

    # Safe idle: outputs can run, but external reference trigger remains disabled
    # because measurement=OFF and group source=manual.
    control.enable_measurement(False)
    control.enable_feedback(False)
    control.enable_continuous_fifo_logging(False)
    control.set_group_source("manual")

    control.set_outputs(
        gpio_enabled=GPIO_ENABLED,
        dac2_enabled=DAC2_ENABLED,
    )

    control.sanity_check(
        require_measurement=False,
        require_gpio=GPIO_ENABLED,
        require_dac2=DAC2_ENABLED,
        require_feedback=False,
    )

    return gpio_info, dac2_info, measurement_info, feedback_info


def print_status(control):
    snap = control.read_snapshot()
    status = snap["status"]
    ref_cfg = control.reference_config()
    print("PC07 status")
    print("-----------")
    print("pin DIO0_P:                  external trigger INPUT")
    print("pin DIO1_P:                  programmable GPIO OUTPUT")
    print("pin DIO2_P:                  reference_window_active OUTPUT")
    print("group/reference source:      {0}".format(control.get_group_source()))
    print("continuous FIFO logging:     {0}".format(int(control.continuous_fifo_logging_enabled())))
    print("measurement enabled/config:  {0}/{1}".format(
        int(status["measurement_enabled"]), int(status["measurement_config_valid"])))
    print("GPIO enabled/config:         {0}/{1}".format(
        int(status["gpio_enabled"]), int(status["gpio_config_valid"])))
    print("DAC2 enabled/config:         {0}/{1}".format(
        int(status["dac2_enabled"]), int(status["dac2_config_valid"])))
    print("feedback enabled/config:     {0}/{1}".format(
        int(status["feedback_enabled"]), int(status["controller_config_valid"])))
    print("target latched:              {0}".format(int(status["target_latched"])))
    print("DIO2/reference window active:{0}".format(int(status["reference_window_active"])))
    if control.get_group_source() == "external":
        print("external trigger accepted:   {0}".format(snap["external_trigger_count"]))
        print("external delay active:       {0} cycles = {1:.1f} ns".format(
            snap["external_trigger_active_delay_cycles"],
            5.0 * snap["external_trigger_active_delay_cycles"]))
        print("external min-HIGH active:    {0} points ~= {1:.1f} ns".format(
            snap["external_trigger_active_min_high_points"],
            5.0 * snap["external_trigger_active_min_high_points"]))
    elif control.get_group_source() == "dac2":
        dac_period_cycles = control.regs.read_u32(0x37C)
        dac_frequency_hz = (
            125000000.0 / float(dac_period_cycles)
            if dac_period_cycles else 0.0
        )
        print("reference trigger:           DAC2 period boundary")
        print("DAC2 active period:          {0} cycles = {1:.3f} us ({2:.6g} Hz)".format(
            dac_period_cycles,
            0.008 * float(dac_period_cycles),
            dac_frequency_hz))
        print("external delay/min-HIGH:     bypassed in DAC2 source mode")
    print("reference target indices:    {0} .. {1}  (count={2})".format(
        ref_cfg["target_start_index"],
        ref_cfg["target_start_index"] + ref_cfg["target_pulse_count"] - 1,
        ref_cfg["target_pulse_count"]))
    print("last reference id/sum:       {0} / {1} count*sample ({2:.6g} V*us)".format(
        snap["last_reference_id"],
        snap["last_reference_sum_counts_samples"],
        control.area_v_us_from_counts(snap["last_reference_sum_counts_samples"])))
    print("last reference count/valid:  {0} / {1}".format(
        snap["last_reference_selected_count"], int(snap["last_reference_valid"])))
    print("valid / invalid pulses:      {0} / {1}".format(
        snap["valid_count"], snap["invalid_count"]))
    print("target reference sum:        {0} count*sample ({1:.6g} V*us)".format(
        snap["target_area_counts_samples"],
        control.area_v_us_from_counts(snap["target_area_counts_samples"])))
    print("live error / P / I / D:      {0} / {1} / {2} / {3}".format(
        snap["area_error_counts_samples"], snap["p_area"], snap["i_area"], snap["d_area"]))
    print("correction pending/active:   {0} / {1} DAC counts".format(
        snap["correction_pending_counts"], snap["correction_active_counts"]))
    print("effective OUT2 HIGH:         {0} counts".format(snap["effective_high_counts"]))
    print("PID processing latency:      {0} cycles".format(snap["processing_latency_cycles"]))
    print("pulse/group FIFO level:      {0} / {1}".format(
        snap["pulse_fifo_level"], snap["group_fifo_level"]))
    print("pulse/group FIFO overflow:   {0} / {1}".format(
        snap["pulse_fifo_overflow"], snap["group_fifo_overflow"]))
    print("trigger overrun seen:        {0}".format(int(status["external_trigger_overrun_seen"])))
    print("reference incomplete seen:   {0}".format(int(status["reference_incomplete_seen"])))
    print("reference sum overflow seen: {0}".format(int(status["reference_sum_overflow_seen"])))


def monitor(control, interval_s, count):
    print("time_s trig ref_id ref_sum ref_valid win target error P I D corr fifo reason")
    start = time.monotonic()
    n = 0
    while count <= 0 or n < count:
        snap = control.read_snapshot()
        print(
            "{0:8.3f} {1:8d} {2:8d} {3:10d} {4:3d} {5:3d} {6:10d} {7:10d} "
            "{8:8d} {9:8d} {10:8d} {11:8d} {12:4d} {13}".format(
                time.monotonic() - start,
                int(snap["external_trigger_count"]),
                int(snap["last_reference_id"]),
                int(snap["last_reference_sum_counts_samples"]),
                int(snap["last_reference_valid"]),
                int(snap["reference_window_active"]),
                int(snap["target_area_counts_samples"]),
                int(snap["area_error_counts_samples"]),
                int(snap["p_area"]),
                int(snap["i_area"]),
                int(snap["d_area"]),
                int(snap["correction_active_counts"]),
                int(snap["pulse_fifo_level"]),
                snap["last_invalid_reason"],
            )
        )
        n += 1
        time.sleep(interval_s)


def reference_csv_fields(target_count):
    fields = [
        "sample_index",
        "host_elapsed_s",
        "snapshot_complete",
        "reference_id",
        "reference_valid",
        "reference_sum_area_v_us",
        "target_reference_sum_v_us",
        "area_error_measured_minus_target_v_us",
        "normalized_area_error",
        "observed_target_pulse_count",
    ]
    for j in range(int(target_count)):
        prefix = "target{0}_".format(j)
        fields.extend([
            prefix + "pulse_id",
            prefix + "fpga_time_s",
            prefix + "valid",
            prefix + "invalid_reason",
            prefix + "pulse_duration_s",
            prefix + "baseline_pre_v",
            prefix + "baseline_post_v",
            prefix + "baseline_for_area_v",
            prefix + "pulse_height_v",
            prefix + "peak_height_v",
            prefix + "pulse_area_v_us",
            prefix + "correction_active_v",
            prefix + "effective_high_v",
        ])
    return fields


def snapshot_to_row(control, result, sample_index, host_elapsed_s):
    row = {
        "sample_index": sample_index,
        "host_elapsed_s": host_elapsed_s,
        "snapshot_complete": int(bool(result["complete"] and result["same_reference"] and result["pulse_ids_match"])),
        "reference_id": "" if result["reference_id"] is None else result["reference_id"],
        "reference_valid": int(bool(result["reference_valid"])),
        "reference_sum_area_v_us": result["reference_sum_v_us"],
        "observed_target_pulse_count": result["observed_count"],
    }

    # Target is a stable controller state after auto-target latches. Unlike old
    # PC05 CSV, we do NOT pretend live P/I/D reads are coherent with the sparse
    # snapshot. The captured pulse records themselves are the exact PID inputs.
    snap = control.read_snapshot()
    target_counts = snap["target_area_counts_samples"]
    target_v_us = control.area_v_us_from_counts(target_counts)
    row["target_reference_sum_v_us"] = target_v_us
    if target_counts != 0 and result["complete"]:
        error_v_us = result["reference_sum_v_us"] - target_v_us
        row["area_error_measured_minus_target_v_us"] = error_v_us
        row["normalized_area_error"] = error_v_us / target_v_us
    else:
        row["area_error_measured_minus_target_v_us"] = ""
        row["normalized_area_error"] = ""

    for j, pulse in enumerate(result["pulses"]):
        prefix = "target{0}_".format(j)
        row[prefix + "pulse_id"] = pulse["pulse_id"]
        row[prefix + "fpga_time_s"] = pulse["time_s"]
        row[prefix + "valid"] = int(bool(pulse["valid"]))
        row[prefix + "invalid_reason"] = pulse["invalid_reason"]
        row[prefix + "pulse_duration_s"] = pulse["duration_s"]
        row[prefix + "baseline_pre_v"] = pulse["bpre_v"]
        row[prefix + "baseline_post_v"] = pulse["bpost_v"]
        row[prefix + "baseline_for_area_v"] = pulse["baseline_for_area_v"]
        row[prefix + "pulse_height_v"] = "" if pulse["pulse_height_v"] is None else pulse["pulse_height_v"]
        row[prefix + "peak_height_v"] = pulse["peak_height_v"]
        row[prefix + "pulse_area_v_us"] = pulse["area_v_us"]
        row[prefix + "correction_active_v"] = pulse["correction_active_v"]
        row[prefix + "effective_high_v"] = pulse["effective_high_v"]
    return row


def sample_reference_csv(control, csv_path, duration_s, interval_s, feedback=False, timeout_s=SNAPSHOT_TIMEOUT_S):
    duration_s = float(duration_s)
    interval_s = float(interval_s)
    timeout_s = float(timeout_s)
    if duration_s <= 0.0:
        raise ValueError("--duration-s must be positive")
    if interval_s <= 0.0:
        raise ValueError("--interval-s must be positive")
    if timeout_s <= 0.0:
        raise ValueError("--timeout-s must be positive")
    if GROUP_SOURCE not in ("external", "dac2"):
        raise RuntimeError(
            "Sparse reference sampling requires GROUP_SOURCE='external' or 'dac2'"
        )
    if GROUP_SOURCE == "dac2" and not DAC2_ENABLED:
        raise RuntimeError("GROUP_SOURCE='dac2' requires DAC2_ENABLED=True")

    target_count = int(control.reference_config()["target_pulse_count"])
    fields = reference_csv_fields(target_count)

    control.enable_feedback(False)
    control.enable_measurement(False)
    control.enable_continuous_fifo_logging(False)
    control.set_group_source("manual")
    control.clear_fifos()
    control.clear_flags()
    control.clear_state()

    start = time.monotonic()
    next_sample = start
    sample_index = 0

    with open(csv_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        handle.flush()

        # Prime rolling baseline before allowing DIO0 to create references.
        control.enable_measurement(True)
        time.sleep(0.001)
        control.sanity_check(
            require_measurement=True,
            require_dac2=(GROUP_SOURCE == "dac2" or feedback),
            require_feedback=feedback,
        )
        control.clear_fifos()
        # Arm the source selected at the top of this file.
        # IMPORTANT: do not hard-code "external" here; in DAC2 mode the
        # reference_start event must come from dac2_period_boundary.
        control.set_group_source(GROUP_SOURCE)
        if control.get_group_source() != GROUP_SOURCE:
            raise RuntimeError(
                "Failed to select reference source: requested {0}, read back {1}".format(
                    GROUP_SOURCE, control.get_group_source()
                )
            )
        if feedback:
            if AREA_TO_DAC_GAIN_V_PER_V_US is None:
                raise RuntimeError("--feedback requires AREA_TO_DAC_GAIN_V_PER_V_US")
            control.enable_feedback(True)

        print(
            "PC07 sparse reference sampling: source={0}, start_index={1}, "
            "count={2}, interval={3:g}s -> {4}".format(
                GROUP_SOURCE,
                REFERENCE_TARGET_START_INDEX,
                REFERENCE_TARGET_PULSE_COUNT,
                interval_s,
                csv_path,
            )
        )

        try:
            while True:
                now = time.monotonic()
                if now - start >= duration_s:
                    break
                if now < next_sample:
                    time.sleep(min(next_sample - now, 0.02))
                    continue

                result = control.read_selected_reference_snapshot(timeout_s=timeout_s)
                elapsed = time.monotonic() - start
                row = snapshot_to_row(control, result, sample_index, elapsed)
                writer.writerow(row)
                handle.flush()

                print(
                    "{0:4d} t={1:8.3f}s ref={2} complete={3} valid={4} sum={5:.7g} V*us ids={6}".format(
                        sample_index,
                        elapsed,
                        row.get("reference_id", ""),
                        row.get("snapshot_complete", 0),
                        row.get("reference_valid", 0),
                        float(result["reference_sum_v_us"]),
                        result["observed_pulse_ids"],
                    )
                )

                sample_index += 1
                next_sample = start + sample_index * interval_s
        finally:
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.enable_continuous_fifo_logging(False)
            control.set_group_source("manual")
            control.clear_fifos()

    return {"csv": csv_path, "samples": sample_index, "duration_s": time.monotonic() - start}


def start_measurement_session(control, feedback=False):
    control.enable_feedback(False)
    control.enable_measurement(False)
    control.enable_continuous_fifo_logging(False)
    control.set_group_source("manual")
    control.clear_fifos()
    control.clear_flags()
    control.clear_state()
    control.enable_measurement(True)
    time.sleep(0.001)
    control.set_group_source(GROUP_SOURCE)
    control.sanity_check(
        require_measurement=True,
        require_dac2=(GROUP_SOURCE == "dac2" or feedback),
        require_feedback=feedback,
    )
    if feedback:
        control.enable_feedback(True)


def build_parser():
    parser = argparse.ArgumentParser(
        description="PC07 external-or-DAC2 reference-source / reference-sum pulse-control runtime"
    )
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("apply", help="apply all edited CONFIGURATION settings")
    sub.add_parser("show", help="show PC07 live status")
    sub.add_parser("outputs-on")
    sub.add_parser("outputs-off")
    sub.add_parser("measurement-on")
    sub.add_parser("measurement-off")
    sub.add_parser("feedback-on")
    sub.add_parser("feedback-off")
    sub.add_parser("clear-state")
    sub.add_parser("clear-flags")
    sub.add_parser("clear-fifos")

    dc = sub.add_parser("dac2-dc", help="physical OUT2 one-segment DC regression")
    dc.add_argument("--counts", type=int, required=True)

    sq = sub.add_parser("dac2-square", help="physical OUT2 square-wave regression")
    sq.add_argument("--low-counts", type=int, default=0)
    sq.add_argument("--high-counts", type=int, required=True)
    sq.add_argument("--frequency", type=float, default=100.0)
    sq.add_argument("--duty", type=float, default=0.5)

    mon = sub.add_parser("monitor")
    mon.add_argument("--interval", type=float, default=0.2)
    mon.add_argument("--count", type=int, default=0)

    once = sub.add_parser("snapshot-once", help="capture the next complete selected PID reference")
    once.add_argument("--timeout-s", type=float, default=SNAPSHOT_TIMEOUT_S)

    sample = sub.add_parser("sample", help="one complete selected PID reference every interval")
    sample.add_argument("--duration-s", type=float, required=True)
    sample.add_argument("--interval-s", type=float, default=0.1)
    sample.add_argument("--timeout-s", type=float, default=SNAPSHOT_TIMEOUT_S)
    sample.add_argument("--csv", default=CSV_FILENAME)
    sample.add_argument("--feedback", action="store_true")

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    if args.command is None:
        parser.print_help()
        return 1

    try:
        control = open_control()
    except PermissionError:
        print("Permission denied opening /dev/mem. Run as root.", file=sys.stderr)
        return 1
    except Exception as error:
        print("Could not open PC07 register map: {0}".format(error), file=sys.stderr)
        return 1

    try:
        if args.command == "apply":
            gpio_info, dac2_info, measurement_info, feedback_info = apply_configuration(control)
            print("GPIO configuration: {0}".format(gpio_info))
            print("DAC2 configuration: {0}".format(dac2_info))
            print("Measurement/reference configuration: {0}".format(measurement_info))
            print("PID configuration: {0}".format(feedback_info))
            print_status(control)

        elif args.command == "show":
            print_status(control)

        elif args.command == "outputs-on":
            control.sanity_check(require_gpio=GPIO_ENABLED, require_dac2=DAC2_ENABLED)
            control.set_outputs(gpio_enabled=GPIO_ENABLED, dac2_enabled=DAC2_ENABLED)

        elif args.command == "outputs-off":
            control.enable_feedback(False)
            control.set_outputs(gpio_enabled=False, dac2_enabled=False)

        elif args.command == "measurement-on":
            start_measurement_session(control, feedback=False)

        elif args.command == "measurement-off":
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.enable_continuous_fifo_logging(False)
            control.set_group_source("manual")

        elif args.command == "feedback-on":
            if AREA_TO_DAC_GAIN_V_PER_V_US is None:
                raise RuntimeError("feedback-on requires AREA_TO_DAC_GAIN_V_PER_V_US")
            control.sanity_check(require_measurement=True, require_dac2=True, require_feedback=True)
            control.enable_feedback(True)

        elif args.command == "feedback-off":
            control.enable_feedback(False)

        elif args.command == "clear-state":
            control.enable_feedback(False)
            control.clear_state()

        elif args.command == "clear-flags":
            control.clear_flags()

        elif args.command == "clear-fifos":
            control.clear_fifos()

        elif args.command == "dac2-dc":
            if args.counts < -8192 or args.counts > 8191:
                raise ValueError("--counts must be in -8192..8191")
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.set_group_source("manual")
            control.set_outputs(gpio_enabled=False, dac2_enabled=False)
            if args.counts < 8191:
                low_counts = args.counts
                high_counts = args.counts + 1
                sequence = [(0.0, 1.0)]
            else:
                low_counts = 8190
                high_counts = 8191
                sequence = [(1.0, 1.0)]
            info = control.configure_dac2(
                1000.0,
                float(low_counts) / DAC_COUNTS_PER_VOLT,
                float(high_counts) / DAC_COUNTS_PER_VOLT,
                sequence,
                enable=True,
                apply=True,
            )
            print("DAC2 DC test: {0}".format(info))

        elif args.command == "dac2-square":
            if args.low_counts < -8192 or args.low_counts > 8191:
                raise ValueError("--low-counts must be in -8192..8191")
            if args.high_counts < -8192 or args.high_counts > 8191:
                raise ValueError("--high-counts must be in -8192..8191")
            if args.high_counts <= args.low_counts:
                raise ValueError("--high-counts must exceed --low-counts")
            if args.frequency <= 0.0:
                raise ValueError("--frequency must be positive")
            if not 0.0 < args.duty < 1.0:
                raise ValueError("--duty must lie in (0,1)")
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.set_group_source("manual")
            control.set_outputs(gpio_enabled=False, dac2_enabled=False)
            info = control.configure_dac2(
                args.frequency,
                float(args.low_counts) / DAC_COUNTS_PER_VOLT,
                float(args.high_counts) / DAC_COUNTS_PER_VOLT,
                [(1.0, args.duty), (0.0, 1.0 - args.duty)],
                enable=True,
                apply=True,
            )
            print("DAC2 square test: {0}".format(info))

        elif args.command == "monitor":
            if args.interval <= 0.0:
                raise ValueError("--interval must be positive")
            monitor(control, args.interval, args.count)

        elif args.command == "snapshot-once":
            # Do not clear state: this is intended to work during an already-running
            # measurement/feedback session. Ensure sparse FIFO mode only.
            control.enable_continuous_fifo_logging(False)
            result = control.read_selected_reference_snapshot(timeout_s=args.timeout_s)
            print("reference snapshot: {0}".format(result))

        elif args.command == "sample":
            result = sample_reference_csv(
                control,
                csv_path=args.csv,
                duration_s=args.duration_s,
                interval_s=args.interval_s,
                feedback=args.feedback,
                timeout_s=args.timeout_s,
            )
            print("Sampling complete: {0}".format(result))
            print_status(control)

        return 0

    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print("Error: {0}".format(error), file=sys.stderr)
        return 1
    finally:
        control.close()


if __name__ == "__main__":
    sys.exit(main())

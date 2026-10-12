"""dual version
Not very clearly documented yet"""

import argparse
import csv
import sys
import time

from pulse_control import PulseControl316


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

ADC_VOLTS_PER_COUNT_A = 1.0 / 8192.0
ADC_VOLTS_PER_COUNT_B = 1.0 / 8192.0

# These are used to convert DAC counts to volts.
# Calibrate A and B separately if needed
DAC_COUNTS_PER_VOLT_A = 8192.0
DAC_COUNTS_PER_VOLT_B = 8192.0

TRIGGER_FREQUENCY_HZ_A = 1000000.0
TRIGGER_FREQUENCY_HZ_B = 1000000.0

TRIGGER_MIN_HIGH_POINTS_A = 5
TRIGGER_MIN_HIGH_POINTS_B = 5

TRIGGER_DELAY_NS_A = 650.0
TRIGGER_DELAY_NS_B = 650.0

REFERENCE_START_INDEX_A = 0
REFERENCE_START_INDEX_B = 0

REFERENCE_PULSE_COUNT_A = 2
REFERENCE_PULSE_COUNT_B = 2

TRIGGER_FRACTION_A = 0.25
TRIGGER_FRACTION_B = 0.25

TARGET_HEIGHT_V_A = 0.100
TARGET_HEIGHT_V_B = 0.100

PRE_BASELINE_SAMPLES_A = 8
PRE_BASELINE_SAMPLES_B = 8

POST_BASELINE_SAMPLES_A = 8
POST_BASELINE_SAMPLES_B = 8

MIN_PULSE_SAMPLES_A = 3
MIN_PULSE_SAMPLES_B = 3

MAX_PULSE_SAMPLES_A = 16384
MAX_PULSE_SAMPLES_B = 16384

ADC_SATURATION_LIMIT_V_A = 0.95
ADC_SATURATION_LIMIT_V_B = 0.95

PULSE_POLARITY_A = "positive"
PULSE_POLARITY_B = "positive"

DAC_ENABLED_A = True
DAC_ENABLED_B = True

DAC_FREQUENCY_HZ_A = 25000.0
DAC_FREQUENCY_HZ_B = 25000.0

# A is OUT1. B is OUT2.
DAC_LOW_V_A = 0.0
DAC_HIGH_V_A = 0.500

DAC_LOW_V_B = 0.0
DAC_HIGH_V_B = 0.500

DAC_SEQUENCE_A = [
    (0.0, 0.20),
    (1.0, 0.20),
    (0.0, 0.60),
]

DAC_SEQUENCE_B = [
    (0.0, 0.20),
    (1.0, 0.20),
    (0.0, 0.60),
]

# PID
TARGET_AREA_V_US_A = None
TARGET_AREA_V_US_B = None

AUTO_TARGET_A = True
AUTO_TARGET_B = True

DEADBAND_FRACTION_A = 0.005
DEADBAND_FRACTION_B = 0.005

KP_A = 0.20
KP_B = 0.20

KI_PER_S_A = 2.0
KI_PER_S_B = 2.0

KD_A = 0.0
KD_B = 0.0


AREA_TO_DAC_GAIN_V_PER_V_US_A = None
AREA_TO_DAC_GAIN_V_PER_V_US_B = None

CORRECTION_SIGN_A = -1.0
CORRECTION_SIGN_B = -1.0

MAX_DAC_STEP_V_A = 0.020
MAX_DAC_STEP_V_B = 0.020

DAC_HIGH_MIN_V_A = DAC_LOW_V_A + 0.010
DAC_HIGH_MAX_V_A = 1.000

DAC_HIGH_MIN_V_B = DAC_LOW_V_B + 0.010
DAC_HIGH_MAX_V_B = 1.000

INTEGRAL_LIMIT_FRACTION_A = 0.25
INTEGRAL_LIMIT_FRACTION_B = 0.25



# Reference source per channel:
# "external" = DIO3_P for A / DIO0_P for B
# "dac"      = that channel's own DAC period boundary (self-loop/debug)
# "manual"   = debug only
GROUP_SOURCE_A = "external"
GROUP_SOURCE_B = "external"

# PID input source. "selected" matches the selected reference-pulse sum.
# "average" uses the full-group mean if enabled in the firmware.
FEEDBACK_SOURCE_A = "selected"
FEEDBACK_SOURCE_B = "selected"

# Sparse reference logging defaults.
CSV_FILENAME = "reference_samples.csv"
SNAPSHOT_TIMEOUT_S = 0.2

# Raw ADC waveform capture defaults.
WAVEFORM_LENGTH_SAMPLES = 16384
WAVEFORM_PRETRIGGER_SAMPLES = 1024
WAVEFORM_TIMEOUT_S = 1.0
WAVEFORM_CSV_FILENAME = "waveform.csv"







# -----------------------------------------------------------------
# Sections below do not need any changes

def normalize_group_source(value):
    key = str(value).lower()
    if key == "dac2":
        key = "dac"
    if key not in ("external", "dac", "manual"):
        raise ValueError(
            "GROUP_SOURCE must be external, dac, or manual"
        )
    return key


def channel_config(name):
    if name == "A":
        return {
            "adc_volts_per_count": ADC_VOLTS_PER_COUNT_A,
            "dac_counts_per_volt": DAC_COUNTS_PER_VOLT_A,
            "group_source": normalize_group_source(GROUP_SOURCE_A),
            "feedback_source": FEEDBACK_SOURCE_A,
            "trigger_frequency_hz": TRIGGER_FREQUENCY_HZ_A,
            "trigger_min_high_points": TRIGGER_MIN_HIGH_POINTS_A,
            "trigger_delay_ns": TRIGGER_DELAY_NS_A,
            "reference_start_index": REFERENCE_START_INDEX_A,
            "reference_pulse_count": REFERENCE_PULSE_COUNT_A,
            "trigger_fraction": TRIGGER_FRACTION_A,
            "target_height_v": TARGET_HEIGHT_V_A,
            "pre_baseline_samples": PRE_BASELINE_SAMPLES_A,
            "post_baseline_samples": POST_BASELINE_SAMPLES_A,
            "min_pulse_samples": MIN_PULSE_SAMPLES_A,
            "max_pulse_samples": MAX_PULSE_SAMPLES_A,
            "adc_saturation_limit_v": ADC_SATURATION_LIMIT_V_A,
            "pulse_polarity": PULSE_POLARITY_A,
            "dac_enabled": DAC_ENABLED_A,
            "dac_frequency_hz": DAC_FREQUENCY_HZ_A,
            "dac_low_v": DAC_LOW_V_A,
            "dac_high_v": DAC_HIGH_V_A,
            "dac_sequence": DAC_SEQUENCE_A,
            "target_area_v_us": TARGET_AREA_V_US_A,
            "auto_target": AUTO_TARGET_A,
            "deadband_fraction": DEADBAND_FRACTION_A,
            "kp": KP_A,
            "ki_per_s": KI_PER_S_A,
            "kd": KD_A,
            "gain": AREA_TO_DAC_GAIN_V_PER_V_US_A,
            "correction_sign": CORRECTION_SIGN_A,
            "max_dac_step_v": MAX_DAC_STEP_V_A,
            "dac_high_min_v": DAC_HIGH_MIN_V_A,
            "dac_high_max_v": DAC_HIGH_MAX_V_A,
            "integral_limit_fraction": INTEGRAL_LIMIT_FRACTION_A,
        }

    return {
        "adc_volts_per_count": ADC_VOLTS_PER_COUNT_B,
        "dac_counts_per_volt": DAC_COUNTS_PER_VOLT_B,
        "group_source": normalize_group_source(GROUP_SOURCE_B),
        "feedback_source": FEEDBACK_SOURCE_B,
        "trigger_frequency_hz": TRIGGER_FREQUENCY_HZ_B,
        "trigger_min_high_points": TRIGGER_MIN_HIGH_POINTS_B,
        "trigger_delay_ns": TRIGGER_DELAY_NS_B,
        "reference_start_index": REFERENCE_START_INDEX_B,
        "reference_pulse_count": REFERENCE_PULSE_COUNT_B,
        "trigger_fraction": TRIGGER_FRACTION_B,
        "target_height_v": TARGET_HEIGHT_V_B,
        "pre_baseline_samples": PRE_BASELINE_SAMPLES_B,
        "post_baseline_samples": POST_BASELINE_SAMPLES_B,
        "min_pulse_samples": MIN_PULSE_SAMPLES_B,
        "max_pulse_samples": MAX_PULSE_SAMPLES_B,
        "adc_saturation_limit_v": ADC_SATURATION_LIMIT_V_B,
        "pulse_polarity": PULSE_POLARITY_B,
        "dac_enabled": DAC_ENABLED_B,
        "dac_frequency_hz": DAC_FREQUENCY_HZ_B,
        "dac_low_v": DAC_LOW_V_B,
        "dac_high_v": DAC_HIGH_V_B,
        "dac_sequence": DAC_SEQUENCE_B,
        "target_area_v_us": TARGET_AREA_V_US_B,
        "auto_target": AUTO_TARGET_B,
        "deadband_fraction": DEADBAND_FRACTION_B,
        "kp": KP_B,
        "ki_per_s": KI_PER_S_B,
        "kd": KD_B,
        "gain": AREA_TO_DAC_GAIN_V_PER_V_US_B,
        "correction_sign": CORRECTION_SIGN_B,
        "max_dac_step_v": MAX_DAC_STEP_V_B,
        "dac_high_min_v": DAC_HIGH_MIN_V_B,
        "dac_high_max_v": DAC_HIGH_MAX_V_B,
        "integral_limit_fraction": INTEGRAL_LIMIT_FRACTION_B,
    }


def open_channel(name):
    cfg = channel_config(name)
    return PulseControl316(
        name,
        adc_volts_per_count=cfg["adc_volts_per_count"],
        dac_counts_per_volt=cfg["dac_counts_per_volt"],
    )


def apply_channel(control, cfg):
    control.enable_feedback(False)
    control.enable_measurement(False)
    control.enable_continuous_fifo_logging(False)
    control.set_group_source("manual")
    control.enable_dac(False)

    control.configure_dac_high_limits(
        cfg["dac_high_min_v"],
        cfg["dac_high_max_v"],
    )

    control.configure_dac(
        cfg["dac_frequency_hz"],
        cfg["dac_low_v"],
        cfg["dac_high_v"],
        cfg["dac_sequence"],
    )

    control.configure_measurement(
        trigger_fraction=cfg["trigger_fraction"],
        target_height_v=cfg["target_height_v"],
        pre_baseline_samples=cfg["pre_baseline_samples"],
        post_baseline_samples=cfg["post_baseline_samples"],
        min_pulse_samples=cfg["min_pulse_samples"],
        max_pulse_samples=cfg["max_pulse_samples"],
        adc_saturation_limit_v=cfg["adc_saturation_limit_v"],
        pulse_polarity=cfg["pulse_polarity"],
        trigger_delay_ns=cfg["trigger_delay_ns"],
        trigger_min_high_points=cfg["trigger_min_high_points"],
        trigger_frequency_hz=cfg["trigger_frequency_hz"],
        reference_start_index=cfg["reference_start_index"],
        reference_pulse_count=cfg["reference_pulse_count"],
    )

    control.set_feedback_source(cfg["feedback_source"])

    if cfg["group_source"] == "dac":
        controller_frequency_hz = cfg["dac_frequency_hz"]
    else:
        controller_frequency_hz = cfg["trigger_frequency_hz"]

    pid_configured = control.configure_feedback(
        target_area_v_us=cfg["target_area_v_us"],
        auto_target=cfg["auto_target"],
        deadband_fraction=cfg["deadband_fraction"],
        kp=cfg["kp"],
        ki_per_s=cfg["ki_per_s"],
        kd=cfg["kd"],
        area_to_dac_gain_v_per_v_us=cfg["gain"],
        correction_sign=cfg["correction_sign"],
        max_dac_step_v=cfg["max_dac_step_v"],
        integral_limit_fraction=cfg["integral_limit_fraction"],
        trigger_frequency_hz=controller_frequency_hz,
    )

    control.clear_all()
    control.enable_feedback(False)
    control.enable_measurement(False)
    control.enable_continuous_fifo_logging(False)
    control.set_group_source("manual")
    control.enable_dac(cfg["dac_enabled"])

    return pid_configured


def start_measurement_session(control, cfg, feedback=False):
    control.enable_feedback(False)
    control.enable_measurement(False)
    control.enable_continuous_fifo_logging(False)
    control.set_group_source("manual")
    control.clear_fifos()
    control.clear_flags()
    control.clear_state()

    control.enable_measurement(True)
    time.sleep(0.001)
    control.set_group_source(cfg["group_source"])

    control.sanity_check(
        require_measurement=True,
        require_dac=(cfg["group_source"] == "dac" or feedback),
        require_feedback=feedback,
    )

    if feedback:
        if cfg["gain"] is None:
            raise RuntimeError(
                "Channel {0}: --feedback requires AREA_TO_DAC_GAIN_V_PER_V_US_{0}".format(
                    control.channel
                )
            )
        if control.get_feedback_source() != cfg["feedback_source"]:
            raise RuntimeError(
                "Channel {0}: feedback source readback mismatch".format(
                    control.channel
                )
            )
        control.enable_feedback(True)


def stop_measurement_session(control):
    control.enable_feedback(False)
    control.enable_measurement(False)
    control.enable_continuous_fifo_logging(False)
    control.set_group_source("manual")
    control.clear_state()
    control.clear_fifos()


def print_status(control):
    s = control.read_snapshot()
    status = s["status"]
    cfg = control.reference_config()

    print("")
    print("PC09 Channel {0}".format(control.channel))
    print("----------------")
    print("channel ID:             {0}".format(s["channel_id"]))
    print("capabilities:           0x{0:08X}".format(s["capabilities"]))
    print("group/reference source: {0}".format(control.get_group_source()))
    print("PID feedback source:    {0}".format(control.get_feedback_source()))
    print("measurement enabled:    {0}".format(int(status["measurement_enabled"])))
    print("feedback enabled:       {0}".format(int(status["feedback_enabled"])))
    print("DAC enabled:            {0}".format(int(status["dac_enabled"])))
    print("measurement config:     {0}".format(int(status["measurement_config_valid"])))
    print("PID config:             {0}".format(int(status["controller_config_valid"])))
    print("DAC config:             {0}".format(int(status["dac_config_valid"])))
    print("external triggers:      {0}".format(s["external_trigger_count"]))
    print("reference window:       {0}".format(int(s["reference_window_active"])))
    print(
        "reference target:       {0} .. {1} (count={2})".format(
            cfg["target_start_index"],
            cfg["target_start_index"] + cfg["target_pulse_count"] - 1,
            cfg["target_pulse_count"],
        )
    )
    print(
        "last ref ID/sum:        {0} / {1} ({2:.7g} V*us)".format(
            s["last_reference_id"],
            s["last_reference_sum"],
            control.area_v_us_from_counts(s["last_reference_sum"]),
        )
    )
    print(
        "last ref count/valid:   {0} / {1}".format(
            s["last_reference_count"],
            int(s["last_reference_valid"]),
        )
    )
    print(
        "valid / invalid pulses: {0} / {1}".format(
            s["valid_count"],
            s["invalid_count"],
        )
    )
    print("last invalid reason:    {0}".format(s["last_invalid_reason"]))
    print(
        "target / error:         {0} / {1}".format(
            s["target"],
            s["error"],
        )
    )
    print(
        "P / I / D:              {0} / {1} / {2}".format(
            s["p"],
            s["i"],
            s["d"],
        )
    )
    print(
        "corr pending / active:  {0} / {1}".format(
            s["correction_pending"],
            s["correction_active"],
        )
    )
    print("effective HIGH:         {0}".format(s["effective_high"]))
    print(
        "PID updates / latency:  {0} / {1} cycles".format(
            s["update_count"],
            s["processing_latency_cycles"],
        )
    )
    print(
        "pulse / group FIFO:     {0} / {1}".format(
            s["pulse_fifo_level"],
            s["group_fifo_level"],
        )
    )
    print(
        "FIFO overflow:          {0} / {1}".format(
            s["pulse_fifo_overflow"],
            s["group_fifo_overflow"],
        )
    )
    print("STATUS raw:             0x{0:08X}".format(s["status_raw"]))


def selected_names(args):
    if getattr(args, "select_a", False):
        return ["A"]
    if getattr(args, "select_b", False):
        return ["B"]
    return ["A", "B"]


def open_selected(args):
    controls = {}
    try:
        for name in selected_names(args):
            controls[name] = open_channel(name)
        return controls
    except Exception:
        for control in controls.values():
            control.close()
        raise


def add_channel_flags(parser):
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--A",
        "--a",
        dest="select_a",
        action="store_true",
        help="operate on Channel A only; default is both channels",
    )
    group.add_argument(
        "--B",
        "--b",
        dest="select_b",
        action="store_true",
        help="operate on Channel B only; default is both channels",
    )


def arm_snapshots_together(controls):
    expected = {}
    for name in sorted(controls.keys()):
        expected[name] = controls[name].begin_selected_reference_snapshot()
    return expected


def collect_snapshots_together(controls, expected, timeout_s):
    deadline = time.monotonic() + float(timeout_s)

    while time.monotonic() < deadline:
        ready = True
        for name in controls:
            level = controls[name].regs.read_u32(0x358) & 0xFFFF
            if level < expected[name]:
                ready = False
                break
        if ready:
            break
        time.sleep(0.00005)

    results = {}
    for name in controls:
        results[name] = controls[name].collect_selected_reference_snapshot(
            expected_count=expected[name]
        )
    return results


def snapshot_once_many(controls, timeout_s):
    for control in controls.values():
        control.enable_continuous_fifo_logging(False)

    expected = arm_snapshots_together(controls)
    return collect_snapshots_together(
        controls,
        expected,
        timeout_s,
    )


def channel_csv_fields(prefix, target_count):
    p = str(prefix)
    fields = [
        p + "snapshot_complete",
        p + "reference_id",
        p + "reference_valid",
        p + "reference_sum_area_v_us",
        p + "target_reference_sum_v_us",
        p + "area_error_measured_minus_target_v_us",
        p + "normalized_area_error",
        p + "observed_target_pulse_count",
    ]

    for j in range(int(target_count)):
        q = p + "target{0}_".format(j)
        fields.extend(
            [
                q + "pulse_id",
                q + "fpga_time_s",
                q + "valid",
                q + "invalid_reason",
                q + "pulse_duration_s",
                q + "baseline_pre_v",
                q + "baseline_post_v",
                q + "baseline_for_area_v",
                q + "pulse_height_v",
                q + "peak_height_v",
                q + "pulse_area_v_us",
                q + "correction_active_v",
                q + "effective_high_v",
            ]
        )
    return fields


def snapshot_fields(control, result, prefix):
    p = str(prefix)
    row = {
        p + "snapshot_complete": int(
            bool(
                result["complete"]
                and result["same_reference"]
                and result["pulse_ids_match"]
            )
        ),
        p + "reference_id": (
            ""
            if result["reference_id"] is None
            else result["reference_id"]
        ),
        p + "reference_valid": int(bool(result["reference_valid"])),
        p + "reference_sum_area_v_us": result["reference_sum_v_us"],
        p + "observed_target_pulse_count": result["observed_count"],
    }

    snap = control.read_snapshot()
    target_counts = snap["target_area_counts_samples"]
    target_v_us = control.area_v_us_from_counts(target_counts)
    row[p + "target_reference_sum_v_us"] = target_v_us

    if target_counts != 0 and result["complete"]:
        error_v_us = result["reference_sum_v_us"] - target_v_us
        row[p + "area_error_measured_minus_target_v_us"] = error_v_us
        row[p + "normalized_area_error"] = error_v_us / target_v_us
    else:
        row[p + "area_error_measured_minus_target_v_us"] = ""
        row[p + "normalized_area_error"] = ""

    for j, pulse in enumerate(result["pulses"]):
        q = p + "target{0}_".format(j)
        row[q + "pulse_id"] = pulse["pulse_id"]
        row[q + "fpga_time_s"] = pulse["time_s"]
        row[q + "valid"] = int(bool(pulse["valid"]))
        row[q + "invalid_reason"] = pulse["invalid_reason"]
        row[q + "pulse_duration_s"] = pulse["duration_s"]
        row[q + "baseline_pre_v"] = pulse["bpre_v"]
        row[q + "baseline_post_v"] = pulse["bpost_v"]
        row[q + "baseline_for_area_v"] = pulse["baseline_for_area_v"]
        row[q + "pulse_height_v"] = (
            ""
            if pulse["pulse_height_v"] is None
            else pulse["pulse_height_v"]
        )
        row[q + "peak_height_v"] = pulse["peak_height_v"]
        row[q + "pulse_area_v_us"] = pulse["area_v_us"]
        row[q + "correction_active_v"] = pulse[
            "correction_active_v"
        ]
        row[q + "effective_high_v"] = pulse["effective_high_v"]

    return row


def sample_reference_csv(
    controls,
    cfgs,
    csv_path,
    duration_s,
    interval_s,
    feedback=False,
    timeout_s=SNAPSHOT_TIMEOUT_S,
):
    duration_s = float(duration_s)
    interval_s = float(interval_s)
    timeout_s = float(timeout_s)

    if duration_s <= 0.0:
        raise ValueError("--duration-s must be positive")
    if interval_s <= 0.0:
        raise ValueError("--interval-s must be positive")
    if timeout_s <= 0.0:
        raise ValueError("--timeout-s must be positive")

    names = sorted(controls.keys())

    for name in names:
        if cfgs[name]["group_source"] == "manual":
            raise RuntimeError(
                "Channel {0}: sample requires GROUP_SOURCE external or dac".format(
                    name
                )
            )
        if (
            cfgs[name]["group_source"] == "dac"
            and not cfgs[name]["dac_enabled"]
        ):
            raise RuntimeError(
                "Channel {0}: GROUP_SOURCE='dac' requires DAC_ENABLED=True".format(
                    name
                )
            )
        if feedback and cfgs[name]["gain"] is None:
            raise RuntimeError(
                "Channel {0}: --feedback requires calibrated AREA_TO_DAC_GAIN".format(
                    name
                )
            )

    target_counts = {}
    for name in names:
        target_counts[name] = int(
            controls[name].reference_config()["target_pulse_count"]
        )

    fields = ["sample_index", "host_elapsed_s"]
    multi = len(names) > 1
    for name in names:
        prefix = name + "_" if multi else ""
        fields.extend(channel_csv_fields(prefix, target_counts[name]))

    # Same safe start policy as the original PC08 logger.
    for name in names:
        start_measurement_session(
            controls[name],
            cfgs[name],
            feedback=feedback,
        )

    start = time.monotonic()
    next_sample = start
    sample_index = 0

    try:
        with open(csv_path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            handle.flush()

            print(
                "PC09 sparse reference sampling: channels={0}, interval={1:g}s -> {2}".format(
                    ",".join(names),
                    interval_s,
                    csv_path,
                )
            )

            while True:
                now = time.monotonic()
                if now - start >= duration_s:
                    break

                if now < next_sample:
                    time.sleep(min(next_sample - now, 0.02))
                    continue

                results = snapshot_once_many(
                    controls,
                    timeout_s=timeout_s,
                )
                elapsed = time.monotonic() - start

                row = {
                    "sample_index": sample_index,
                    "host_elapsed_s": elapsed,
                }

                summaries = []
                for name in names:
                    prefix = name + "_" if multi else ""
                    row.update(
                        snapshot_fields(
                            controls[name],
                            results[name],
                            prefix,
                        )
                    )
                    summaries.append(
                        "{0}:ref={1} valid={2} sum={3:.7g}".format(
                            name,
                            results[name]["reference_id"],
                            int(results[name]["reference_valid"]),
                            float(results[name]["reference_sum_v_us"]),
                        )
                    )

                writer.writerow(row)
                handle.flush()

                print(
                    "{0:4d} t={1:8.3f}s {2}".format(
                        sample_index,
                        elapsed,
                        " | ".join(summaries),
                    )
                )

                sample_index += 1
                next_sample = start + sample_index * interval_s

    finally:
        for name in names:
            stop_measurement_session(controls[name])

    return {
        "csv": csv_path,
        "samples": sample_index,
        "duration_s": time.monotonic() - start,
        "channels": names,
    }


def save_waveform_csv(control, result, csv_path):
    trigger_index = int(result["trigger_index"])
    with open(csv_path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "sample_index",
                "time_relative_trigger_s",
                "adc_counts",
                "adc_v",
                "reference_window_active",
            ]
        )
        for i in range(int(result["valid_sample_count"])):
            writer.writerow(
                [
                    i,
                    float(i - trigger_index) / 125000000.0,
                    result["adc_counts"][i],
                    result["adc_v"][i],
                    int(bool(result["reference_window_active"][i])),
                ]
            )


def waveform_csv_path(base_path, name, multiple):
    if not multiple:
        return base_path
    if base_path.lower().endswith(".csv"):
        return base_path[:-4] + "_" + name + ".csv"
    return base_path + "_" + name + ".csv"


def capture_waveform(control, cfg, length, pretrigger, timeout, csv_path):
    started_session = False
    status = control.read_status()

    if not status["measurement_enabled"]:
        start_measurement_session(control, cfg, feedback=False)
        started_session = True
    elif control.get_group_source() != cfg["group_source"]:
        raise RuntimeError(
            "Channel {0}: active source is {1}, config requests {2}".format(
                control.channel,
                control.get_group_source(),
                cfg["group_source"],
            )
        )

    try:
        result = control.acquire_waveform(
            length_samples=length,
            pretrigger_samples=pretrigger,
            timeout_s=timeout,
        )
        if csv_path:
            save_waveform_csv(control, result, csv_path)
        return result
    finally:
        if started_session:
            stop_measurement_session(control)


def monitor(controls, interval_s, count):
    interval_s = float(interval_s)
    count = int(count)
    if interval_s <= 0.0:
        raise ValueError("--interval must be positive")

    print(
        "channel time_s trig ref_id ref_sum ref_valid win target error P I D corr fifo reason"
    )
    start = time.monotonic()
    n = 0

    while count <= 0 or n < count:
        elapsed = time.monotonic() - start
        for name in sorted(controls.keys()):
            snap = controls[name].read_snapshot()
            print(
                "{0:>7} {1:8.3f} {2:8d} {3:8d} {4:10d} {5:3d} {6:3d} "
                "{7:10d} {8:10d} {9:8d} {10:8d} {11:8d} {12:8d} {13:4d} {14}".format(
                    name,
                    elapsed,
                    int(snap["external_trigger_count"]),
                    int(snap["last_reference_id"]),
                    int(snap["last_reference_sum"]),
                    int(snap["last_reference_valid"]),
                    int(snap["reference_window_active"]),
                    int(snap["target"]),
                    int(snap["error"]),
                    int(snap["p"]),
                    int(snap["i"]),
                    int(snap["d"]),
                    int(snap["correction_active"]),
                    int(snap["pulse_fifo_level"]),
                    snap["last_invalid_reason"],
                )
            )
        n += 1
        time.sleep(interval_s)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "PC09 dual-channel pulse control. "
            "No channel flag means A+B; add --A or --B for one channel."
        )
    )
    sub = parser.add_subparsers(dest="command")

    simple_commands = [
        ("apply", "apply edited A/B configuration"),
        ("show", "show live status"),
        ("outputs-on", "enable selected DAC output(s)"),
        ("outputs-off", "disable selected DAC output(s)"),
        ("measurement-on", "start measurement/reference session"),
        ("measurement-off", "stop measurement/reference session"),
        ("feedback-on", "enable PID feedback"),
        ("feedback-off", "disable PID feedback"),
        ("clear", "clear state, flags, and FIFOs"),
        ("clear-state", "clear controller/measurement state"),
        ("clear-flags", "clear sticky flags"),
        ("clear-fifos", "clear pulse/group FIFOs"),
        ("waveform-status", "show waveform capture status"),
        ("waveform-abort", "abort waveform capture"),
    ]

    for command, help_text in simple_commands:
        p = sub.add_parser(command, help=help_text)
        add_channel_flags(p)

    once = sub.add_parser(
        "snapshot-once",
        help="capture next complete selected PID reference",
    )
    once.add_argument(
        "--timeout-s",
        type=float,
        default=SNAPSHOT_TIMEOUT_S,
    )
    add_channel_flags(once)

    sample = sub.add_parser(
        "sample",
        help="sample complete selected references to CSV",
    )
    sample.add_argument("--duration-s", type=float, required=True)
    sample.add_argument("--interval-s", type=float, default=0.1)
    sample.add_argument(
        "--timeout-s",
        type=float,
        default=SNAPSHOT_TIMEOUT_S,
    )
    sample.add_argument("--csv", default=CSV_FILENAME)
    sample.add_argument("--feedback", action="store_true")
    add_channel_flags(sample)

    mon = sub.add_parser("monitor", help="live text monitor")
    mon.add_argument("--interval", type=float, default=0.2)
    mon.add_argument("--count", type=int, default=0)
    add_channel_flags(mon)

    fs = sub.add_parser(
        "set-feedback-source",
        help="write selected/average feedback-source register",
    )
    fs.add_argument("source", choices=["selected", "average"])
    add_channel_flags(fs)

    wave = sub.add_parser(
        "waveform-capture",
        help="capture raw ADC/reference-marker waveform",
    )
    wave.add_argument(
        "--length-samples",
        type=int,
        default=WAVEFORM_LENGTH_SAMPLES,
    )
    wave.add_argument(
        "--pretrigger-samples",
        type=int,
        default=WAVEFORM_PRETRIGGER_SAMPLES,
    )
    wave.add_argument(
        "--timeout-s",
        type=float,
        default=WAVEFORM_TIMEOUT_S,
    )
    wave.add_argument("--csv", default=WAVEFORM_CSV_FILENAME)
    add_channel_flags(wave)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    if args.command is None:
        parser.print_help()
        return 1

    controls = {}
    try:
        controls = open_selected(args)
        cfgs = {}
        for name in controls:
            cfgs[name] = channel_config(name)

        if args.command == "apply":
            for name in sorted(controls.keys()):
                configured = apply_channel(
                    controls[name],
                    cfgs[name],
                )
                print(
                    "Channel {0}: apply complete; PID configured={1}".format(
                        name,
                        int(configured),
                    )
                )
                print_status(controls[name])

        elif args.command == "show":
            for name in sorted(controls.keys()):
                print_status(controls[name])

        elif args.command == "outputs-on":
            for name in controls:
                controls[name].enable_dac(
                    cfgs[name]["dac_enabled"]
                )
                print_status(controls[name])

        elif args.command == "outputs-off":
            for name in controls:
                controls[name].enable_feedback(False)
                controls[name].enable_dac(False)
                print_status(controls[name])

        elif args.command == "measurement-on":
            for name in controls:
                start_measurement_session(
                    controls[name],
                    cfgs[name],
                    feedback=False,
                )
                print_status(controls[name])

        elif args.command == "measurement-off":
            for name in controls:
                stop_measurement_session(controls[name])
                print_status(controls[name])

        elif args.command == "feedback-on":
            # Validate all first so the default-both operation is not half-enabled.
            for name in controls:
                if cfgs[name]["gain"] is None:
                    raise RuntimeError(
                        "Channel {0}: set AREA_TO_DAC_GAIN_V_PER_V_US_{0} first".format(
                            name
                        )
                    )
                status = controls[name].read_status()
                if not status["measurement_enabled"]:
                    raise RuntimeError(
                        "Channel {0}: feedback-on requires measurement-on".format(
                            name
                        )
                    )
                if (
                    controls[name].get_feedback_source()
                    != cfgs[name]["feedback_source"]
                ):
                    raise RuntimeError(
                        "Channel {0}: feedback source readback mismatch".format(
                            name
                        )
                    )
                controls[name].sanity_check(
                    require_measurement=True,
                    require_dac=True,
                    require_feedback=True,
                )

            for name in controls:
                controls[name].enable_feedback(True)
                print_status(controls[name])

        elif args.command == "feedback-off":
            for name in controls:
                controls[name].enable_feedback(False)
                print_status(controls[name])

        elif args.command == "clear":
            for name in controls:
                controls[name].enable_feedback(False)
                controls[name].clear_all()
                print_status(controls[name])

        elif args.command == "clear-state":
            for name in controls:
                controls[name].enable_feedback(False)
                controls[name].clear_state()

        elif args.command == "clear-flags":
            for name in controls:
                controls[name].clear_flags()

        elif args.command == "clear-fifos":
            for name in controls:
                controls[name].clear_fifos()

        elif args.command == "snapshot-once":
            results = snapshot_once_many(
                controls,
                timeout_s=args.timeout_s,
            )
            for name in sorted(results.keys()):
                print(
                    "Channel {0} reference snapshot: {1}".format(
                        name,
                        results[name],
                    )
                )

        elif args.command == "sample":
            result = sample_reference_csv(
                controls,
                cfgs,
                csv_path=args.csv,
                duration_s=args.duration_s,
                interval_s=args.interval_s,
                feedback=args.feedback,
                timeout_s=args.timeout_s,
            )
            print("Sampling complete: {0}".format(result))
            for name in sorted(controls.keys()):
                print_status(controls[name])

        elif args.command == "monitor":
            monitor(
                controls,
                interval_s=args.interval,
                count=args.count,
            )

        elif args.command == "set-feedback-source":
            for name in controls:
                controls[name].enable_feedback(False)
                controls[name].set_feedback_source(args.source)
                controls[name].clear_state()
                print(
                    "Channel {0} PID feedback source: {1}".format(
                        name,
                        controls[name].get_feedback_source(),
                    )
                )

        elif args.command == "waveform-status":
            for name in sorted(controls.keys()):
                print(
                    "Channel {0} waveform status: {1}".format(
                        name,
                        controls[name].get_waveform_status(),
                    )
                )

        elif args.command == "waveform-capture":
            multiple = len(controls) > 1
            for name in sorted(controls.keys()):
                out_csv = waveform_csv_path(
                    args.csv,
                    name,
                    multiple,
                )
                result = capture_waveform(
                    controls[name],
                    cfgs[name],
                    length=args.length_samples,
                    pretrigger=args.pretrigger_samples,
                    timeout=args.timeout_s,
                    csv_path=out_csv,
                )
                print(
                    "Channel {0} waveform complete: id={1}, samples={2}, "
                    "trigger_index={3} -> {4}".format(
                        name,
                        result["capture_id"],
                        result["valid_sample_count"],
                        result["trigger_index"],
                        out_csv,
                    )
                )

        elif args.command == "waveform-abort":
            for name in controls:
                controls[name].abort_waveform_capture()

        return 0

    except PermissionError:
        print(
            "Permission denied opening /dev/mem. Run as root.",
            file=sys.stderr,
        )
        return 1
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print("Error: {0}".format(error), file=sys.stderr)
        return 1
    finally:
        for control in controls.values():
            control.close()


if __name__ == "__main__":
    sys.exit(main())

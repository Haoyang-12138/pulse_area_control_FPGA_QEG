#!/usr/bin/env python3

import argparse
import csv
import sys
import time

from pulse_control import PulseControl

# Theoretical STEMlab 125-14 calibration. Convert counts to voltage
#Replace with measured values if needed, but if you are also using this to do the setpoint, it's not needed
ADC_VOLTS_PER_COUNT = 1.0 / 8192.0
DAC_COUNTS_PER_VOLT = 8192.0


# 1. External experimental group trigger on DIO0_P.
# This is the expected trigger/group repetition rate used only to discretize KI and KD in software 
# The FPGA does not synthesize this clock.
EXTERNAL_TRIGGER_FREQUENCY_HZ = 1000000.0

# DIO0_P rising edge -> 200 MHz synchronizer -> this delay -> group event.
# Resolution is 5 ns
EXTERNAL_TRIGGER_DELAY_NS = 650

# Recommended external TTL pulse HIGH width is >= 10 ns so the asynchronous input is reliably observed by the 200 MHz synchronizer.

# 2. Fast GPIO sequence on DIO1_P. 5 ns resolution.
GPIO_ENABLED = True
GPIO_FREQUENCY_HZ = 1000000.0

# Each pair: (start fraction, end fraction) within one GPIO period.
GPIO_PULSES = [
    (0.000, 0.010),
]
#3. OUT2 analog sequence output, 8 ns resolution.
#OUT2 was changed to 0-2V by removing 2 resistors, so 0V here is actually output 1V

DAC2_ENABLED = True
DAC2_FREQUENCY_HZ = 1000000.0
DAC2_LOW_VOLTAGE_V = 0.0

# Leave headroom for pid
DAC2_HIGH_VOLTAGE_V = 0.500

# Each pair :(normalized level, fraction of one DAC2 period)
DAC2_SEQUENCE = [
    (1.0, 1.0),
]

# 4. ADC IN1 measurement
MEASUREMENT_ENABLED = False
TRIGGER_FRACTION = 0.25
TARGET_HEIGHT_V = 0.3
PRE_BASELINE_SAMPLES = 16
POST_BASELINE_SAMPLES = 16
MIN_PULSE_SAMPLES = 3
MAX_PULSE_SAMPLES = 16384
ADC_SATURATION_LIMIT_V = 0.95
PULSE_POLARITY = "positive"

# 0 means do not enforce an exact count.
EXPECTED_PULSES_PER_GROUP = 0
MIN_VALID_PULSES_PER_GROUP = 1

# Choose what defines one acquisition group:
#   "external" "dac2" "manual" 
GROUP_SOURCE = "external"


# 5. PID controller.

FEEDBACK_ENABLED = False

TARGET_MEAN_PULSE_AREA_V_US = None
USE_FIRST_VALID_GROUP_AS_TARGET = True

AREA_ERROR_DEADBAND_V_US = None
AREA_ERROR_DEADBAND_FRACTION = 0.005

KP = 0.20

KI_PER_S = 0.0

# D_area = KD * d(error)/dt. 0.0 gives PI/P behaviour.
KD = 0.0

# REQUIRED before FEEDBACK_ENABLED=True.

AREA_TO_DAC_GAIN_V_PER_V_US = None
DAC_CORRECTION_SIGN = -1.0

MAX_DAC_STEP_PER_UPDATE_V = 0.020
DAC_HIGH_VOLTAGE_MIN_V = DAC2_LOW_VOLTAGE_V + 0.010
DAC_HIGH_VOLTAGE_MAX_V = 1.000
MAX_INTEGRAL_AREA_TERM_FRACTION = 0.25

MARKER_ENABLED = True


# 6. Single CSV logging.

CSV_FILENAME = "pulse_control_data.csv"
CSV_WRITE_INTERVAL_S = 0.10

CSV_FIFO_POLL_INTERVAL_S = 0.001





# ===========================================================================
def open_control():
    return PulseControl(
        adc_volts_per_count=ADC_VOLTS_PER_COUNT,
        dac_counts_per_volt=DAC_COUNTS_PER_VOLT,
    )


def apply_configuration(control):
    if EXTERNAL_TRIGGER_FREQUENCY_HZ <= 0.0:
        raise ValueError("EXTERNAL_TRIGGER_FREQUENCY_HZ must be positive")
    if DAC2_ENABLED and abs(DAC2_FREQUENCY_HZ - EXTERNAL_TRIGGER_FREQUENCY_HZ) > 1.0e-6:
        print(
            "WARNING: DAC2 frequency ({0:g} Hz) differs from external group trigger "
            "({1:g} Hz). Correction is still applied only on DAC2 period boundaries.".format(
                DAC2_FREQUENCY_HZ, EXTERNAL_TRIGGER_FREQUENCY_HZ
            )
        )

    # Safe starting state. Sparse FIFO logging is the default runtime mode.
    # Disable the old every-record logger before touching measurement/grouping.
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

    control.configure_measurement(
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
        external_trigger_frequency_hz=EXTERNAL_TRIGGER_FREQUENCY_HZ,
        enable=False,
    )

    feedback_info = None
    if AREA_TO_DAC_GAIN_V_PER_V_US is not None:
        feedback_info = control.configure_feedback(
            target_mean_pulse_area_v_us=TARGET_MEAN_PULSE_AREA_V_US,
            use_first_valid_group_as_target=USE_FIRST_VALID_GROUP_AS_TARGET,
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
            controller_dt_s=(1.0 / EXTERNAL_TRIGGER_FREQUENCY_HZ)
                if GROUP_SOURCE == "external" else None,
            enable=False,
        )
    elif FEEDBACK_ENABLED:
        raise RuntimeError(
            "FEEDBACK_ENABLED=True but AREA_TO_DAC_GAIN_V_PER_V_US is None"
        )

    control.enable_marker(MARKER_ENABLED)
    control.clear_flags()
    control.clear_fifos()
    control.clear_state()

    # Safe idle mode: outputs may run, but measurement/feedback remain OFF and
    # the group source is MANUAL.  Otherwise every DAC2 period creates an empty
    # group even with measurement disabled, filling the 256-entry group FIFO.
    control.enable_measurement(False)
    control.enable_feedback(False)
    control.set_group_source("manual")

    # One CONTROL write starts GPIO and OUT2 as close together as the two clock
    # domains allow. This is not exact phase locking; see notes below.
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

    return gpio_info, dac2_info, feedback_info


def print_status(control):
    snap = control.read_snapshot()
    status = snap["status"]
    print("PC05 status")
    print("-----------")
    print("group source:               {0}".format(control.get_group_source()))
    print("continuous FIFO logging:    {0}".format(
        int(control.continuous_fifo_logging_enabled())))
    print("measurement enabled/config: {0}/{1}".format(
        int(status["measurement_enabled"]), int(status["measurement_config_valid"])))
    print("GPIO enabled/config:        {0}/{1}".format(
        int(status["gpio_enabled"]), int(status["gpio_config_valid"])))
    print("DAC2 enabled/config:        {0}/{1}".format(
        int(status["dac2_enabled"]), int(status["dac2_config_valid"])))
    print("feedback enabled/config:    {0}/{1}".format(
        int(status["feedback_enabled"]), int(status["controller_config_valid"])))
    print("target latched:             {0}".format(int(status["target_latched"])))
    print("valid / invalid pulses:     {0} / {1}".format(
        snap["valid_count"], snap["invalid_count"]))
    print("last invalid reason:        {0}".format(snap["last_invalid_reason"]))
    print("group count:                {0}".format(snap["group_count"]))
    print("area [count*sample]:        {0}".format(snap["area_counts_samples"]))
    print("target [count*sample]:      {0}".format(snap["target_area_counts_samples"]))
    print("error [count*sample]:       {0}".format(snap["area_error_counts_samples"]))
    print("P / I / D:                  {0} / {1} / {2}".format(
        snap["p_area"], snap["i_area"], snap["d_area"]))
    print("correction pending/active:  {0} / {1} DAC counts".format(
        snap["correction_pending_counts"], snap["correction_active_counts"]))
    print("effective OUT2 HIGH:        {0} counts".format(snap["effective_high_counts"]))
    print("PID processing latency:     {0} cycles".format(snap["processing_latency_cycles"]))
    print("external trigger count:      {0}".format(snap["external_trigger_count"]))
    print("trigger delay active:        {0} cycles ({1:.1f} ns)".format(
        snap["external_trigger_active_delay_cycles"],
        5.0 * snap["external_trigger_active_delay_cycles"]))
    print("trigger overrun seen:        {0}".format(
        int(status["external_trigger_overrun_seen"])))
    print("pulse/group FIFO level:     {0} / {1}".format(
        snap["pulse_fifo_level"], snap["group_fifo_level"]))
    print("pulse/group FIFO overflow:  {0} / {1}".format(
        snap["pulse_fifo_overflow"], snap["group_fifo_overflow"]))


def monitor(control, interval_s, count):
    print("time_s valid invalid groups area target error P I D corr_active latency fifo_p fifo_g reason")
    start = time.monotonic()
    n = 0
    while count <= 0 or n < count:
        snap = control.read_snapshot()
        print(
            "{0:8.3f} {1:8d} {2:8d} {3:8d} {4:10d} {5:10d} {6:10d} "
            "{7:8d} {8:8d} {9:8d} {10:8d} {11:7d} {12:6d} {13:6d} {14}".format(
                time.monotonic() - start,
                int(snap["valid_count"]),
                int(snap["invalid_count"]),
                int(snap["group_count"]),
                int(snap["area_counts_samples"]),
                int(snap["target_area_counts_samples"]),
                int(snap["area_error_counts_samples"]),
                int(snap["p_area"]),
                int(snap["i_area"]),
                int(snap["d_area"]),
                int(snap["correction_active_counts"]),
                int(snap["processing_latency_cycles"]),
                int(snap["pulse_fifo_level"]),
                int(snap["group_fifo_level"]),
                snap["last_invalid_reason"],
            )
        )
        n += 1
        time.sleep(interval_s)


def build_parser():
    parser = argparse.ArgumentParser(description="PC05 v3.13.2 sparse-snapshot/external-trigger/Q31-Ki runtime tool")
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("apply", help="apply all edited settings above")
    sub.add_parser("show", help="show live FPGA status")
    sub.add_parser("outputs-on", help="enable outputs selected by GPIO_ENABLED/DAC2_ENABLED")
    sub.add_parser("outputs-off", help="disable GPIO and DAC2 together")
    sub.add_parser("measurement-on")
    sub.add_parser("measurement-off")
    sub.add_parser("feedback-on")
    sub.add_parser("feedback-off")
    sub.add_parser("marker-on")
    sub.add_parser("marker-off")
    sub.add_parser("clear-state")
    sub.add_parser("clear-flags")
    sub.add_parser("clear-fifos")
    sub.add_parser("manual-group")

    dc = sub.add_parser("dac2-dc", help="DAC2-only one-segment DC hardware test")
    dc.add_argument("--counts", type=int, required=True,
                    help="signed DAC command counts, -8192..8191; 0 is DAC midscale")

    sq = sub.add_parser("dac2-square", help="DAC2-only slow square-wave hardware test")
    sq.add_argument("--low-counts", type=int, default=0)
    sq.add_argument("--high-counts", type=int, required=True)
    sq.add_argument("--frequency", type=float, default=100.0)
    sq.add_argument("--duty", type=float, default=0.5)

    mon = sub.add_parser("monitor")
    mon.add_argument("--interval", type=float, default=0.2)
    mon.add_argument("--count", type=int, default=0, help="0 means run until Ctrl+C")

    sample = sub.add_parser("sample", help="save one fresh pulse snapshot every N seconds")
    sample.add_argument("--duration-s", type=float, required=True)
    sample.add_argument("--interval-s", type=float, default=0.5)
    sample.add_argument("--csv", default="measurement.csv")
    sample.add_argument("--feedback", action="store_true", help="enable PID during sampling")

    record = sub.add_parser("record", help="record one complete-group CSV")
    record.add_argument("--csv", default=None, help="override CSV_FILENAME")
    record.add_argument("--duration-s", type=float, default=0.0,
                        help="0 means run until Ctrl+C")
    record.add_argument("--write-interval-s", type=float, default=None,
                        help="override CSV_WRITE_INTERVAL_S; disk batching only")

    meas = sub.add_parser("measure", help="FIFO-safe measurement + CSV in one process")
    meas.add_argument("--csv", default=None, help="override CSV_FILENAME")
    meas.add_argument("--duration-s", type=float, default=60.0)
    meas.add_argument("--write-interval-s", type=float, default=None)

    loop = sub.add_parser("closed-loop", help="FIFO-safe measurement + PID + CSV")
    loop.add_argument("--csv", default=None, help="override CSV_FILENAME")
    loop.add_argument("--duration-s", type=float, default=60.0)
    loop.add_argument("--write-interval-s", type=float, default=None)

    return parser



def sample_snapshot_csv(control, csv_path, duration_s, interval_s, feedback=False):
    """Save exactly one fresh coherent pulse record every interval_s.

    Measurement/group/PID continue at full FPGA rate. FIFO logging is OFF
    between samples. At each host sample time Python arms a hardware one-shot;
    exactly the next completed pulse is pushed into the pulse FIFO, then the
    FPGA auto-disarms immediately.
    """
    duration_s = float(duration_s)
    interval_s = float(interval_s)
    if duration_s <= 0.0:
        raise ValueError("--duration-s must be positive")
    if interval_s <= 0.0:
        raise ValueError("--interval-s must be positive")
    if GROUP_SOURCE not in ("external", "dac2"):
        raise RuntimeError(
            "Snapshot logger requires GROUP_SOURCE='external' or GROUP_SOURCE='dac2'"
        )

    fields = [
        "sample_index", "host_elapsed_s", "pulse_fpga_time_s",
        "group_id", "pulse_id", "valid", "invalid_reason",
        "baseline_pre_v", "baseline_post_v", "baseline_for_area_v",
        "threshold_v", "pulse_duration_s", "pulse_height_v",
        "peak_height_v", "pulse_area_v_us",
        "correction_active_v", "effective_high_v",
        "live_target_area_v_us", "live_area_error_v_us",
        "live_p_term_v_us", "live_i_term_v_us", "live_d_term_v_us",
        "live_pid_latency_cycles",
    ]

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

        # Prime only the detector. Logging is still disabled.
        control.enable_measurement(True)
        time.sleep(0.001)

        status = control.read_status()
        static_problems = []
        if not status["measurement_config_valid"]:
            static_problems.append("measurement_config_valid=0")
        if (GROUP_SOURCE == "dac2" or feedback) and not status["dac2_config_valid"]:
            static_problems.append("dac2_config_valid=0")
        if feedback and not status["controller_config_valid"]:
            static_problems.append("controller_config_valid=0")
        if static_problems:
            raise RuntimeError(
                "PC05 snapshot configuration check failed: " +
                "; ".join(static_problems)
            )

        control.clear_fifos()
        control.clear_flags()
        control.set_group_source(GROUP_SOURCE)

        if feedback:
            if AREA_TO_DAC_GAIN_V_PER_V_US is None:
                raise RuntimeError("--feedback requires AREA_TO_DAC_GAIN_V_PER_V_US")
            control.enable_feedback(True)

        print(
            "Sparse snapshot started: source={0}, {1:.3f} s, "
            "one pulse every {2:.3f} s -> {3}".format(
                GROUP_SOURCE, duration_s, interval_s, csv_path
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

                # clear_fifos also cancels a stale one-shot arm in hardware.
                control.clear_fifos()
                control.arm_pulse_snapshot()

                deadline = time.monotonic() + 0.1
                pulse = None
                while time.monotonic() < deadline:
                    if control.read_status()["pulse_fifo_nonempty"]:
                        # POP immediately after the coherent head read.  In
                        # sparse mode this leaves the FIFO empty between host
                        # samples; hardware has already auto-disarmed.
                        pulse = control.read_pulse_record(pop=True)
                        break

                snap = control.read_snapshot()
                elapsed = time.monotonic() - start

                if pulse is None:
                    row = dict((name, "") for name in fields)
                    row.update({
                        "sample_index": sample_index,
                        "host_elapsed_s": elapsed,
                        "valid": 0,
                        "invalid_reason": "no_pulse_sampled",
                        "live_target_area_v_us": control.area_v_us_from_counts(snap["target_area_counts_samples"]),
                        "live_area_error_v_us": control.area_v_us_from_counts(snap["area_error_counts_samples"]),
                        "live_p_term_v_us": control.area_v_us_from_counts(snap["p_area"]),
                        "live_i_term_v_us": control.area_v_us_from_counts(snap["i_area"]),
                        "live_d_term_v_us": control.area_v_us_from_counts(snap["d_area"]),
                        "live_pid_latency_cycles": snap["processing_latency_cycles"],
                    })
                else:
                    row = {
                        "sample_index": sample_index,
                        "host_elapsed_s": elapsed,
                        "pulse_fpga_time_s": pulse["time_s"],
                        "group_id": pulse["group_id"],
                        "pulse_id": pulse["pulse_id"],
                        "valid": int(bool(pulse["valid"])),
                        "invalid_reason": pulse["invalid_reason"],
                        "baseline_pre_v": pulse["bpre_v"],
                        "baseline_post_v": pulse["bpost_v"],
                        "baseline_for_area_v": pulse["baseline_for_area_v"],
                        "threshold_v": float(pulse["threshold_counts"]) * ADC_VOLTS_PER_COUNT,
                        "pulse_duration_s": pulse["duration_s"],
                        "pulse_height_v": pulse["pulse_height_v"],
                        "peak_height_v": pulse["peak_height_v"],
                        "pulse_area_v_us": pulse["area_v_us"],
                        "correction_active_v": pulse["correction_active_v"],
                        "effective_high_v": pulse["effective_high_v"],
                        "live_target_area_v_us": control.area_v_us_from_counts(snap["target_area_counts_samples"]),
                        "live_area_error_v_us": control.area_v_us_from_counts(snap["area_error_counts_samples"]),
                        "live_p_term_v_us": control.area_v_us_from_counts(snap["p_area"]),
                        "live_i_term_v_us": control.area_v_us_from_counts(snap["i_area"]),
                        "live_d_term_v_us": control.area_v_us_from_counts(snap["d_area"]),
                        "live_pid_latency_cycles": snap["processing_latency_cycles"],
                    }

                writer.writerow(row)
                handle.flush()
                print("{0:4d}  t={1:7.3f}s  group={2}  valid={3}  area={4}".format(
                    sample_index,
                    elapsed,
                    row.get("group_id", ""),
                    row.get("valid", ""),
                    row.get("pulse_area_v_us", ""),
                ))

                sample_index += 1
                next_sample = start + sample_index * interval_s
        finally:
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.enable_continuous_fifo_logging(False)
            control.set_group_source("manual")
            control.clear_fifos()
            control.clear_flags()

    return {
        "csv": csv_path,
        "samples": sample_index,
        "duration_s": time.monotonic() - start,
    }


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
        print("Could not open PC04 register map: {0}".format(error), file=sys.stderr)
        return 1

    try:
        if args.command == "apply":
            gpio_info, dac2_info, feedback_info = apply_configuration(control)
            if gpio_info is None:
                print("GPIO configuration: skipped (GPIO_ENABLED=False)")
            else:
                print("GPIO configuration:", gpio_info)
            print("DAC2 configuration:", dac2_info)
            if feedback_info is not None:
                print("PID quantization/calibration:", feedback_info)
            else:
                print("PID not configured: AREA_TO_DAC_GAIN_V_PER_V_US is None")
            print_status(control)

        elif args.command == "show":
            print_status(control)

        elif args.command == "outputs-on":
            control.sanity_check(
                require_gpio=GPIO_ENABLED,
                require_dac2=DAC2_ENABLED,
            )
            control.set_outputs(
                gpio_enabled=GPIO_ENABLED,
                dac2_enabled=DAC2_ENABLED,
            )

        elif args.command == "outputs-off":
            control.enable_feedback(False)
            control.set_outputs(gpio_enabled=False, dac2_enabled=False)

        elif args.command == "measurement-on":
            # Debug-only.  Do not enable continuous FIFO logging here: a 1 MHz
            # detector would otherwise intentionally fill the logging FIFO.
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.enable_continuous_fifo_logging(False)
            control.set_group_source("manual")
            control.clear_state()
            control.clear_fifos()
            control.clear_flags()
            control.sanity_check(require_measurement=True)
            control.enable_measurement(True)
            time.sleep(0.001)
            control.set_group_source(GROUP_SOURCE)

        elif args.command == "measurement-off":
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.enable_continuous_fifo_logging(False)
            control.set_group_source("manual")

        elif args.command == "feedback-on":
            control.sanity_check(
                require_measurement=True,
                require_dac2=True,
                require_feedback=True,
            )
            control.enable_feedback(True)

        elif args.command == "feedback-off":
            control.enable_feedback(False)

        elif args.command == "marker-on":
            control.enable_marker(True)

        elif args.command == "marker-off":
            control.enable_marker(False)

        elif args.command == "clear-state":
            control.enable_feedback(False)
            control.clear_state()

        elif args.command == "clear-flags":
            control.clear_flags()

        elif args.command == "clear-fifos":
            control.clear_fifos()

        elif args.command == "manual-group":
            control.command(1 << 7)

        elif args.command == "dac2-dc":
            if args.counts < -8192 or args.counts > 8191:
                raise ValueError("--counts must be in -8192..8191")
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.set_group_source("manual")
            control.clear_fifos()
            control.clear_flags()
            control.set_outputs(gpio_enabled=False, dac2_enabled=False)
            # configure_dac2 uses volts internally; convert counts back through
            # DAC_COUNTS_PER_VOLT.  Prefer the one-segment LOW bypass because LOW
            # is not affected by the PID HIGH safety clamp, so the requested raw
            # count is exact even when it is zero or negative.
            if args.counts < 8191:
                low_counts = args.counts
                high_counts = args.counts + 1
                sequence = [(0.0, 1.0)]
            else:
                low_counts = 8190
                high_counts = 8191
                sequence = [(1.0, 1.0)]
            low_v = float(low_counts) / DAC_COUNTS_PER_VOLT
            high_v = float(high_counts) / DAC_COUNTS_PER_VOLT
            info = control.configure_dac2(1000.0, low_v, high_v, sequence, enable=True, apply=True)
            print("DAC2 DC test:", info)
            print_status(control)

        elif args.command == "dac2-square":
            if args.low_counts < -8192 or args.low_counts > 8191:
                raise ValueError("--low-counts must be in -8192..8191")
            if args.high_counts < -8192 or args.high_counts > 8191:
                raise ValueError("--high-counts must be in -8192..8191")
            if args.high_counts <= args.low_counts:
                raise ValueError("--high-counts must be greater than --low-counts")
            if args.frequency <= 0.0:
                raise ValueError("--frequency must be positive")
            if not 0.0 < args.duty < 1.0:
                raise ValueError("--duty must lie in (0,1)")
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.set_group_source("manual")
            control.clear_fifos()
            control.clear_flags()
            control.set_outputs(gpio_enabled=False, dac2_enabled=False)
            low_v = float(args.low_counts) / DAC_COUNTS_PER_VOLT
            high_v = float(args.high_counts) / DAC_COUNTS_PER_VOLT
            info = control.configure_dac2(
                args.frequency, low_v, high_v,
                [(1.0, args.duty), (0.0, 1.0 - args.duty)],
                enable=True, apply=True,
            )
            print("DAC2 square test:", info)
            print_status(control)

        elif args.command == "monitor":
            if args.interval <= 0.0:
                raise ValueError("--interval must be positive")
            monitor(control, args.interval, args.count)

        elif args.command == "sample":
            result = sample_snapshot_csv(
                control,
                csv_path=args.csv,
                duration_s=args.duration_s,
                interval_s=args.interval_s,
                feedback=args.feedback,
            )
            print("Snapshot sampling complete:", result)
            print_status(control)

        elif args.command in ("measure", "closed-loop"):
            if args.duration_s <= 0.0:
                raise ValueError("--duration-s must be positive")
            want_feedback = (args.command == "closed-loop")
            if want_feedback and AREA_TO_DAC_GAIN_V_PER_V_US is None:
                raise RuntimeError(
                    "closed-loop requires AREA_TO_DAC_GAIN_V_PER_V_US to be configured"
                )

            csv_path = CSV_FILENAME if args.csv is None else args.csv
            write_interval_s = (
                CSV_WRITE_INTERVAL_S
                if args.write_interval_s is None
                else args.write_interval_s
            )

            # Freeze group production while the CSV is prepared.
            control.enable_feedback(False)
            control.enable_measurement(False)
            control.set_group_source("manual")
            control.clear_fifos()
            control.clear_flags()
            control.clear_state()

            def session_start():
                # Keep external trigger disarmed while the ADC rolling baseline
                # primes, then arm DIO0_P groups.  This avoids making the first
                # experimental trigger also serve as detector initialization.
                control.set_group_source("manual")
                control.enable_measurement(True)
                time.sleep(0.001)
                control.set_group_source(GROUP_SOURCE)
                control.sanity_check(
                    require_measurement=True,
                    require_dac2=(GROUP_SOURCE == "dac2" or want_feedback),
                    require_feedback=False,
                )
                if want_feedback:
                    control.enable_feedback(True)

            def session_stop():
                control.enable_feedback(False)
                control.enable_measurement(False)
                control.set_group_source("manual")

            result = control.record_grouped_csv(
                csv_path=csv_path,
                duration_s=args.duration_s,
                write_interval_s=write_interval_s,
                fifo_poll_interval_s=CSV_FIFO_POLL_INTERVAL_S,
                session_start=session_start,
                session_stop=session_stop,
            )
            print("Session complete:", result)
            print_status(control)

        elif args.command == "record":
            csv_path = CSV_FILENAME if args.csv is None else args.csv
            write_interval_s = (
                CSV_WRITE_INTERVAL_S
                if args.write_interval_s is None
                else args.write_interval_s
            )
            result = control.record_grouped_csv(
                csv_path=csv_path,
                duration_s=args.duration_s,
                write_interval_s=write_interval_s,
                fifo_poll_interval_s=CSV_FIFO_POLL_INTERVAL_S,
            )
            print("CSV recording complete:", result)

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

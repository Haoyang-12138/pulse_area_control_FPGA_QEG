#!/usr/bin/env python3
"""
We have 200 MHz DIO0_P external group trigger with programmable delay for this py
"""

import argparse
import csv
import mmap
import os
import struct
import sys
import time
import warnings


BASE_ADDRESS = 0x40700000
MAP_SIZE = 0x1000

ADC_CLOCK_HZ = 125000000.0
GPIO_CLOCK_HZ = 200000000.0
TRIGGER_CLOCK_HZ = 200000000.0
ADC_SAMPLE_PERIOD_US = 0.008

# STEMlab 125-14 theoretical conversions.  Replace with measured calibration if need.
DEFAULT_ADC_VOLTS_PER_COUNT = 1.0 / 8192.0
DEFAULT_DAC_COUNTS_PER_VOLT = 8192.0

MAX_GPIO_PULSES = 16
MAX_DAC_SEGMENTS = 16

# Core registers
REG_CONTROL = 0x000
REG_COMMAND = 0x004
REG_STATUS = 0x008
REG_VERSION = 0x00C

# Measurement
REG_TRIGGER_FRACTION_Q16 = 0x010
REG_TARGET_HEIGHT = 0x014
REG_PRE_BASELINE = 0x018
REG_POST_BASELINE = 0x01C
REG_MIN_PULSE_SAMPLES = 0x020
REG_MAX_PULSE_SAMPLES = 0x024
REG_ADC_SATURATION_LIMIT = 0x028
REG_EXPECTED_PULSES = 0x030
REG_MIN_VALID_PULSES = 0x034
REG_GROUP_SOURCE = 0x038
REG_EXTERNAL_TRIGGER_DELAY_CYCLES = 0x03C

# PID
REG_TARGET_AREA = 0x040
REG_DEADBAND_ABS = 0x044
REG_DEADBAND_FRACTION_Q16 = 0x048
REG_DEADBAND_MODE = 0x04C
REG_KP_Q16 = 0x050
REG_KI_Q31_PER_UPDATE = 0x054
REG_AREA_TO_DAC_GAIN_Q24 = 0x058
REG_CORRECTION_SIGN = 0x05C
REG_MAX_DAC_STEP = 0x060
REG_DAC_HIGH_MIN = 0x064
REG_DAC_HIGH_MAX = 0x068
REG_INTEGRAL_MIN = 0x06C
REG_INTEGRAL_MAX = 0x070
REG_INTEGRAL_LIMIT_FRACTION_Q16 = 0x074
REG_INTEGRAL_FRACTION_MODE = 0x078
REG_KD_Q16_PER_UPDATE = 0x07C

# GPIO shadows
REG_GPIO_PERIOD = 0x080
REG_GPIO_PULSE_COUNT = 0x084
REG_GPIO_WINDOWS_BASE = 0x100

# DAC2 shadows
REG_DAC2_PERIOD = 0x180
REG_DAC2_SEGMENT_COUNT = 0x184
REG_DAC2_LOW = 0x188
REG_DAC2_HIGH = 0x18C
REG_DAC2_SEGMENTS_BASE = 0x200

# Live monitoring
REG_BPRE = 0x300
REG_BPOST = 0x304
REG_THRESHOLD = 0x308
REG_PEAK_RAW = 0x30C
REG_PEAK_HEIGHT = 0x310
REG_DURATION = 0x314
REG_AREA = 0x318
REG_VALID_COUNT = 0x31C
REG_INVALID_COUNT = 0x320
REG_INVALID_REASON = 0x324
REG_TIMESTAMP_LO = 0x328
REG_TIMESTAMP_HI = 0x32C
REG_GROUP_COUNT = 0x330
REG_TARGET_ACTIVE = 0x334
REG_AREA_ERROR = 0x338
REG_P_TERM = 0x33C
REG_I_TERM = 0x340
REG_CORRECTION_PENDING = 0x344
REG_CORRECTION_ACTIVE = 0x348
REG_EFFECTIVE_HIGH = 0x34C
REG_UPDATE_COUNT = 0x350
REG_PROCESSING_LATENCY = 0x354
REG_FIFO_LEVELS = 0x358
REG_PULSE_FIFO_OVERFLOW = 0x35C
REG_GROUP_FIFO_OVERFLOW = 0x360
REG_MARKER_CYCLES = 0x364
REG_LAST_GROUP_ID = 0x368
REG_LAST_GROUP_COUNTS = 0x36C
REG_LAST_GROUP_MEAN = 0x370
REG_GPIO_ACTIVE_PERIOD = 0x374
REG_GPIO_ACTIVE_PULSE_COUNT = 0x378
REG_DAC2_ACTIVE_PERIOD = 0x37C
REG_DAC2_ACTIVE_SEGMENT_COUNT = 0x380
REG_D_TERM = 0x384
REG_EXTERNAL_TRIGGER_COUNT = 0x388
REG_EXTERNAL_TRIGGER_ACTIVE_DELAY = 0x38C

REG_PULSE_FIFO_HEAD = 0x400
PULSE_FIFO_WORDS = 10
REG_GROUP_FIFO_HEAD = 0x440
GROUP_FIFO_WORDS = 13

# CONTROL bits
CONTROL_MEASUREMENT_ENABLE = 1 << 0
CONTROL_FEEDBACK_ENABLE = 1 << 1
CONTROL_HOLD_INTEGRATOR = 1 << 2
CONTROL_MARKER_ENABLE = 1 << 3
CONTROL_GPIO_ENABLE = 1 << 4
CONTROL_DAC2_ENABLE = 1 << 5
CONTROL_AUTO_TARGET = 1 << 6
CONTROL_NEGATIVE_POLARITY = 1 << 7
CONTROL_FIFO_CONTINUOUS_LOG_ENABLE = 1 << 8

# COMMAND bits
COMMAND_CLEAR_STATE = 1 << 0
COMMAND_CLEAR_FLAGS = 1 << 1
COMMAND_GPIO_APPLY = 1 << 2
COMMAND_DAC2_APPLY = 1 << 3
COMMAND_CLEAR_FIFOS = 1 << 4
COMMAND_POP_PULSE = 1 << 5
COMMAND_POP_GROUP = 1 << 6
COMMAND_MANUAL_GROUP = 1 << 7
COMMAND_ARM_PULSE_SNAPSHOT = 1 << 8

GROUP_SOURCE_EXTERNAL = 0
GROUP_SOURCE_DAC2 = 1
GROUP_SOURCE_MANUAL = 2

KNOWN_VERSION = 0x50433035

INVALID_REASONS = {
    0x00: "none",
    0x01: "pulse_too_short",
    0x02: "pulse_too_long_or_no_falling_edge",
    0x03: "adc_saturation",
    0x04: "pulse_overlaps_post_baseline_window",
    0x05: "area_arithmetic_overflow",
    0x07: "measurement_configuration_invalid",
}

CSV_FIELDNAMES = [
    "group_id",
    "pulse_id",
    "group_start_time_s",
    "group_end_time_s",
    "pulse_measurement_time_s",
    "pulse_time_in_group_s",
    "time_from_first_pulse_s",
    "valid",
    "invalid_reason",
    "pulse_duration_s",
    "baseline_pre_v",
    "baseline_post_v",
    "baseline_for_area_v",
    "pulse_height_v",
    "peak_height_v",
    "pulse_area_v_s",
    "group_detected_pulses",
    "group_valid_pulses",
    "group_invalid_pulses",
    "group_mean_area_v_s",
    "target_area_v_s",
    "area_error_v_s",
    "normalized_area_error",
    "group_valid_for_feedback",
    "expected_pulse_mismatch",
    "group_overflow",
    "boundary_overrun",
    "deadband",
    "feedback_applied",
    "p_term_v_s",
    "i_term_v_s",
    "d_term_v_s",
    "correction_active_v",
    "effective_high_v",
    "processing_latency_s",
]


class RegisterMap(object):
    def __init__(self, base_address=BASE_ADDRESS):
        self.base_address = int(base_address)
        self.fd = None
        self.memory = None
        try:
            self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
            self.memory = mmap.mmap(
                self.fd,
                MAP_SIZE,
                mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE,
                offset=self.base_address,
            )
        except Exception:
            self.close()
            raise

    def close(self):
        if self.memory is not None:
            self.memory.close()
            self.memory = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def read_u32(self, offset):
        # /dev/mem maps an AXI-Lite MMIO register window, not normal RAM.
        # struct.unpack_from()/pack_into() was here before and result in host memory accesses that this MMIO slave does not tolerate, producing SIGBUS ("Bus error").
        self.memory.seek(int(offset))
        data = self.memory.read(4)
        if len(data) != 4:
            raise IOError("Short MMIO read at offset 0x{0:X}".format(int(offset)))
        return struct.unpack("<I", data)[0]

    def read_u32_block(self, offset, count):
        # FIFO records are also MMIO registers.  Read each aligned 32-bit word, individually; never use a wide/bulk mmap access here.
        count = int(count)
        offset = int(offset)
        if count <= 0:
            return []
        return [self.read_u32(offset + 4 * index) for index in range(count)]

    def read_s32(self, offset):
        value = self.read_u32(offset)
        if value & 0x80000000:
            return value - 0x100000000
        return value

    def write_u32(self, offset, value):
        self.memory.seek(int(offset))
        self.memory.write(struct.pack("<I", int(value) & 0xFFFFFFFF))

    def write_s32(self, offset, value):
        self.write_u32(offset, value)


def signed16(value):
    value = int(value) & 0xFFFF
    if value & 0x8000:
        return value - 0x10000
    return value


def signed32(value):
    value = int(value) & 0xFFFFFFFF
    if value & 0x80000000:
        return value - 0x100000000
    return value


def q16(value):
    return int(round(float(value) * 65536.0))


def q31_nonnegative(value):
    """Encode a non-negative Q1.31 value; legal range is [0, 1)."""
    raw = int(round(float(value) * 2147483648.0))
    if raw < 0 or raw > 0x7FFFFFFF:
        raise ValueError("Value converts outside non-negative Q1.31 range [0,1)")
    return raw


def q24(value):
    return int(round(float(value) * 16777216.0))


def fraction_q16(value):
    """Q0.16 fraction with 1.0 saturated to 0xFFFF rather than wrapping."""
    raw = int(round(float(value) * 65536.0))
    if raw < 0:
        raw = 0
    if raw > 65535:
        raw = 65535
    return raw


def is_power_of_two(value):
    value = int(value)
    return value > 0 and (value & (value - 1)) == 0


class PulseControl(object):
    def __init__(
        self,
        base_address=BASE_ADDRESS,
        adc_volts_per_count=DEFAULT_ADC_VOLTS_PER_COUNT,
        dac_counts_per_volt=DEFAULT_DAC_COUNTS_PER_VOLT,
    ):
        self.regs = RegisterMap(base_address)
        self.adc_volts_per_count = float(adc_volts_per_count)
        self.dac_counts_per_volt = float(dac_counts_per_volt)
        version = self.regs.read_u32(REG_VERSION)
        if version != KNOWN_VERSION:
            raise RuntimeError(
                "Unexpected FPGA register-map version 0x{0:08X}; expected PC05 0x{1:08X}".format(
                    version, KNOWN_VERSION
                )
            )

    def close(self):
        self.regs.close()

    def _set_control_bit(self, mask, enabled):
        value = self.regs.read_u32(REG_CONTROL)
        if enabled:
            value |= mask
        else:
            value &= ~mask
        self.regs.write_u32(REG_CONTROL, value)

    def command(self, mask):
        self.regs.write_u32(REG_COMMAND, mask)

    def clear_state(self):
        self.command(COMMAND_CLEAR_STATE)

    def clear_flags(self):
        self.command(COMMAND_CLEAR_FLAGS)

    def clear_fifos(self):
        self.command(COMMAND_CLEAR_FIFOS)

    def arm_pulse_snapshot(self):
        """Capture exactly the next completed pulse into the pulse FIFO.

        Hardware auto-disarms after the first successful push. The real-time
        measurement/group/PID path is unaffected.
        """
        self.command(COMMAND_ARM_PULSE_SNAPSHOT)

    def enable_continuous_fifo_logging(self, enabled=True):
        """Enable full-rate pulse+group FIFO logging.

        Keep this OFF for sparse snapshot mode.  Turning it ON reproduces the
        original every-record FIFO behavior used by complete-group recording.
        """
        self._set_control_bit(CONTROL_FIFO_CONTINUOUS_LOG_ENABLE, enabled)

    def continuous_fifo_logging_enabled(self):
        return bool(
            self.regs.read_u32(REG_CONTROL) & CONTROL_FIFO_CONTINUOUS_LOG_ENABLE
        )

    def enable_measurement(self, enabled=True):
        self._set_control_bit(CONTROL_MEASUREMENT_ENABLE, enabled)

    def set_group_source(self, source):
        """Select the event that starts/closes acquisition groups.

        "external" uses delayed DIO0_P rising edges.  The first delayed edge
        starts group 0; every later edge closes the current group and starts the
        next.  Use "manual" while idle/configuring so external triggers cannot
        generate records and fill the FIFOs.
        """
        names = {
            "external": GROUP_SOURCE_EXTERNAL,
            "dac2": GROUP_SOURCE_DAC2,
            "manual": GROUP_SOURCE_MANUAL,
        }
        if source not in names:
            raise ValueError("group source must be external, dac2, or manual")
        self.regs.write_u32(REG_GROUP_SOURCE, names[source])

    def get_group_source(self):
        value = self.regs.read_u32(REG_GROUP_SOURCE) & 0x3
        if value == GROUP_SOURCE_EXTERNAL:
            return "external"
        if value == GROUP_SOURCE_DAC2:
            return "dac2"
        if value == GROUP_SOURCE_MANUAL:
            return "manual"
        return "unknown"

    def configure_external_trigger_delay(self, delay_ns, expected_frequency_hz=None):
        """Configure DIO0_P trigger delay with 5 ns resolution.

        The 32-bit delay bus crosses from the 125 MHz register domain into the
        200 MHz trigger front-end.  To keep that bundled-data transfer safe, this
        method forces GROUP_SOURCE=manual, writes the shadow register, and waits
        until the synchronized active-delay readback matches before returning.

        The hardware has one pending delay counter.  When expected_frequency_hz
        is supplied, require delay < one trigger period so consecutive delayed
        edges cannot overlap.
        """
        delay_ns = float(delay_ns)
        if delay_ns < 0.0:
            raise ValueError("external trigger delay must be non-negative")
        delay_cycles = int(round(delay_ns * 1.0e-9 * TRIGGER_CLOCK_HZ))
        if delay_cycles < 0 or delay_cycles > 0xFFFFFFFF:
            raise ValueError("external trigger delay converts outside 32-bit range")
        if expected_frequency_hz is not None:
            expected_frequency_hz = float(expected_frequency_hz)
            if expected_frequency_hz <= 0.0:
                raise ValueError("expected external trigger frequency must be positive")
            period_cycles = int(round(TRIGGER_CLOCK_HZ / expected_frequency_hz))
            if period_cycles < 1:
                raise ValueError("expected external trigger period is below one 200 MHz clock")
            if delay_cycles >= period_cycles:
                raise ValueError(
                    "external trigger delay must be shorter than one trigger period; "
                    "requested {0} cycles, period about {1} cycles".format(
                        delay_cycles, period_cycles
                    )
                )

        if self.get_group_source() != "manual":
            raise RuntimeError(
                "configure_external_trigger_delay requires GROUP_SOURCE='manual'"
            )
        self.regs.write_u32(REG_EXTERNAL_TRIGGER_DELAY_CYCLES, delay_cycles)
        self._wait_until(
            lambda: self.regs.read_u32(REG_EXTERNAL_TRIGGER_ACTIVE_DELAY) == delay_cycles,
            "external trigger delay CDC/readback",
            timeout_s=0.100,
        )
        return {
            "requested_delay_ns": delay_ns,
            "delay_cycles_200mhz": delay_cycles,
            "actual_delay_ns": 5.0 * delay_cycles,
            "resolution_ns": 5.0,
        }

    def enable_feedback(self, enabled=True):
        self._set_control_bit(CONTROL_FEEDBACK_ENABLE, enabled)

    def enable_gpio(self, enabled=True):
        self._set_control_bit(CONTROL_GPIO_ENABLE, enabled)

    def enable_dac2(self, enabled=True):
        self._set_control_bit(CONTROL_DAC2_ENABLE, enabled)

    def enable_marker(self, enabled=True):
        self._set_control_bit(CONTROL_MARKER_ENABLE, enabled)

    def set_hold_integrator(self, enabled=True):
        self._set_control_bit(CONTROL_HOLD_INTEGRATOR, enabled)

    def set_outputs(self, gpio_enabled=None, dac2_enabled=None):
        """Update GPIO/DAC2 enable bits in one CONTROL register write.

        This is preferable when both outputs should start as close together as
        the two hardware clock domains allow. It is not a phase-lock command:
        DIO1_P crosses into the 200 MHz domain through synchronizers, while
        OUT2 runs in the 125 MHz ADC domain.
        """
        value = self.regs.read_u32(REG_CONTROL)
        if gpio_enabled is not None:
            if gpio_enabled:
                value |= CONTROL_GPIO_ENABLE
            else:
                value &= ~CONTROL_GPIO_ENABLE
        if dac2_enabled is not None:
            if dac2_enabled:
                value |= CONTROL_DAC2_ENABLE
            else:
                value &= ~CONTROL_DAC2_ENABLE
        self.regs.write_u32(REG_CONTROL, value)

    def _wait_until(self, predicate, description, timeout_s=0.100, poll_interval_s=0.0002):
        deadline = time.monotonic() + float(timeout_s)
        last_value = None
        while time.monotonic() < deadline:
            last_value = predicate()
            if last_value:
                return last_value
            time.sleep(float(poll_interval_s))
        raise RuntimeError(
            "Timed out waiting for {0}; last check={1}".format(description, last_value)
        )

    def _pause_feedback(self):
        was_enabled = bool(self.regs.read_u32(REG_CONTROL) & CONTROL_FEEDBACK_ENABLE)
        if was_enabled:
            self.enable_feedback(False)
        return was_enabled

    def adc_counts_from_volts(self, volts):
        return int(round(float(volts) / self.adc_volts_per_count))

    def dac_counts_from_volts(self, volts):
        value = int(round(float(volts) * self.dac_counts_per_volt))
        if value > 8191:
            value = 8191
        if value < -8192:
            value = -8192
        return value

    def area_counts_from_v_us(self, area_v_us):
        scale = self.adc_volts_per_count * ADC_SAMPLE_PERIOD_US
        return int(round(float(area_v_us) / scale))

    def area_v_us_from_counts(self, area_counts):
        return float(area_counts) * self.adc_volts_per_count * ADC_SAMPLE_PERIOD_US

    def dac_volts_from_counts(self, counts):
        return float(counts) / self.dac_counts_per_volt

    def configure_gpio(self, frequency_hz, pulses, enable=None, apply=True):
        """
        Configure DIO1_P pulse windows.  For the resource-optimized FPGA
        implementation the multi-bit window table is a bundled-data CDC, so
        this method always stops GPIO before rewriting it, applies the complete
        table, then restores the requested enable state.

        pulses: list of (start_fraction, end_fraction), e.g.
            [(0.0, 0.10), (0.20, 0.50)]
        """
        frequency_hz = float(frequency_hz)
        if frequency_hz <= 0.0:
            raise ValueError("GPIO frequency must be positive")
        if len(pulses) > MAX_GPIO_PULSES:
            raise ValueError("At most {0} GPIO pulses per period".format(MAX_GPIO_PULSES))

        period_cycles = int(round(GPIO_CLOCK_HZ / frequency_hz))
        if period_cycles < 2:
            raise ValueError("GPIO period must be at least 2 clocks (10 ns)")

        windows = []
        last_end = 0
        for start_fraction, end_fraction in pulses:
            start_fraction = float(start_fraction)
            end_fraction = float(end_fraction)
            if not (0.0 <= start_fraction < end_fraction <= 1.0):
                raise ValueError("Each GPIO pulse must satisfy 0 <= start < end <= 1")
            start_cycle = int(round(start_fraction * period_cycles))
            end_cycle = int(round(end_fraction * period_cycles))
            if end_cycle > period_cycles:
                end_cycle = period_cycles
            if start_cycle < last_end:
                raise ValueError("GPIO pulse windows must be sorted and non-overlapping")
            if end_cycle <= start_cycle:
                raise ValueError("A requested GPIO pulse rounded to zero clocks")
            if (end_cycle - start_cycle) < 2:
                raise ValueError(
                    "Each GPIO HIGH window must be at least 2 clocks (10 ns at 200 MHz) "
                    "for the supported fast GPIO sequencer configuration"
                )
            windows.append((start_cycle, end_cycle))
            last_end = end_cycle

        control_before = self.regs.read_u32(REG_CONTROL)
        was_enabled = bool(control_before & CONTROL_GPIO_ENABLE)
        # Reconfiguring the experimental GPIO pattern can change the optical
        # plant even though grouping now comes from DIO0_P, so pause feedback.
        feedback_was_enabled = self._pause_feedback()
        if was_enabled:
            self.enable_gpio(False)
            time.sleep(0.001)

        self.regs.write_u32(REG_GPIO_PERIOD, period_cycles)
        self.regs.write_u32(REG_GPIO_PULSE_COUNT, len(windows))
        for index in range(MAX_GPIO_PULSES):
            if index < len(windows):
                start_cycle, end_cycle = windows[index]
            else:
                start_cycle, end_cycle = 0, 0
            base = REG_GPIO_WINDOWS_BASE + index * 8
            self.regs.write_u32(base, start_cycle)
            self.regs.write_u32(base + 4, end_cycle)

        if apply:
            self.command(COMMAND_GPIO_APPLY)
            def gpio_ready():
                status = self.read_status()
                return (
                    status["gpio_config_valid"]
                    and self.regs.read_u32(REG_GPIO_ACTIVE_PERIOD) == period_cycles
                    and (self.regs.read_u32(REG_GPIO_ACTIVE_PULSE_COUNT) & 0x1F) == len(windows)
                )
            self._wait_until(gpio_ready, "GPIO APPLY/readback")

        final_enable = was_enabled if enable is None else bool(enable)
        self.enable_gpio(final_enable)
        if feedback_was_enabled and final_enable:
            self.enable_feedback(True)

        return {
            "requested_frequency_hz": frequency_hz,
            "actual_frequency_hz": GPIO_CLOCK_HZ / period_cycles,
            "period_cycles": period_cycles,
            "windows_cycles": windows,
            "resolution_ns": 5.0,
        }

    def configure_dac2(self, frequency_hz, low_voltage_v, high_voltage_v, sequence, enable=None, apply=True):
        """
        Configure OUT2 segments.  The resource-optimized FPGA does not duplicate
        the whole segment table, so this method stops OUT2 before table writes,
        applies the complete configuration, then restores the requested state.

        sequence: [(normalized_level, fraction_of_period), ...]
        normalized_level is in [0,1].  Fractions must sum to 1.
        """
        frequency_hz = float(frequency_hz)
        if frequency_hz <= 0.0:
            raise ValueError("DAC2 frequency must be positive")
        if len(sequence) < 1 or len(sequence) > MAX_DAC_SEGMENTS:
            raise ValueError("DAC2 sequence must contain 1..{0} segments".format(MAX_DAC_SEGMENTS))

        total_fraction = sum(float(item[1]) for item in sequence)
        if abs(total_fraction - 1.0) > 0.000001:
            raise ValueError("DAC2 segment fractions must sum to 1.0")

        period_cycles = int(round(ADC_CLOCK_HZ / frequency_hz))
        if period_cycles < len(sequence):
            raise ValueError("DAC2 period is too short for the requested segment count")

        low_counts = self.dac_counts_from_volts(low_voltage_v)
        high_counts = self.dac_counts_from_volts(high_voltage_v)
        if high_counts <= low_counts:
            raise ValueError("DAC2 high voltage must exceed low voltage")

        durations = []
        used = 0
        for index, item in enumerate(sequence):
            level = float(item[0])
            fraction = float(item[1])
            if not 0.0 <= level <= 1.0:
                raise ValueError("DAC2 normalized levels must lie in [0,1]")
            if fraction <= 0.0:
                raise ValueError("DAC2 segment durations must be positive")
            if index == len(sequence) - 1:
                duration = period_cycles - used
            else:
                duration = int(round(fraction * period_cycles))
            if duration < 1:
                raise ValueError("A DAC2 segment rounded to zero clocks")
            durations.append(duration)
            used += duration

        if sum(durations) != period_cycles:
            durations[-1] += period_cycles - sum(durations)

        control_before = self.regs.read_u32(REG_CONTROL)
        was_enabled = bool(control_before & CONTROL_DAC2_ENABLE)
        feedback_was_enabled = self._pause_feedback()
        if was_enabled:
            self.enable_dac2(False)
            time.sleep(0.001)

        self.regs.write_u32(REG_DAC2_PERIOD, period_cycles)
        self.regs.write_u32(REG_DAC2_SEGMENT_COUNT, len(sequence))
        self.regs.write_s32(REG_DAC2_LOW, low_counts)
        self.regs.write_s32(REG_DAC2_HIGH, high_counts)

        for index in range(MAX_DAC_SEGMENTS):
            base = REG_DAC2_SEGMENTS_BASE + index * 8
            if index < len(sequence):
                level = fraction_q16(sequence[index][0])
                duration = durations[index]
            else:
                level = 0
                duration = 1
            self.regs.write_u32(base, level)
            self.regs.write_u32(base + 4, duration)

        if apply:
            self.command(COMMAND_DAC2_APPLY)
            def dac2_ready():
                status = self.read_status()
                return (
                    status["dac2_config_valid"]
                    and self.regs.read_u32(REG_DAC2_ACTIVE_PERIOD) == period_cycles
                    and (self.regs.read_u32(REG_DAC2_ACTIVE_SEGMENT_COUNT) & 0x1F) == len(sequence)
                )
            self._wait_until(dac2_ready, "DAC2 APPLY/table build/readback")

        final_enable = was_enabled if enable is None else bool(enable)
        self.enable_dac2(final_enable)
        if feedback_was_enabled and final_enable:
            self.enable_feedback(True)

        return {
            "requested_frequency_hz": frequency_hz,
            "actual_frequency_hz": ADC_CLOCK_HZ / period_cycles,
            "period_cycles": period_cycles,
            "durations_cycles": durations,
            "resolution_ns": 8.0,
            "low_counts": low_counts,
            "high_counts": high_counts,
        }

    def configure_measurement(
        self,
        trigger_fraction=0.25,
        target_height_v=0.1,
        pre_baseline_samples=32,
        post_baseline_samples=32,
        min_pulse_samples=3,
        max_pulse_samples=16384,
        adc_saturation_limit_v=0.999,
        pulse_polarity="positive",
        expected_pulses_per_group=0,
        min_valid_pulses_per_group=1,
        group_source="external",
        external_trigger_delay_ns=0.0,
        external_trigger_frequency_hz=None,
        enable=None,
    ):
        was_enabled = bool(self.regs.read_u32(REG_CONTROL) & CONTROL_MEASUREMENT_ENABLE)
        feedback_was_enabled = self._pause_feedback()
        # Avoid evaluating a half-written detector configuration.
        self.enable_measurement(False)
        if not 0.0 < float(trigger_fraction) < 1.0:
            raise ValueError("trigger_fraction must be between 0 and 1")
        if not is_power_of_two(pre_baseline_samples) or int(pre_baseline_samples) > 256:
            raise ValueError("pre_baseline_samples must be a power of two <= 256")
        if not is_power_of_two(post_baseline_samples) or int(post_baseline_samples) > 256:
            raise ValueError("post_baseline_samples must be a power of two <= 256")
        if int(min_pulse_samples) < 3:
            raise ValueError("min_pulse_samples must be at least 3")
        if int(max_pulse_samples) < int(min_pulse_samples):
            raise ValueError("max_pulse_samples must be >= min_pulse_samples")
        if not 0 <= int(expected_pulses_per_group) <= 32:
            raise ValueError("expected_pulses_per_group must be 0..32")
        if not 1 <= int(min_valid_pulses_per_group) <= 32:
            raise ValueError("min_valid_pulses_per_group must be 1..32")

        source_names = {
            "external": GROUP_SOURCE_EXTERNAL,
            "dac2": GROUP_SOURCE_DAC2,
            "manual": GROUP_SOURCE_MANUAL,
        }
        if group_source not in source_names:
            raise ValueError("group_source must be external, dac2, or manual")

        target_height_counts = self.adc_counts_from_volts(abs(float(target_height_v)))
        saturation_counts = self.adc_counts_from_volts(abs(float(adc_saturation_limit_v)))
        if target_height_counts < 1 or target_height_counts > 65535:
            raise ValueError("target_height_v converts outside the supported count range")
        if saturation_counts < 1:
            saturation_counts = 1
        if saturation_counts > 8191:
            saturation_counts = 8191

        self.regs.write_u32(REG_TRIGGER_FRACTION_Q16, fraction_q16(trigger_fraction))
        self.regs.write_u32(REG_TARGET_HEIGHT, target_height_counts)
        self.regs.write_u32(REG_PRE_BASELINE, int(pre_baseline_samples))
        self.regs.write_u32(REG_POST_BASELINE, int(post_baseline_samples))
        self.regs.write_u32(REG_MIN_PULSE_SAMPLES, int(min_pulse_samples))
        self.regs.write_u32(REG_MAX_PULSE_SAMPLES, int(max_pulse_samples))
        self.regs.write_u32(REG_ADC_SATURATION_LIMIT, saturation_counts)
        self.regs.write_u32(REG_EXPECTED_PULSES, int(expected_pulses_per_group))
        self.regs.write_u32(REG_MIN_VALID_PULSES, int(min_valid_pulses_per_group))

        # Configure the external-trigger CDC only while the group source is
        # manual.  This also prevents a trigger from creating a group while the
        # detector configuration is half-written.
        self.set_group_source("manual")
        self.configure_external_trigger_delay(
            external_trigger_delay_ns,
            expected_frequency_hz=external_trigger_frequency_hz,
        )
        self.regs.write_u32(REG_GROUP_SOURCE, source_names[group_source])

        negative = str(pulse_polarity).lower() in ("negative", "neg", "-")
        self._set_control_bit(CONTROL_NEGATIVE_POLARITY, negative)
        # Baseline/window changes need a clean re-prime before measurement resumes.
        self.clear_state()
        final_measurement_enable = was_enabled if enable is None else bool(enable)
        self.enable_measurement(final_measurement_enable)
        if final_measurement_enable:
            self._wait_until(
                lambda: self.read_status()["measurement_config_valid"],
                "measurement configuration valid",
            )
        if feedback_was_enabled:
            self.enable_feedback(True)

    def _group_interval_s(self, controller_dt_s=None):
        source = self.regs.read_u32(REG_GROUP_SOURCE) & 0x3
        if source == GROUP_SOURCE_DAC2:
            cycles = self.regs.read_u32(REG_DAC2_PERIOD)
            return float(cycles) / ADC_CLOCK_HZ
        # DIO0_P is external, so the FPGA register map cannot infer its period.
        # The caller must state the expected valid-group interval for Ki/Kd
        # discretization.  Manual source has the same requirement.
        if controller_dt_s is None:
            raise ValueError(
                "controller_dt_s is required for external/manual group source"
            )
        return float(controller_dt_s)

    def configure_feedback(
        self,
        target_mean_pulse_area_v_us=None,
        use_first_valid_group_as_target=True,
        area_error_deadband_v_us=None,
        area_error_deadband_fraction=0.005,
        kp=0.20,
        ki_per_s=0.05,
        kd=0.0,
        area_to_dac_gain_v_per_v_us=None,
        dac_correction_sign=-1.0,
        max_dac_step_per_update_v=0.020,
        dac_high_voltage_min_v=0.010,
        dac_high_voltage_max_v=1.000,
        max_integral_area_term_fraction=0.25,
        controller_dt_s=None,
        enable=None,
    ):
        was_enabled = bool(self.regs.read_u32(REG_CONTROL) & CONTROL_FEEDBACK_ENABLE)
        # Match the safety behaviour of the old PA01 tool: never rewrite PI
        # coefficients while the controller is actively updating.
        self.enable_feedback(False)
        auto_target = bool(use_first_valid_group_as_target)
        if target_mean_pulse_area_v_us is None and not auto_target:
            raise ValueError("A target area is required when auto target is disabled")
        if area_to_dac_gain_v_per_v_us is None:
            raise ValueError("AREA_TO_DAC_GAIN_V_PER_V_US must be provided/calibrated")
        if float(kp) < 0.0 or float(ki_per_s) < 0.0 or float(kd) < 0.0:
            raise ValueError("KP, KI_PER_S, and KD must be non-negative")
        if not 0.0 <= float(area_error_deadband_fraction) < 1.0:
            raise ValueError("deadband fraction must lie in [0,1)")
        if not 0.0 < float(max_integral_area_term_fraction) <= 1.0:
            raise ValueError("max_integral_area_term_fraction must lie in (0,1]")

        if target_mean_pulse_area_v_us is None:
            target_area = 0
        else:
            target_area = self.area_counts_from_v_us(target_mean_pulse_area_v_us)

        if area_error_deadband_v_us is None:
            deadband_abs = 0
            deadband_mode = 1
        else:
            deadband_abs = abs(self.area_counts_from_v_us(area_error_deadband_v_us))
            deadband_mode = 0

        dt_s = self._group_interval_s(controller_dt_s)
        if dt_s <= 0.0:
            raise ValueError("group/controller interval must be positive")
        ki_per_update = float(ki_per_s) * dt_s
        kp_q16 = q16(kp)
        ki_q31 = q31_nonnegative(ki_per_update)
        if kp_q16 < 0 or kp_q16 > 0x7FFFFFFF:
            raise ValueError("KP converts outside positive Q16.16 range")
        if float(ki_per_s) > 0.0 and ki_q31 == 0:
            min_ki_per_s = 1.0 / (2147483648.0 * dt_s)
            raise ValueError(
                "KI_PER_S={0:g} is below the FPGA Q1.31 resolution for group_dt={1:g}s; "
                "minimum non-zero KI_PER_S is about {2:g} 1/s.".format(
                    float(ki_per_s), dt_s, min_ki_per_s
                )
            )
        if float(ki_per_s) > 0.0:
            effective_ki_per_s = (float(ki_q31) / 2147483648.0) / dt_s
            relative_ki_error = abs(effective_ki_per_s - float(ki_per_s)) / float(ki_per_s)
            if relative_ki_error > 0.01:
                warnings.warn(
                    "KI_PER_S quantizes from {0:g} to {1:g} 1/s at group_dt={2:g}s "
                    "with hardware Q1.31 per-update Ki.".format(
                        float(ki_per_s), effective_ki_per_s, dt_s
                    )
                )
        # Continuous-form derivative term: KD * de/dt.  KD therefore has units
        # of seconds. Hardware sees consecutive group errors, so its discrete
        # multiplier is KD / dt.
        kd_per_update = float(kd) / dt_s if float(kd) != 0.0 else 0.0
        kd_q16 = q16(kd_per_update)
        if kd_q16 < 0 or kd_q16 > 0x7FFFFFFF:
            raise ValueError("KD / group_dt converts outside positive Q16.16 range")

        # Physical conversion:
        # internal area count-sample -> V*us is ADC_V_PER_COUNT * 0.008.
        # physical gain then gives DAC volts; finally convert volts -> DAC counts.
        internal_gain = (
            float(area_to_dac_gain_v_per_v_us)
            * self.adc_volts_per_count
            * ADC_SAMPLE_PERIOD_US
            * self.dac_counts_per_volt
        )

        gain_q24 = q24(internal_gain)
        if gain_q24 <= 0 or gain_q24 > 0x7FFFFFFF:
            raise ValueError("AREA_TO_DAC_GAIN converts outside Q8.24 range")

        max_step_counts = abs(self.dac_counts_from_volts(max_dac_step_per_update_v))
        high_min_counts = self.dac_counts_from_volts(dac_high_voltage_min_v)
        high_max_counts = self.dac_counts_from_volts(dac_high_voltage_max_v)
        if high_min_counts >= high_max_counts:
            raise ValueError("DAC high minimum must be below maximum")

        self.regs.write_s32(REG_TARGET_AREA, target_area)
        self.regs.write_u32(REG_DEADBAND_ABS, deadband_abs)
        self.regs.write_u32(REG_DEADBAND_FRACTION_Q16, fraction_q16(area_error_deadband_fraction))
        self.regs.write_u32(REG_DEADBAND_MODE, deadband_mode)
        self.regs.write_u32(REG_KP_Q16, kp_q16)
        self.regs.write_u32(REG_KI_Q31_PER_UPDATE, ki_q31)
        self.regs.write_u32(REG_KD_Q16_PER_UPDATE, kd_q16)
        self.regs.write_u32(REG_AREA_TO_DAC_GAIN_Q24, gain_q24)
        self.regs.write_u32(REG_CORRECTION_SIGN, 1 if float(dac_correction_sign) < 0.0 else 0)
        self.regs.write_u32(REG_MAX_DAC_STEP, max_step_counts)
        self.regs.write_s32(REG_DAC_HIGH_MIN, high_min_counts)
        self.regs.write_s32(REG_DAC_HIGH_MAX, high_max_counts)

        # Fractional integral limiting is done in hardware after the active
        # target is known, so it also works with first-valid-group auto target.
        self.regs.write_u32(
            REG_INTEGRAL_LIMIT_FRACTION_Q16,
            fraction_q16(max_integral_area_term_fraction),
        )
        self.regs.write_u32(REG_INTEGRAL_FRACTION_MODE, 1)

        self._set_control_bit(CONTROL_AUTO_TARGET, auto_target)
        self.clear_state()
        self.enable_feedback(was_enabled if enable is None else bool(enable))

        return {
            "group_dt_s": dt_s,
            "kp_q16": kp_q16,
            "kp_effective": float(kp_q16) / 65536.0,
            "ki_per_update": ki_per_update,
            "ki_q31_per_update": ki_q31,
            "ki_per_s_effective": (float(ki_q31) / 2147483648.0) / dt_s,
            "kd": float(kd),
            "kd_per_update": kd_per_update,
            "kd_q16_per_update": kd_q16,
            "area_to_dac_gain_internal": internal_gain,
            "area_to_dac_gain_q24": gain_q24,
        }

    def manual_group_boundary(self):
        self.command(COMMAND_MANUAL_GROUP)

    def read_status(self):
        status = self.regs.read_u32(REG_STATUS)
        return {
            "raw": status,
            "measurement_enabled": bool(status & (1 << 0)),
            "feedback_enabled": bool(status & (1 << 1)),
            "gpio_enabled": bool(status & (1 << 2)),
            "dac2_enabled": bool(status & (1 << 3)),
            "marker_enabled": bool(status & (1 << 4)),
            "measurement_config_valid": bool(status & (1 << 5)),
            "controller_config_valid": bool(status & (1 << 6)),
            "gpio_config_valid": bool(status & (1 << 7)),
            "dac2_config_valid": bool(status & (1 << 8)),
            "target_latched": bool(status & (1 << 9)),
            "pulse_fifo_nonempty": bool(status & (1 << 10)),
            "group_fifo_nonempty": bool(status & (1 << 11)),
            "pulse_fifo_full": bool(status & (1 << 12)),
            "group_fifo_full": bool(status & (1 << 13)),
            "pulse_fifo_overflow_seen": bool(status & (1 << 14)),
            "group_fifo_overflow_seen": bool(status & (1 << 15)),
            "invalid_seen": bool(status & (1 << 16)),
            "correction_saturation_seen": bool(status & (1 << 17)),
            "integral_saturation_seen": bool(status & (1 << 18)),
            "marker_timeout_seen": bool(status & (1 << 19)),
            "group_mismatch_seen": bool(status & (1 << 20)),
            "group_overflow_seen": bool(status & (1 << 21)),
            "boundary_overrun_seen": bool(status & (1 << 22)),
            "correction_active_nonzero": bool(status & (1 << 23)),
            "external_trigger_overrun_seen": bool(status & (1 << 24)),
        }

    def read_snapshot(self):
        fifo_levels = self.regs.read_u32(REG_FIFO_LEVELS)
        return {
            "status": self.read_status(),
            "bpre": self.regs.read_s32(REG_BPRE),
            "bpost": self.regs.read_s32(REG_BPOST),
            "threshold": self.regs.read_s32(REG_THRESHOLD),
            "peak_raw": self.regs.read_s32(REG_PEAK_RAW),
            "peak_height": self.regs.read_u32(REG_PEAK_HEIGHT) & 0xFFFF,
            "duration_samples": self.regs.read_u32(REG_DURATION),
            "area_counts_samples": self.regs.read_s32(REG_AREA),
            "valid_count": self.regs.read_u32(REG_VALID_COUNT),
            "invalid_count": self.regs.read_u32(REG_INVALID_COUNT),
            "last_invalid_reason": INVALID_REASONS.get(
                self.regs.read_u32(REG_INVALID_REASON) & 0xFF, "unknown"
            ),
            "group_count": self.regs.read_u32(REG_GROUP_COUNT),
            "target_area_counts_samples": self.regs.read_s32(REG_TARGET_ACTIVE),
            "area_error_counts_samples": self.regs.read_s32(REG_AREA_ERROR),
            "p_area": self.regs.read_s32(REG_P_TERM),
            "i_area": self.regs.read_s32(REG_I_TERM),
            "d_area": self.regs.read_s32(REG_D_TERM),
            "correction_pending_counts": self.regs.read_s32(REG_CORRECTION_PENDING),
            "correction_active_counts": self.regs.read_s32(REG_CORRECTION_ACTIVE),
            "effective_high_counts": self.regs.read_s32(REG_EFFECTIVE_HIGH),
            "update_count": self.regs.read_u32(REG_UPDATE_COUNT),
            "processing_latency_cycles": self.regs.read_u32(REG_PROCESSING_LATENCY),
            "pulse_fifo_level": fifo_levels & 0xFFFF,
            "group_fifo_level": (fifo_levels >> 16) & 0xFFFF,
            "pulse_fifo_overflow": self.regs.read_u32(REG_PULSE_FIFO_OVERFLOW),
            "group_fifo_overflow": self.regs.read_u32(REG_GROUP_FIFO_OVERFLOW),
            "last_marker_cycles": self.regs.read_u32(REG_MARKER_CYCLES),
            "external_trigger_count": self.regs.read_u32(REG_EXTERNAL_TRIGGER_COUNT),
            "external_trigger_active_delay_cycles": self.regs.read_u32(REG_EXTERNAL_TRIGGER_ACTIVE_DELAY),
        }

    def sanity_check(self, require_measurement=False, require_gpio=False, require_dac2=False, require_feedback=False):
        status = self.read_status()
        problems = []
        if require_measurement and not status["measurement_config_valid"]:
            problems.append("measurement_config_valid=0")
        if require_gpio and not status["gpio_config_valid"]:
            problems.append("gpio_config_valid=0")
        if require_dac2 and not status["dac2_config_valid"]:
            problems.append("dac2_config_valid=0")
        if require_feedback and not status["controller_config_valid"]:
            problems.append("controller_config_valid=0")
        if status["pulse_fifo_overflow_seen"] or status["group_fifo_overflow_seen"]:
            problems.append("FIFO overflow has occurred")
        if status["external_trigger_overrun_seen"]:
            problems.append("external trigger delay overrun has occurred")
        if problems:
            raise RuntimeError("PC05 sanity check failed: " + "; ".join(problems))
        return status

    def _read_words(self, base, count):
        # FIFO head words are a contiguous read-only window and remain stable
        # until the explicit POP command, so read the whole record in one mmap
        # unpack instead of one Python mmap operation per 32-bit word.
        return self.regs.read_u32_block(base, count)

    def read_pulse_record(self, pop=True):
        levels = self.regs.read_u32(REG_FIFO_LEVELS)
        if (levels & 0xFFFF) == 0:
            return None
        words = self._read_words(REG_PULSE_FIFO_HEAD, PULSE_FIFO_WORDS)
        timestamp = words[0] | (words[1] << 32)
        group_id = words[2]
        pulse_id = (words[3] >> 16) & 0xFFFF
        reason = (words[3] >> 8) & 0xFF
        flags = words[3] & 0xFF
        bpre = signed16((words[4] >> 16) & 0xFFFF)
        bpost = signed16(words[4] & 0xFFFF)
        threshold = signed16((words[5] >> 16) & 0xFFFF)
        peak_raw = signed16(words[5] & 0xFFFF)
        duration = words[6]
        area = signed32(words[7])
        correction = signed16((words[8] >> 16) & 0xFFFF)
        effective_high = signed16(words[8] & 0xFFFF)
        peak_height = (words[9] >> 16) & 0xFFFF
        if pop:
            self.command(COMMAND_POP_PULSE)

        duration_s = float(duration) / ADC_CLOCK_HZ
        area_v_s = self.area_v_us_from_counts(area) * 1.0e-6
        valid = bool(flags & 0x01)
        pulse_height_v = None
        if valid and duration_s > 0.0:
            # Effective mean baseline-subtracted pulse height.  This is the
            # single height which obeys area = pulse_height * duration.
            pulse_height_v = area_v_s / duration_s

        return {
            "timestamp_cycles": timestamp,
            "time_s": float(timestamp) / ADC_CLOCK_HZ,
            "group_id": group_id,
            "pulse_id": pulse_id,
            "valid": valid,
            "invalid": bool(flags & 0x02),
            "invalid_reason": INVALID_REASONS.get(reason, "unknown"),
            "bpre_counts": bpre,
            "bpost_counts": bpost,
            "bpre_v": float(bpre) * self.adc_volts_per_count,
            "bpost_v": float(bpost) * self.adc_volts_per_count,
            "baseline_for_area_v": 0.5 * float(bpre + bpost) * self.adc_volts_per_count,
            "threshold_counts": threshold,
            "peak_raw_counts": peak_raw,
            "peak_height_counts": peak_height,
            "peak_height_v": float(peak_height) * self.adc_volts_per_count,
            "duration_samples": duration,
            "duration_s": duration_s,
            "area_counts_samples": area,
            "area_v_us": self.area_v_us_from_counts(area),
            "area_v_s": area_v_s,
            "pulse_height_v": pulse_height_v,
            "correction_active_counts": correction,
            "correction_active_v": self.dac_volts_from_counts(correction),
            "effective_high_counts": effective_high,
            "effective_high_v": self.dac_volts_from_counts(effective_high),
        }

    def read_group_record(self, pop=True):
        levels = self.regs.read_u32(REG_FIFO_LEVELS)
        if ((levels >> 16) & 0xFFFF) == 0:
            return None
        words = self._read_words(REG_GROUP_FIFO_HEAD, GROUP_FIFO_WORDS)
        timestamp = words[0] | (words[1] << 32)
        flags = (words[3] >> 24) & 0xFF
        detected = words[3] & 0xFF
        valid = (words[3] >> 8) & 0xFF
        invalid = (words[3] >> 16) & 0xFF
        mean_area = signed32(words[4])
        target_area = signed32(words[5])
        area_error = signed32(words[6])
        p_area = signed32(words[7])
        i_area = signed32(words[8])
        d_area = signed32(words[9])
        correction_pending = signed16((words[10] >> 16) & 0xFFFF)
        correction_active = signed16(words[10] & 0xFFFF)
        nominal_high = signed16((words[11] >> 16) & 0xFFFF)
        effective_high = signed16(words[11] & 0xFFFF)
        latency_cycles = words[12]
        if pop:
            self.command(COMMAND_POP_GROUP)
        return {
            "timestamp_cycles": timestamp,
            "time_s": float(timestamp) / ADC_CLOCK_HZ,
            "group_id": words[2],
            "detected_pulses": detected,
            "valid_pulses": valid,
            "invalid_pulses": invalid,
            "group_valid_for_feedback": bool(flags & 0x01),
            "expected_mismatch": bool(flags & 0x02),
            "group_overflow": bool(flags & 0x04),
            "boundary_overrun": bool(flags & 0x08),
            "deadband": bool(flags & 0x10),
            "feedback_applied": bool(flags & 0x20),
            "mean_area_counts_samples": mean_area,
            "mean_area_v_us": self.area_v_us_from_counts(mean_area),
            "mean_area_v_s": self.area_v_us_from_counts(mean_area) * 1.0e-6,
            "target_area_counts_samples": target_area,
            "target_area_v_us": self.area_v_us_from_counts(target_area),
            "target_area_v_s": self.area_v_us_from_counts(target_area) * 1.0e-6,
            "area_error_counts_samples": area_error,
            "area_error_v_us": self.area_v_us_from_counts(area_error),
            "area_error_v_s": self.area_v_us_from_counts(area_error) * 1.0e-6,
            "p_area": p_area,
            "i_area": i_area,
            "d_area": d_area,
            "p_term_v_s": self.area_v_us_from_counts(p_area) * 1.0e-6,
            "i_term_v_s": self.area_v_us_from_counts(i_area) * 1.0e-6,
            "d_term_v_s": self.area_v_us_from_counts(d_area) * 1.0e-6,
            "correction_pending_counts": correction_pending,
            "correction_active_counts": correction_active,
            "correction_active_v": self.dac_volts_from_counts(correction_active),
            "nominal_high_counts": nominal_high,
            "nominal_high_v": self.dac_volts_from_counts(nominal_high),
            "effective_high_counts": effective_high,
            "effective_high_v": self.dac_volts_from_counts(effective_high),
            "processing_latency_cycles": latency_cycles,
            "processing_latency_ns": float(latency_cycles) * 8.0,
            "processing_latency_s": float(latency_cycles) / ADC_CLOCK_HZ,
        }

    def _combined_csv_rows(self, group, pulses, group_start_cycles, first_pulse_cycles):
        target = group["target_area_v_s"]
        if target != 0.0:
            normalized_error = group["area_error_v_s"] / target
        else:
            normalized_error = None

        group_start_time_s = ""
        if group_start_cycles is not None:
            group_start_time_s = float(group_start_cycles) / ADC_CLOCK_HZ

        common = {
            "group_id": group["group_id"],
            "group_start_time_s": group_start_time_s,
            "group_end_time_s": group["time_s"],
            "group_detected_pulses": group["detected_pulses"],
            "group_valid_pulses": group["valid_pulses"],
            "group_invalid_pulses": group["invalid_pulses"],
            "group_mean_area_v_s": group["mean_area_v_s"],
            "target_area_v_s": group["target_area_v_s"],
            "area_error_v_s": group["area_error_v_s"],
            "normalized_area_error": normalized_error,
            "group_valid_for_feedback": int(group["group_valid_for_feedback"]),
            "expected_pulse_mismatch": int(group["expected_mismatch"]),
            "group_overflow": int(group["group_overflow"]),
            "boundary_overrun": int(group["boundary_overrun"]),
            "deadband": int(group["deadband"]),
            "feedback_applied": int(group["feedback_applied"]),
            "p_term_v_s": group["p_term_v_s"],
            "i_term_v_s": group["i_term_v_s"],
            "d_term_v_s": group["d_term_v_s"],
            "processing_latency_s": group["processing_latency_s"],
        }

        # Preserve a zero-pulse group as one row with blank pulse fields so
        # missing-pulse groups are not silently absent from the CSV.
        if not pulses:
            row = dict((name, "") for name in CSV_FIELDNAMES)
            row.update(common)
            row["valid"] = 0
            row["invalid_reason"] = "no_pulse_detected"
            row["correction_active_v"] = group["correction_active_v"]
            row["effective_high_v"] = group["effective_high_v"]
            return [row], first_pulse_cycles

        rows = []
        for pulse in sorted(pulses, key=lambda item: item["pulse_id"]):
            if first_pulse_cycles is None:
                first_pulse_cycles = pulse["timestamp_cycles"]

            pulse_time_in_group_s = ""
            if group_start_cycles is not None:
                pulse_time_in_group_s = (
                    float(pulse["timestamp_cycles"] - group_start_cycles) / ADC_CLOCK_HZ
                )

            row = dict((name, "") for name in CSV_FIELDNAMES)
            row.update(common)
            row.update({
                "pulse_id": pulse["pulse_id"],
                "pulse_measurement_time_s": pulse["time_s"],
                "pulse_time_in_group_s": pulse_time_in_group_s,
                "time_from_first_pulse_s": (
                    float(pulse["timestamp_cycles"] - first_pulse_cycles) / ADC_CLOCK_HZ
                ),
                "valid": int(pulse["valid"]),
                "invalid_reason": pulse["invalid_reason"],
                "pulse_duration_s": pulse["duration_s"],
                "baseline_pre_v": pulse["bpre_v"],
                "baseline_post_v": pulse["bpost_v"],
                "baseline_for_area_v": pulse["baseline_for_area_v"],
                "pulse_height_v": "" if pulse["pulse_height_v"] is None else pulse["pulse_height_v"],
                "peak_height_v": pulse["peak_height_v"],
                "pulse_area_v_s": pulse["area_v_s"],
                # Use the correction that was active when this pulse was measured.
                "correction_active_v": pulse["correction_active_v"],
                "effective_high_v": pulse["effective_high_v"],
            })
            rows.append(row)

        return rows, first_pulse_cycles

    def record_grouped_csv(
        self,
        csv_path="pulse_control_data.csv",
        duration_s=0.0,
        write_interval_s=0.1,
        fifo_poll_interval_s=0.001,
        session_start=None,
        session_stop=None,
        progress_interval_s=1.0,
        max_pulse_records_per_pass=128,
        max_group_records_per_pass=64,
    ):
        """Record one CSV while preserving complete acquisition groups.

        FPGA measurement and feedback continue at full hardware rate.  Python
        drains both hardware FIFOs continuously into RAM.  write_interval_s is
        only the disk batching/flush interval; it does NOT downsample pulses or
        groups.  A group is written only after its group record and all of its
        pulse records are present.

        The method raises if either FPGA FIFO overflows because, in that case,
        complete-group logging can no longer be guaranteed.
        """
        write_interval_s = float(write_interval_s)
        fifo_poll_interval_s = float(fifo_poll_interval_s)
        duration_s = float(duration_s)
        if write_interval_s <= 0.0:
            raise ValueError("write_interval_s must be positive")
        if fifo_poll_interval_s <= 0.0:
            raise ValueError("fifo_poll_interval_s must be positive")
        progress_interval_s = float(progress_interval_s)
        max_pulse_records_per_pass = int(max_pulse_records_per_pass)
        max_group_records_per_pass = int(max_group_records_per_pass)
        if progress_interval_s < 0.0:
            raise ValueError("progress_interval_s must be non-negative")
        if max_pulse_records_per_pass <= 0 or max_group_records_per_pass <= 0:
            raise ValueError("FIFO batch limits must be positive")

        pending_pulses = {}
        pending_groups = {}
        rows_waiting_for_disk = []
        first_pulse_cycles = None
        previous_group_end_cycles = None
        last_completed_group_id = None
        start_monotonic = time.monotonic()
        last_disk_write = start_monotonic
        last_progress_print = start_monotonic
        groups_written = 0
        rows_written = 0
        session_started = False

        status = self.read_status()
        if status["pulse_fifo_overflow_seen"] or status["group_fifo_overflow_seen"]:
            raise RuntimeError(
                "FIFO overflow flag is already set. Run clear-fifos/clear-flags "
                "and restart recording so complete groups can be guaranteed."
            )

        continuous_log_was_enabled = bool(
            self.regs.read_u32(REG_CONTROL) & CONTROL_FIFO_CONTINUOUS_LOG_ENABLE
        )
        self.enable_continuous_fifo_logging(True)

        csv_file = open(csv_path, "w", newline="")
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        csv_file.flush()

        try:
            # The CSV is fully opened before acquisition starts, so even a small
            # hardware FIFO is drained immediately with no shell/terminal race.
            if session_start is not None:
                session_start()
                session_started = True
                # Duration is acquisition time, not CSV setup time.
                start_monotonic = time.monotonic()
                last_disk_write = start_monotonic
                last_progress_print = start_monotonic
                print("Acquisition started: {0:.3f} s, writing {1}".format(duration_s, csv_path), flush=True)

            while duration_s <= 0.0 or time.monotonic() - start_monotonic < duration_s:
                did_work = False

                # Drain in bounded batches.  With a continuously producing
                # source an unbounded "until empty" loop can starve the outer
                # duration/progress checks if the FIFO never reaches exactly 0.
                for _ in range(max_pulse_records_per_pass):
                    pulse = self.read_pulse_record(pop=True)
                    if pulse is None:
                        break
                    pending_pulses.setdefault(pulse["group_id"], []).append(pulse)
                    did_work = True

                for _ in range(max_group_records_per_pass):
                    group = self.read_group_record(pop=True)
                    if group is None:
                        break
                    pending_groups[group["group_id"]] = group
                    did_work = True

                # Never silently write incomplete data after a hardware FIFO overflow.
                snapshot = self.read_snapshot()
                if snapshot["pulse_fifo_overflow"] or snapshot["group_fifo_overflow"]:
                    raise RuntimeError(
                        "FPGA FIFO overflow during recording (pulse={0}, group={1}). "
                        "CSV stopped rather than silently writing incomplete groups.".format(
                            snapshot["pulse_fifo_overflow"], snapshot["group_fifo_overflow"]
                        )
                    )

                if progress_interval_s > 0.0 and time.monotonic() - last_progress_print >= progress_interval_s:
                    elapsed = time.monotonic() - start_monotonic
                    approx_rows = rows_written + len(rows_waiting_for_disk)
                    print(
                        "{0:6.1f}s  groups={1}  rows={2}  fifo_p={3}  fifo_g={4}".format(
                            elapsed, groups_written, approx_rows,
                            snapshot["pulse_fifo_level"], snapshot["group_fifo_level"]
                        ),
                        flush=True,
                    )
                    last_progress_print = time.monotonic()

                # Finalize complete groups in monotonically increasing group-id order.
                for group_id in sorted(list(pending_groups.keys())):
                    group = pending_groups[group_id]
                    pulses = pending_pulses.get(group_id, [])
                    detected = int(group["detected_pulses"])
                    if len(pulses) < detected:
                        # Group FIFO order is chronological.  Do not write a later
                        # group before an earlier group is complete, otherwise the
                        # derived group_start_time_s would be wrong.
                        break
                    if len(pulses) > detected:
                        raise RuntimeError(
                            "Group {0} has {1} pulse records but hardware reports {2}.".format(
                                group_id, len(pulses), detected
                            )
                        )
                    if last_completed_group_id is not None and group_id <= last_completed_group_id:
                        raise RuntimeError("Non-monotonic/repeated group id {0}".format(group_id))

                    rows, first_pulse_cycles = self._combined_csv_rows(
                        group, pulses, previous_group_end_cycles, first_pulse_cycles
                    )
                    rows_waiting_for_disk.extend(rows)
                    previous_group_end_cycles = group["timestamp_cycles"]
                    last_completed_group_id = group_id
                    groups_written += 1
                    pending_groups.pop(group_id, None)
                    pending_pulses.pop(group_id, None)

                now = time.monotonic()
                if now - last_disk_write >= write_interval_s and rows_waiting_for_disk:
                    writer.writerows(rows_waiting_for_disk)
                    rows_written += len(rows_waiting_for_disk)
                    rows_waiting_for_disk[:] = []
                    csv_file.flush()
                    last_disk_write = now

                if not did_work:
                    time.sleep(fifo_poll_interval_s)

        finally:
            # Freeze the producer before the final drain.  In FIFO-safe sessions
            # this disables measurement and selects manual group boundaries.
            if session_started and session_stop is not None:
                session_stop()

            # One final FIFO drain can complete groups that were already emitted by
            # hardware just before Ctrl+C / duration expiry.
            while True:
                pulse = self.read_pulse_record(pop=True)
                if pulse is None:
                    break
                pending_pulses.setdefault(pulse["group_id"], []).append(pulse)
            while True:
                group = self.read_group_record(pop=True)
                if group is None:
                    break
                pending_groups[group["group_id"]] = group

            for group_id in sorted(list(pending_groups.keys())):
                group = pending_groups[group_id]
                pulses = pending_pulses.get(group_id, [])
                if len(pulses) != int(group["detected_pulses"]):
                    continue
                rows, first_pulse_cycles = self._combined_csv_rows(
                    group, pulses, previous_group_end_cycles, first_pulse_cycles
                )
                rows_waiting_for_disk.extend(rows)
                previous_group_end_cycles = group["timestamp_cycles"]
                last_completed_group_id = group_id
                groups_written += 1
                pending_groups.pop(group_id, None)
                pending_pulses.pop(group_id, None)

            if rows_waiting_for_disk:
                writer.writerows(rows_waiting_for_disk)
                rows_written += len(rows_waiting_for_disk)
                csv_file.flush()
            csv_file.close()
            self.enable_continuous_fifo_logging(continuous_log_was_enabled)

        return {
            "csv_path": csv_path,
            "groups_written": groups_written,
            "rows_written": rows_written,
            "pending_incomplete_groups": len(pending_groups),
            "pending_unmatched_pulse_groups": len(pending_pulses),
        }


def print_snapshot(snapshot):
    for key in sorted(snapshot.keys()):
        print("{0}: {1}".format(key, snapshot[key]))


def build_parser():
    parser = argparse.ArgumentParser(description="Control the PC05 pulse-control FPGA source")
    parser.add_argument("--base-address", type=lambda x: int(x, 0), default=BASE_ADDRESS)
    sub = parser.add_subparsers(dest="command")

    sub.add_parser("show")
    sub.add_parser("clear-state")
    sub.add_parser("clear-fifos")

    record = sub.add_parser("record")
    record.add_argument("--csv", default="pulse_control_data.csv")
    record.add_argument("--duration-s", type=float, default=0.0)
    record.add_argument("--write-interval-s", type=float, default=0.1)
    record.add_argument("--fifo-poll-interval-s", type=float, default=0.001)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    if args.command is None:
        parser.print_help()
        return 1

    try:
        control = PulseControl(base_address=args.base_address)
    except PermissionError:
        print("Permission denied opening /dev/mem. Run as root.", file=sys.stderr)
        return 1
    except Exception as error:
        print("Could not open pulse-control register map: {0}".format(error), file=sys.stderr)
        return 1

    try:
        if args.command == "show":
            print_snapshot(control.read_snapshot())
        elif args.command == "clear-state":
            control.clear_state()
        elif args.command == "clear-fifos":
            control.clear_fifos()
        elif args.command == "record":
            result = control.record_grouped_csv(
                csv_path=args.csv,
                duration_s=args.duration_s,
                write_interval_s=args.write_interval_s,
                fifo_poll_interval_s=args.fifo_poll_interval_s,
            )
            print(result)
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

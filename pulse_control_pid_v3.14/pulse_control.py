#!/usr/bin/env python3
"""Low-level PC07 Red Pitaya pulse-control runtime.

PC06 hardware contract used here:
  DIO0_P : qualified external reference trigger input
  DIO1_P : existing programmable 200 MHz GPIO output
  DIO2_P : reference_window_active scope marker
  IN1    : 125 MS/s pulse measurement
  OUT2   : 32-segment DAC2 waveform + existing PID correction path

The driver intentionally keeps the proven safe /dev/mem access pattern:
ONE aligned 32-bit AXI-Lite transaction per mmap read/write.  Do not replace
these primitives with struct.unpack_from(), pack_into(), or bulk MMIO access.

Compatible with older Python 3.x used on the Red Pitaya (no f-strings).
"""

import mmap
import os
import struct
import time
import warnings


BASE_ADDRESS = 0x40700000
MAP_SIZE = 0x1000

ADC_CLOCK_HZ = 125000000.0
GPIO_CLOCK_HZ = 200000000.0
TRIGGER_CLOCK_HZ = 200000000.0
ADC_SAMPLE_PERIOD_US = 0.008

DEFAULT_ADC_VOLTS_PER_COUNT = 1.0 / 8192.0
DEFAULT_DAC_COUNTS_PER_VOLT = 8192.0

MAX_GPIO_PULSES = 16
MAX_DAC_SEGMENTS = 32
MAX_REFERENCE_PULSES = 32

# Core registers
REG_CONTROL = 0x000
REG_COMMAND = 0x004
REG_STATUS = 0x008
REG_VERSION = 0x00C

# Measurement / grouping
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

# PID / calibration
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

# GPIO + new PC06 trigger/reference configuration
REG_GPIO_PERIOD = 0x080
REG_GPIO_PULSE_COUNT = 0x084
REG_EXTERNAL_TRIGGER_MIN_HIGH_POINTS = 0x088
REG_REFERENCE_TARGET_START_INDEX = 0x08C
REG_REFERENCE_TARGET_PULSE_COUNT = 0x090
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
REG_EXTERNAL_TRIGGER_ACTIVE_MIN_HIGH = 0x390
REG_LAST_REFERENCE_ID = 0x394
REG_LAST_REFERENCE_SUM = 0x398
REG_REFERENCE_STATUS = 0x39C

REG_PULSE_FIFO_HEAD = 0x400
PULSE_FIFO_WORDS = 10
REG_GROUP_FIFO_HEAD = 0x440
GROUP_FIFO_WORDS = 13

# CONTROL bits
CONTROL_MEASUREMENT_ENABLE = 1 << 0
CONTROL_FEEDBACK_ENABLE = 1 << 1
CONTROL_HOLD_INTEGRATOR = 1 << 2
CONTROL_MARKER_ENABLE = 1 << 3  # legacy internal debug marker; DIO2 is direct reference marker in PC06
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
# PC06 semantics: arm the next complete SELECTED reference-pulse snapshot.
COMMAND_ARM_REFERENCE_SNAPSHOT = 1 << 8

GROUP_SOURCE_EXTERNAL = 0
GROUP_SOURCE_DAC2 = 1
GROUP_SOURCE_MANUAL = 2

KNOWN_VERSION = 0x50433037  # "PC07"

INVALID_REASONS = {
    0x00: "none",
    0x01: "pulse_too_short",
    0x02: "pulse_too_long_or_no_falling_edge",
    0x03: "adc_saturation",
    0x04: "pulse_overlaps_post_baseline_window",
    0x05: "area_arithmetic_overflow",
    0x07: "measurement_configuration_invalid",
}


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
        self.memory.seek(int(offset))
        data = self.memory.read(4)
        if len(data) != 4:
            raise IOError("Short MMIO read at offset 0x{0:X}".format(int(offset)))
        return struct.unpack("<I", data)[0]

    def read_u32_block(self, offset, count):
        # Deliberately one aligned 32-bit transaction per word.
        count = int(count)
        return [self.read_u32(int(offset) + 4 * i) for i in range(count)]

    def read_s32(self, offset):
        value = self.read_u32(offset)
        return signed32(value)

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


def q24(value):
    return int(round(float(value) * 16777216.0))


def q31_nonnegative(value):
    raw = int(round(float(value) * 2147483648.0))
    if raw < 0 or raw > 0x7FFFFFFF:
        raise ValueError("Value converts outside non-negative Q1.31 range [0,1)")
    return raw


def fraction_q16(value):
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
            self.close()
            raise RuntimeError(
                "Unexpected FPGA register-map version 0x{0:08X}; expected PC07 0x{1:08X}".format(
                    version, KNOWN_VERSION
                )
            )

    def close(self):
        self.regs.close()

    def command(self, mask):
        self.regs.write_u32(REG_COMMAND, mask)

    def _set_control_bit(self, mask, enabled):
        value = self.regs.read_u32(REG_CONTROL)
        if enabled:
            value |= mask
        else:
            value &= ~mask
        self.regs.write_u32(REG_CONTROL, value)

    def clear_state(self):
        self.command(COMMAND_CLEAR_STATE)

    def clear_flags(self):
        self.command(COMMAND_CLEAR_FLAGS)

    def clear_fifos(self):
        self.command(COMMAND_CLEAR_FIFOS)

    def arm_reference_snapshot(self):
        """Arm sparse capture of all selected target pulses in the NEXT reference."""
        self.command(COMMAND_ARM_REFERENCE_SNAPSHOT)

    # Backward-friendly alias; semantics inherited from PC06.
    def arm_pulse_snapshot(self):
        self.arm_reference_snapshot()

    def enable_continuous_fifo_logging(self, enabled=True):
        self._set_control_bit(CONTROL_FIFO_CONTINUOUS_LOG_ENABLE, enabled)

    def continuous_fifo_logging_enabled(self):
        return bool(self.regs.read_u32(REG_CONTROL) & CONTROL_FIFO_CONTINUOUS_LOG_ENABLE)

    def enable_measurement(self, enabled=True):
        self._set_control_bit(CONTROL_MEASUREMENT_ENABLE, enabled)

    def enable_feedback(self, enabled=True):
        self._set_control_bit(CONTROL_FEEDBACK_ENABLE, enabled)

    def enable_gpio(self, enabled=True):
        self._set_control_bit(CONTROL_GPIO_ENABLE, enabled)

    def enable_dac2(self, enabled=True):
        self._set_control_bit(CONTROL_DAC2_ENABLE, enabled)

    def enable_marker(self, enabled=True):
        # PC06 DIO2_P is NOT gated by this bit. It directly shows reference_window_active.
        self._set_control_bit(CONTROL_MARKER_ENABLE, enabled)

    def set_hold_integrator(self, enabled=True):
        self._set_control_bit(CONTROL_HOLD_INTEGRATOR, enabled)

    def set_outputs(self, gpio_enabled=None, dac2_enabled=None):
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

    def set_group_source(self, source):
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

    # ------------------------- unit conversion -------------------------
    def adc_counts_from_volts(self, volts):
        return int(round(float(volts) / self.adc_volts_per_count))

    def dac_counts_from_volts(self, volts):
        value = int(round(float(volts) * self.dac_counts_per_volt))
        if value > 8191:
            value = 8191
        if value < -8192:
            value = -8192
        return value

    def dac_volts_from_counts(self, counts):
        return float(counts) / self.dac_counts_per_volt

    def area_counts_from_v_us(self, area_v_us):
        scale = self.adc_volts_per_count * ADC_SAMPLE_PERIOD_US
        return int(round(float(area_v_us) / scale))

    def area_v_us_from_counts(self, area_counts):
        return float(area_counts) * self.adc_volts_per_count * ADC_SAMPLE_PERIOD_US

    # ---------------------------- configuration ------------------------
    def configure_external_trigger(
        self,
        delay_ns,
        min_high_points,
        expected_frequency_hz=None,
    ):
        """Configure PC06 DIO0_P qualification and delay.

        200 MHz => one HIGH qualification point and one delay cycle are both 5 ns.
        The held-static buses cross 125 -> 200 MHz, so software configures them
        only while GROUP_SOURCE=manual and waits for active readback.
        """
        if self.get_group_source() != "manual":
            raise RuntimeError("configure_external_trigger requires GROUP_SOURCE='manual'")

        delay_ns = float(delay_ns)
        min_high_points = int(min_high_points)
        if delay_ns < 0.0:
            raise ValueError("external trigger delay must be non-negative")
        if min_high_points < 1 or min_high_points > 65535:
            raise ValueError("min_high_points must be 1..65535")

        delay_cycles = int(round(delay_ns * 1.0e-9 * TRIGGER_CLOCK_HZ))
        if delay_cycles < 0 or delay_cycles > 0xFFFFFFFF:
            raise ValueError("external trigger delay converts outside 32-bit range")

        if expected_frequency_hz is not None:
            expected_frequency_hz = float(expected_frequency_hz)
            if expected_frequency_hz <= 0.0:
                raise ValueError("expected external trigger frequency must be positive")
            period_cycles = int(round(TRIGGER_CLOCK_HZ / expected_frequency_hz))
            if period_cycles < 1:
                raise ValueError("expected trigger period is below one 200 MHz clock")
            if delay_cycles >= period_cycles:
                raise ValueError(
                    "external trigger delay must be shorter than one expected trigger period; "
                    "requested {0} cycles, expected period about {1} cycles".format(
                        delay_cycles, period_cycles
                    )
                )

        self.regs.write_u32(REG_EXTERNAL_TRIGGER_DELAY_CYCLES, delay_cycles)
        self.regs.write_u32(REG_EXTERNAL_TRIGGER_MIN_HIGH_POINTS, min_high_points)

        def ready():
            return (
                self.regs.read_u32(REG_EXTERNAL_TRIGGER_ACTIVE_DELAY) == delay_cycles
                and (self.regs.read_u32(REG_EXTERNAL_TRIGGER_ACTIVE_MIN_HIGH) & 0xFFFF)
                == min_high_points
            )

        self._wait_until(ready, "external trigger delay/min-HIGH CDC readback")
        return {
            "requested_delay_ns": delay_ns,
            "delay_cycles_200mhz": delay_cycles,
            "actual_programmed_delay_ns": 5.0 * delay_cycles,
            "min_high_points": min_high_points,
            "approx_min_high_ns": 5.0 * min_high_points,
            "resolution_ns": 5.0,
        }

    def configure_reference(self, target_start_index, target_pulse_count):
        target_start_index = int(target_start_index)
        target_pulse_count = int(target_pulse_count)
        if target_start_index < 0 or target_start_index > 31:
            raise ValueError("reference target_start_index must be 0..31")
        if target_pulse_count < 1 or target_pulse_count > MAX_REFERENCE_PULSES:
            raise ValueError("reference target_pulse_count must be 1..32")
        if target_start_index + target_pulse_count > 32:
            raise ValueError("reference target range must fit within pulse indices 0..31")
        if self.read_status()["reference_window_active"]:
            raise RuntimeError("Do not change reference selection during an active reference window")
        self.regs.write_u32(REG_REFERENCE_TARGET_START_INDEX, target_start_index)
        self.regs.write_u32(REG_REFERENCE_TARGET_PULSE_COUNT, target_pulse_count)
        return {
            "target_start_index": target_start_index,
            "target_pulse_count": target_pulse_count,
            "target_end_index_inclusive": target_start_index + target_pulse_count - 1,
        }

    def reference_config(self):
        start = self.regs.read_u32(REG_REFERENCE_TARGET_START_INDEX) & 0x1F
        count = self.regs.read_u32(REG_REFERENCE_TARGET_PULSE_COUNT) & 0x3F
        return {"target_start_index": start, "target_pulse_count": count}

    def reference_config_valid(self):
        cfg = self.reference_config()
        return (
            0 <= cfg["target_start_index"] <= 31
            and 1 <= cfg["target_pulse_count"] <= 32
            and cfg["target_start_index"] + cfg["target_pulse_count"] <= 32
        )

    def configure_gpio(self, frequency_hz, pulses, enable=None, apply=True):
        """Configure the existing DIO1_P 200 MHz GPIO output; PC06 keeps its pin role."""
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
                raise ValueError("GPIO windows must be sorted and non-overlapping")
            if end_cycle <= start_cycle:
                raise ValueError("A GPIO HIGH window rounded to zero clocks")
            if (end_cycle - start_cycle) < 2:
                raise ValueError("Each GPIO HIGH window must be at least 10 ns at 200 MHz")
            windows.append((start_cycle, end_cycle))
            last_end = end_cycle

        was_enabled = bool(self.regs.read_u32(REG_CONTROL) & CONTROL_GPIO_ENABLE)
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
                return (
                    self.read_status()["gpio_config_valid"]
                    and self.regs.read_u32(REG_GPIO_ACTIVE_PERIOD) == period_cycles
                    and (self.regs.read_u32(REG_GPIO_ACTIVE_PULSE_COUNT) & 0x1F)
                    == len(windows)
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
        """Configure OUT2 with 1..32 segments. The verified physical DAC path is untouched."""
        frequency_hz = float(frequency_hz)
        if frequency_hz <= 0.0:
            raise ValueError("DAC2 frequency must be positive")
        if len(sequence) < 1 or len(sequence) > MAX_DAC_SEGMENTS:
            raise ValueError("DAC2 sequence must contain 1..32 segments")

        total_fraction = sum(float(item[1]) for item in sequence)
        if abs(total_fraction - 1.0) > 0.000001:
            raise ValueError("DAC2 segment fractions must sum to 1.0")

        period_cycles = int(round(ADC_CLOCK_HZ / frequency_hz))
        if period_cycles < len(sequence):
            raise ValueError("DAC2 period is too short for the segment count")

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
                raise ValueError("DAC2 segment fractions must be positive")
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
        if durations[-1] < 1:
            raise ValueError("DAC2 rounded segment durations do not fit in one period")

        was_enabled = bool(self.regs.read_u32(REG_CONTROL) & CONTROL_DAC2_ENABLE)
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
                level_raw = fraction_q16(sequence[index][0])
                duration = durations[index]
            else:
                level_raw = 0
                duration = 1
            self.regs.write_u32(base, level_raw)
            self.regs.write_u32(base + 4, duration)

        if apply:
            self.command(COMMAND_DAC2_APPLY)

            def dac2_ready():
                return (
                    self.read_status()["dac2_config_valid"]
                    and self.regs.read_u32(REG_DAC2_ACTIVE_PERIOD) == period_cycles
                    and (self.regs.read_u32(REG_DAC2_ACTIVE_SEGMENT_COUNT) & 0x3F)
                    == len(sequence)
                )

            self._wait_until(dac2_ready, "DAC2 APPLY/readback")

        final_enable = was_enabled if enable is None else bool(enable)
        self.enable_dac2(final_enable)
        if feedback_was_enabled and final_enable:
            self.enable_feedback(True)

        return {
            "requested_frequency_hz": frequency_hz,
            "actual_frequency_hz": ADC_CLOCK_HZ / period_cycles,
            "period_cycles": period_cycles,
            "segment_count": len(sequence),
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
        external_trigger_min_high_points=2,
        external_trigger_frequency_hz=None,
        reference_target_start_index=0,
        reference_target_pulse_count=1,
        enable=None,
    ):
        was_enabled = bool(self.regs.read_u32(REG_CONTROL) & CONTROL_MEASUREMENT_ENABLE)
        feedback_was_enabled = self._pause_feedback()
        self.enable_measurement(False)
        self.set_group_source("manual")

        # PC06 reference_window_active is stateful: stopping measurement or
        # selecting MANUAL prevents new reference starts, but does not by itself
        # close a reference that was already open.  Clear the old reference
        # state BEFORE changing target_start/count; otherwise configure_reference()
        # correctly rejects the write with:
        #   "Do not change reference selection during an active reference window"
        #
        # This is safe here because feedback is paused, measurement is OFF, and
        # the reference source is MANUAL.
        self.clear_state()
        self._wait_until(
            lambda: not self.read_status()["reference_window_active"],
            "reference window clear before reconfiguration",
        )

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
        if group_source not in ("external", "dac2", "manual"):
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

        self.configure_external_trigger(
            external_trigger_delay_ns,
            external_trigger_min_high_points,
            expected_frequency_hz=external_trigger_frequency_hz,
        )
        self.configure_reference(reference_target_start_index, reference_target_pulse_count)

        negative = str(pulse_polarity).lower() in ("negative", "neg", "-")
        self._set_control_bit(CONTROL_NEGATIVE_POLARITY, negative)
        self.clear_state()

        # Store the requested source only after all held-static trigger config is settled.
        self.set_group_source(group_source)
        final_enable = was_enabled if enable is None else bool(enable)
        self.enable_measurement(final_enable)
        if final_enable:
            self._wait_until(
                lambda: self.read_status()["measurement_config_valid"],
                "measurement configuration valid",
            )
        if feedback_was_enabled:
            self.enable_feedback(True)

        return {
            "external_trigger": {
                "delay_ns": 5.0 * self.regs.read_u32(REG_EXTERNAL_TRIGGER_ACTIVE_DELAY),
                "min_high_points": self.regs.read_u32(REG_EXTERNAL_TRIGGER_ACTIVE_MIN_HIGH) & 0xFFFF,
            },
            "reference": self.reference_config(),
        }

    def _controller_interval_s(self, controller_dt_s=None):
        source = self.regs.read_u32(REG_GROUP_SOURCE) & 0x3
        if source == GROUP_SOURCE_DAC2:
            cycles = self.regs.read_u32(REG_DAC2_PERIOD)
            return float(cycles) / ADC_CLOCK_HZ
        if controller_dt_s is None:
            raise ValueError("controller_dt_s is required for external/manual reference source")
        return float(controller_dt_s)

    def configure_feedback(
        self,
        target_reference_sum_v_us=None,
        use_first_valid_reference_as_target=True,
        area_error_deadband_v_us=None,
        area_error_deadband_fraction=0.005,
        kp=0.20,
        ki_per_s=0.0,
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
        self.enable_feedback(False)
        auto_target = bool(use_first_valid_reference_as_target)
        if target_reference_sum_v_us is None and not auto_target:
            raise ValueError("A reference-sum target is required when auto target is disabled")
        if area_to_dac_gain_v_per_v_us is None:
            raise ValueError("AREA_TO_DAC_GAIN_V_PER_V_US must be provided/calibrated")
        if float(kp) < 0.0 or float(ki_per_s) < 0.0 or float(kd) < 0.0:
            raise ValueError("KP, KI_PER_S, and KD must be non-negative")
        if not 0.0 <= float(area_error_deadband_fraction) < 1.0:
            raise ValueError("deadband fraction must lie in [0,1)")
        if not 0.0 < float(max_integral_area_term_fraction) <= 1.0:
            raise ValueError("max_integral_area_term_fraction must lie in (0,1]")

        target_area = 0 if target_reference_sum_v_us is None else self.area_counts_from_v_us(target_reference_sum_v_us)
        if area_error_deadband_v_us is None:
            deadband_abs = 0
            deadband_mode = 1
        else:
            deadband_abs = abs(self.area_counts_from_v_us(area_error_deadband_v_us))
            deadband_mode = 0

        dt_s = self._controller_interval_s(controller_dt_s)
        if dt_s <= 0.0:
            raise ValueError("controller/reference interval must be positive")

        kp_q16 = q16(kp)
        ki_per_update = float(ki_per_s) * dt_s
        ki_q31 = q31_nonnegative(ki_per_update)
        if kp_q16 < 0 or kp_q16 > 0x7FFFFFFF:
            raise ValueError("KP converts outside positive Q16.16 range")
        if float(ki_per_s) > 0.0 and ki_q31 == 0:
            min_ki_per_s = 1.0 / (2147483648.0 * dt_s)
            raise ValueError(
                "KI_PER_S={0:g} is below Q1.31 resolution; minimum non-zero is about {1:g} 1/s".format(
                    float(ki_per_s), min_ki_per_s
                )
            )
        if float(ki_per_s) > 0.0:
            effective_ki_per_s = (float(ki_q31) / 2147483648.0) / dt_s
            if abs(effective_ki_per_s - float(ki_per_s)) / float(ki_per_s) > 0.01:
                warnings.warn(
                    "KI_PER_S quantizes from {0:g} to {1:g} 1/s".format(
                        float(ki_per_s), effective_ki_per_s
                    )
                )

        kd_per_update = float(kd) / dt_s if float(kd) != 0.0 else 0.0
        kd_q16 = q16(kd_per_update)
        if kd_q16 < 0 or kd_q16 > 0x7FFFFFFF:
            raise ValueError("KD / dt converts outside positive Q16.16 range")

        internal_gain = (
            float(area_to_dac_gain_v_per_v_us)
            * self.adc_volts_per_count
            * ADC_SAMPLE_PERIOD_US
            * self.dac_counts_per_volt
        )
        gain_q24 = q24(internal_gain)
        if gain_q24 <= 0 or gain_q24 > 0x7FFFFFFF:
            raise ValueError("AREA_TO_DAC_GAIN converts outside positive Q8.24 range")

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
        self.regs.write_u32(REG_INTEGRAL_LIMIT_FRACTION_Q16, fraction_q16(max_integral_area_term_fraction))
        self.regs.write_u32(REG_INTEGRAL_FRACTION_MODE, 1)
        self._set_control_bit(CONTROL_AUTO_TARGET, auto_target)
        self.clear_state()
        self.enable_feedback(was_enabled if enable is None else bool(enable))

        return {
            "reference_dt_s": dt_s,
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

    # ----------------------------- status -------------------------------
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
            "reference_incomplete_seen": bool(status & (1 << 25)),
            "reference_sum_overflow_seen": bool(status & (1 << 26)),
            "reference_window_active": bool(status & (1 << 27)),
        }

    def read_snapshot(self):
        fifo_levels = self.regs.read_u32(REG_FIFO_LEVELS)
        ref_status = self.regs.read_u32(REG_REFERENCE_STATUS)
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
            "external_trigger_active_min_high_points": self.regs.read_u32(REG_EXTERNAL_TRIGGER_ACTIVE_MIN_HIGH) & 0xFFFF,
            "last_reference_id": self.regs.read_u32(REG_LAST_REFERENCE_ID),
            "last_reference_sum_counts_samples": self.regs.read_s32(REG_LAST_REFERENCE_SUM),
            "last_reference_selected_count": ref_status & 0x3F,
            "last_reference_valid": bool(ref_status & (1 << 8)),
            "reference_window_active": bool(ref_status & (1 << 9)),
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
        if not self.reference_config_valid():
            problems.append("reference selection config invalid")
        if status["pulse_fifo_overflow_seen"] or status["group_fifo_overflow_seen"]:
            problems.append("FIFO overflow has occurred")
        if status["external_trigger_overrun_seen"]:
            problems.append("external trigger delay overrun has occurred")
        if status["reference_sum_overflow_seen"]:
            problems.append("reference sum overflow has occurred")
        if problems:
            raise RuntimeError("PC07 sanity check failed: " + "; ".join(problems))
        return status

    # ------------------------------ FIFO --------------------------------
    def read_pulse_record(self, pop=True):
        levels = self.regs.read_u32(REG_FIFO_LEVELS)
        if (levels & 0xFFFF) == 0:
            return None
        words = self.regs.read_u32_block(REG_PULSE_FIFO_HEAD, PULSE_FIFO_WORDS)
        timestamp = words[0] | (words[1] << 32)
        reference_id = words[2]  # PC06 selector rewrites legacy group_id to reference_id
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
        area_v_us = self.area_v_us_from_counts(area)
        valid = bool(flags & 0x01)
        pulse_height_v = None
        if valid and duration_s > 0.0:
            pulse_height_v = (area_v_us * 1.0e-6) / duration_s

        return {
            "timestamp_cycles": timestamp,
            "time_s": float(timestamp) / ADC_CLOCK_HZ,
            "reference_id": reference_id,
            "group_id": reference_id,  # compatibility alias
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
            "area_v_us": area_v_us,
            "pulse_height_v": pulse_height_v,
            "correction_active_counts": correction,
            "correction_active_v": self.dac_volts_from_counts(correction),
            "effective_high_counts": effective_high,
            "effective_high_v": self.dac_volts_from_counts(effective_high),
        }

    def read_group_record(self, pop=True):
        """Optional continuous PID-reference result FIFO reader (legacy group record format)."""
        levels = self.regs.read_u32(REG_FIFO_LEVELS)
        if ((levels >> 16) & 0xFFFF) == 0:
            return None
        words = self.regs.read_u32_block(REG_GROUP_FIFO_HEAD, GROUP_FIFO_WORDS)
        timestamp = words[0] | (words[1] << 32)
        flags = (words[3] >> 24) & 0xFF
        detected = words[3] & 0xFF
        valid = (words[3] >> 8) & 0xFF
        invalid = (words[3] >> 16) & 0xFF
        reference_sum = signed32(words[4])
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
            "reference_id": words[2],
            "detected_pulses": detected,
            "valid_pulses": valid,
            "invalid_pulses": invalid,
            "reference_valid_for_feedback": bool(flags & 0x01),
            "reference_sum_counts_samples": reference_sum,
            "reference_sum_v_us": self.area_v_us_from_counts(reference_sum),
            "target_area_counts_samples": target_area,
            "target_area_v_us": self.area_v_us_from_counts(target_area),
            "area_error_counts_samples": area_error,
            "area_error_v_us": self.area_v_us_from_counts(area_error),
            "p_area": p_area,
            "i_area": i_area,
            "d_area": d_area,
            "correction_pending_counts": correction_pending,
            "correction_active_counts": correction_active,
            "nominal_high_counts": nominal_high,
            "effective_high_counts": effective_high,
            "processing_latency_cycles": latency_cycles,
        }

    def read_selected_reference_snapshot(self, timeout_s=0.2):
        """Arm and read the next sparse PC07 reference snapshot.

        The FPGA captures ALL configured selected target pulse records from the
        next reference.  This method assumes sparse logging (continuous logging
        disabled) and that the FIFO is empty before arming.
        """
        cfg = self.reference_config()
        expected_count = int(cfg["target_pulse_count"])
        if expected_count < 1:
            raise RuntimeError("reference target count is invalid")
        if self.continuous_fifo_logging_enabled():
            raise RuntimeError("Sparse reference snapshot requires continuous FIFO logging OFF")

        self.clear_fifos()
        self.arm_reference_snapshot()
        deadline = time.monotonic() + float(timeout_s)

        while time.monotonic() < deadline:
            level = self.regs.read_u32(REG_FIFO_LEVELS) & 0xFFFF
            if level >= expected_count:
                break
            time.sleep(0.00005)

        level = self.regs.read_u32(REG_FIFO_LEVELS) & 0xFFFF
        pulses = []
        for _ in range(min(level, expected_count)):
            pulse = self.read_pulse_record(pop=True)
            if pulse is not None:
                pulses.append(pulse)

        complete = len(pulses) == expected_count
        same_reference = complete and len(set(p["reference_id"] for p in pulses)) == 1
        expected_ids = list(range(cfg["target_start_index"], cfg["target_start_index"] + expected_count))
        observed_ids = [int(p["pulse_id"]) for p in pulses]
        ids_match = complete and observed_ids == expected_ids
        reference_id = pulses[0]["reference_id"] if pulses and same_reference else None

        reference_sum_counts = sum(
            int(p["area_counts_samples"]) for p in pulses if p["valid"]
        )
        all_valid = complete and all(p["valid"] for p in pulses)
        return {
            "complete": complete,
            "same_reference": same_reference,
            "pulse_ids_match": ids_match,
            "reference_id": reference_id,
            "pulses": pulses,
            "reference_sum_counts_samples": reference_sum_counts,
            "reference_sum_v_us": self.area_v_us_from_counts(reference_sum_counts),
            "reference_valid": bool(complete and same_reference and ids_match and all_valid),
            "expected_count": expected_count,
            "observed_count": len(pulses),
            "expected_pulse_ids": expected_ids,
            "observed_pulse_ids": observed_ids,
            "timed_out": not complete,
        }

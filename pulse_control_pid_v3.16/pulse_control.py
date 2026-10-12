#!/usr/bin/env python3
"""Minimal PC09 / v3.16 dual-channel Red Pitaya driver."""

import mmap
import os
import struct
import time

BASE_ADDRESS = 0x40700000
MAP_SIZE = 0x1000

BANK_B = 0x000
BANK_A = 0x800

ADC_CLOCK_HZ = 125000000.0
TRIGGER_CLOCK_HZ = 200000000.0
ADC_SAMPLE_PERIOD_US = 0.008

KNOWN_VERSION = 0x50433039  # "PC09"
CHANNEL_ID_A = 1
CHANNEL_ID_B = 2

REG_CONTROL = 0x000
REG_COMMAND = 0x004
REG_STATUS = 0x008
REG_VERSION = 0x00C

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
REG_INTEGRAL_LIMIT_FRACTION_Q16 = 0x074
REG_INTEGRAL_FRACTION_MODE = 0x078
REG_KD_Q16_PER_UPDATE = 0x07C

REG_EXTERNAL_TRIGGER_MIN_HIGH_POINTS = 0x088
REG_REFERENCE_TARGET_START_INDEX = 0x08C
REG_REFERENCE_TARGET_PULSE_COUNT = 0x090
REG_FEEDBACK_SOURCE = 0x094
REG_CAPABILITIES = 0x098

REG_WAVEFORM_CONTROL = 0x09C
REG_WAVEFORM_STATUS = 0x0A0
REG_WAVEFORM_LENGTH = 0x0A4
REG_WAVEFORM_PRETRIGGER = 0x0A8
REG_WAVEFORM_TRIGGER_TIMESTAMP_LO = 0x0AC
REG_WAVEFORM_TRIGGER_TIMESTAMP_HI = 0x0B0
REG_WAVEFORM_TRIGGER_INDEX = 0x0B4
REG_WAVEFORM_CAPTURE_ID = 0x0B8
REG_WAVEFORM_VALID_SAMPLES = 0x0BC
REG_WAVEFORM_READ_INDEX = 0x0C0
REG_WAVEFORM_READ_DATA = 0x0C4

REG_CHANNEL_ID = 0x0C8

REG_DAC_PERIOD = 0x180
REG_DAC_SEGMENT_COUNT = 0x184
REG_DAC_LOW = 0x188
REG_DAC_HIGH = 0x18C
REG_DAC_SEGMENTS_BASE = 0x200

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
REG_DAC_ACTIVE_PERIOD = 0x37C
REG_DAC_ACTIVE_SEGMENT_COUNT = 0x380
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

CONTROL_MEASUREMENT_ENABLE = 1 << 0
CONTROL_FEEDBACK_ENABLE = 1 << 1
CONTROL_DAC_ENABLE = 1 << 5
CONTROL_AUTO_TARGET = 1 << 6
CONTROL_NEGATIVE_POLARITY = 1 << 7
CONTROL_FIFO_CONTINUOUS_LOG_ENABLE = 1 << 8

COMMAND_CLEAR_STATE = 1 << 0
COMMAND_CLEAR_FLAGS = 1 << 1
COMMAND_DAC_APPLY = 1 << 3
COMMAND_CLEAR_FIFOS = 1 << 4
COMMAND_POP_PULSE = 1 << 5
COMMAND_POP_GROUP = 1 << 6
COMMAND_MANUAL_GROUP = 1 << 7
COMMAND_ARM_REFERENCE_SNAPSHOT = 1 << 8

GROUP_SOURCE_EXTERNAL = 0
GROUP_SOURCE_DAC = 1
GROUP_SOURCE_MANUAL = 2

FEEDBACK_SOURCE_SELECTED = 0
FEEDBACK_SOURCE_GROUP_MEAN = 1

WAVEFORM_CONTROL_ARM = 1 << 0
WAVEFORM_CONTROL_ABORT = 1 << 1

INVALID_REASONS = {
    0x00: "none",
    0x01: "pulse_too_short",
    0x02: "pulse_too_long_or_no_falling_edge",
    0x03: "adc_saturation",
    0x04: "pulse_overlaps_post_baseline_window",
    0x05: "area_arithmetic_overflow",
    0x07: "measurement_configuration_invalid",
}


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


def fraction_q16(value):
    raw = int(round(float(value) * 65536.0))
    if raw < 0:
        raw = 0
    if raw > 65535:
        raw = 65535
    return raw


def q16(value):
    return int(round(float(value) * 65536.0))


def q24(value):
    return int(round(float(value) * 16777216.0))


def q31_nonnegative(value):
    raw = int(round(float(value) * 2147483648.0))
    if raw < 0 or raw > 0x7FFFFFFF:
        raise ValueError("value is outside Q1.31 non-negative range")
    return raw


def is_power_of_two(value):
    value = int(value)
    return value > 0 and (value & (value - 1)) == 0


class RegisterMap(object):
    """One 4 kB mapping, with a per-channel bank offset."""

    def __init__(self, bank_offset, base_address=BASE_ADDRESS):
        self.bank_offset = int(bank_offset)
        self.fd = None
        self.memory = None
        try:
            self.fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
            self.memory = mmap.mmap(
                self.fd,
                MAP_SIZE,
                mmap.MAP_SHARED,
                mmap.PROT_READ | mmap.PROT_WRITE,
                offset=int(base_address),
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

    def _offset(self, offset):
        value = self.bank_offset + int(offset)
        if value < 0 or value + 4 > MAP_SIZE or (value & 0x3):
            raise ValueError("invalid MMIO offset 0x{0:X}".format(value))
        return value

    def read_u32(self, offset):
        absolute = self._offset(offset)
        self.memory.seek(absolute)
        data = self.memory.read(4)
        if len(data) != 4:
            raise IOError("short MMIO read at 0x{0:X}".format(absolute))
        return struct.unpack("<I", data)[0]

    def read_u32_block(self, offset, count):
        count = int(count)
        return [self.read_u32(int(offset) + 4 * i) for i in range(count)]

    def read_s32(self, offset):
        return signed32(self.read_u32(offset))

    def write_u32(self, offset, value):
        absolute = self._offset(offset)
        self.memory.seek(absolute)
        self.memory.write(struct.pack("<I", int(value) & 0xFFFFFFFF))

    def write_s32(self, offset, value):
        self.write_u32(offset, value)


class PulseControl316(object):
    def __init__(
        self,
        channel,
        adc_volts_per_count=1.0 / 8192.0,
        dac_counts_per_volt=8192.0,
    ):
        self.channel = str(channel).upper()
        if self.channel == "A":
            self.bank_offset = BANK_A
            self.expected_channel_id = CHANNEL_ID_A
        elif self.channel == "B":
            self.bank_offset = BANK_B
            self.expected_channel_id = CHANNEL_ID_B
        else:
            raise ValueError("channel must be A or B")

        self.regs = RegisterMap(self.bank_offset)
        self.adc_volts_per_count = float(adc_volts_per_count)
        self.dac_counts_per_volt = float(dac_counts_per_volt)

        version = self.regs.read_u32(REG_VERSION)
        channel_id = self.regs.read_u32(REG_CHANNEL_ID)
        if version != KNOWN_VERSION or channel_id != self.expected_channel_id:
            self.close()
            raise RuntimeError(
                "Channel {0}: version/id mismatch: 0x{1:08X}/{2}, expected 0x{3:08X}/{4}".format(
                    self.channel,
                    version,
                    channel_id,
                    KNOWN_VERSION,
                    self.expected_channel_id,
                )
            )

    def close(self):
        if self.regs is not None:
            self.regs.close()
            self.regs = None

    def command(self, mask):
        self.regs.write_u32(REG_COMMAND, mask)

    def _set_control_bit(self, mask, enabled):
        value = self.regs.read_u32(REG_CONTROL)
        if enabled:
            value |= mask
        else:
            value &= ~mask
        self.regs.write_u32(REG_CONTROL, value)

    def enable_measurement(self, enabled=True):
        self._set_control_bit(CONTROL_MEASUREMENT_ENABLE, enabled)

    def enable_feedback(self, enabled=True):
        self._set_control_bit(CONTROL_FEEDBACK_ENABLE, enabled)

    def enable_dac(self, enabled=True):
        self._set_control_bit(CONTROL_DAC_ENABLE, enabled)

    def set_group_source(self, source):
        key = str(source).lower()
        if key == "external":
            value = GROUP_SOURCE_EXTERNAL
        elif key in ("dac", "dac2"):
            value = GROUP_SOURCE_DAC
        elif key == "manual":
            value = GROUP_SOURCE_MANUAL
        else:
            raise ValueError("group source must be external, dac, or manual")
        self.regs.write_u32(REG_GROUP_SOURCE, value)

    def get_group_source(self):
        value = self.regs.read_u32(REG_GROUP_SOURCE) & 0x3
        if value == GROUP_SOURCE_EXTERNAL:
            return "external"
        if value == GROUP_SOURCE_DAC:
            return "dac"
        if value == GROUP_SOURCE_MANUAL:
            return "manual"
        return "unknown"

    def enable_continuous_fifo_logging(self, enabled=True):
        self._set_control_bit(CONTROL_FIFO_CONTINUOUS_LOG_ENABLE, enabled)

    def continuous_fifo_logging_enabled(self):
        return bool(
            self.regs.read_u32(REG_CONTROL) & CONTROL_FIFO_CONTINUOUS_LOG_ENABLE
        )

    def arm_reference_snapshot(self):
        self.command(COMMAND_ARM_REFERENCE_SNAPSHOT)

    def manual_group_boundary(self):
        self.command(COMMAND_MANUAL_GROUP)

    def get_feedback_source(self):
        value = self.regs.read_u32(REG_FEEDBACK_SOURCE) & 0x1
        return "average" if value == FEEDBACK_SOURCE_GROUP_MEAN else "selected"

    def set_feedback_source(self, source):
        key = str(source).lower()
        names = {
            "selected": FEEDBACK_SOURCE_SELECTED,
            "reference": FEEDBACK_SOURCE_SELECTED,
            "selected_reference": FEEDBACK_SOURCE_SELECTED,
            "average": FEEDBACK_SOURCE_GROUP_MEAN,
            "group_mean": FEEDBACK_SOURCE_GROUP_MEAN,
            "mean": FEEDBACK_SOURCE_GROUP_MEAN,
        }
        if key not in names:
            raise ValueError("feedback source must be selected or average")
        if self.regs.read_u32(REG_CONTROL) & CONTROL_FEEDBACK_ENABLE:
            raise RuntimeError("feedback must be OFF before feedback-source changes")
        requested = names[key]
        self.regs.write_u32(REG_FEEDBACK_SOURCE, requested)
        actual = self.regs.read_u32(REG_FEEDBACK_SOURCE) & 0x1
        if actual != requested:
            raise RuntimeError(
                "Channel {0}: feedback-source readback mismatch".format(self.channel)
            )
        return self.get_feedback_source()

    def clear_state(self):
        self.command(COMMAND_CLEAR_STATE)

    def clear_flags(self):
        self.command(COMMAND_CLEAR_FLAGS)

    def clear_fifos(self):
        self.command(COMMAND_CLEAR_FIFOS)

    def clear_all(self):
        self.clear_flags()
        self.clear_fifos()
        self.clear_state()

    def adc_counts_from_volts(self, volts):
        counts = int(round(abs(float(volts)) / self.adc_volts_per_count))
        if counts < 1:
            counts = 1
        if counts > 8191:
            counts = 8191
        return counts

    def dac_counts_from_volts(self, volts):
        counts = int(round(float(volts) * self.dac_counts_per_volt))
        if counts < -8192:
            counts = -8192
        if counts > 8191:
            counts = 8191
        return counts

    def area_counts_from_v_us(self, area_v_us):
        scale = self.adc_volts_per_count * ADC_SAMPLE_PERIOD_US
        return int(round(float(area_v_us) / scale))

    def area_v_us_from_counts(self, area_counts):
        return (
            float(area_counts)
            * self.adc_volts_per_count
            * ADC_SAMPLE_PERIOD_US
        )

    def dac_volts_from_counts(self, counts):
        return float(counts) / self.dac_counts_per_volt

    def _wait_register(self, offset, expected, mask=0xFFFFFFFF, timeout_s=0.2):
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if (self.regs.read_u32(offset) & mask) == (int(expected) & mask):
                return
            time.sleep(0.0002)
        raise RuntimeError(
            "Channel {0}: timeout waiting for register 0x{1:03X}".format(
                self.channel, offset
            )
        )

    def configure_dac_high_limits(self, minimum_v, maximum_v):
        if self.regs.read_u32(REG_CONTROL) & CONTROL_FEEDBACK_ENABLE:
            raise RuntimeError("feedback must be OFF while changing DAC limits")

        minimum_counts = self.dac_counts_from_volts(minimum_v)
        maximum_counts = self.dac_counts_from_volts(maximum_v)
        if minimum_counts >= maximum_counts:
            raise ValueError("DAC HIGH minimum must be below maximum")

        self.regs.write_s32(REG_DAC_HIGH_MIN, minimum_counts)
        self.regs.write_s32(REG_DAC_HIGH_MAX, maximum_counts)

        if self.regs.read_s32(REG_DAC_HIGH_MIN) != minimum_counts:
            raise RuntimeError("DAC HIGH minimum readback mismatch")
        if self.regs.read_s32(REG_DAC_HIGH_MAX) != maximum_counts:
            raise RuntimeError("DAC HIGH maximum readback mismatch")

    def configure_dac(self, frequency_hz, low_v, high_v, sequence):
        frequency_hz = float(frequency_hz)
        if frequency_hz <= 0.0:
            raise ValueError("DAC frequency must be positive")
        if len(sequence) < 1 or len(sequence) > 32:
            raise ValueError("DAC sequence must contain 1..32 segments")

        total_fraction = sum(float(item[1]) for item in sequence)
        if abs(total_fraction - 1.0) > 0.000001:
            raise ValueError("DAC sequence fractions must sum to 1")

        period_cycles = int(round(ADC_CLOCK_HZ / frequency_hz))
        low_counts = self.dac_counts_from_volts(low_v)
        high_counts = self.dac_counts_from_volts(high_v)
        if high_counts <= low_counts:
            raise ValueError("DAC HIGH must exceed LOW")

        durations = []
        used = 0
        for index, item in enumerate(sequence):
            level = float(item[0])
            if level < 0.0 or level > 1.0:
                raise ValueError("DAC normalized level must be in [0,1]")
            if index == len(sequence) - 1:
                duration = period_cycles - used
            else:
                duration = int(round(float(item[1]) * period_cycles))
            if duration < 1:
                raise ValueError("DAC segment rounded below one clock")
            durations.append(duration)
            used += duration

        self.regs.write_u32(REG_DAC_PERIOD, period_cycles)
        self.regs.write_u32(REG_DAC_SEGMENT_COUNT, len(sequence))
        self.regs.write_s32(REG_DAC_LOW, low_counts)
        self.regs.write_s32(REG_DAC_HIGH, high_counts)

        for index in range(32):
            base = REG_DAC_SEGMENTS_BASE + 8 * index
            if index < len(sequence):
                level_raw = fraction_q16(sequence[index][0])
                duration = durations[index]
            else:
                level_raw = 0
                duration = 1
            self.regs.write_u32(base, level_raw)
            self.regs.write_u32(base + 4, duration)

        self.command(COMMAND_DAC_APPLY)
        self._wait_register(REG_DAC_ACTIVE_PERIOD, period_cycles)
        self._wait_register(REG_DAC_ACTIVE_SEGMENT_COUNT, len(sequence), mask=0x3F)

    def configure_measurement(
        self,
        trigger_fraction,
        target_height_v,
        pre_baseline_samples,
        post_baseline_samples,
        min_pulse_samples,
        max_pulse_samples,
        adc_saturation_limit_v,
        pulse_polarity,
        trigger_delay_ns,
        trigger_min_high_points,
        trigger_frequency_hz,
        reference_start_index,
        reference_pulse_count,
    ):
        if not is_power_of_two(pre_baseline_samples) or int(pre_baseline_samples) > 256:
            raise ValueError("pre_baseline_samples must be a power of two <= 256")
        if not is_power_of_two(post_baseline_samples) or int(post_baseline_samples) > 256:
            raise ValueError("post_baseline_samples must be a power of two <= 256")
        if int(min_pulse_samples) < 3:
            raise ValueError("min_pulse_samples must be >= 3")
        if int(max_pulse_samples) < int(min_pulse_samples):
            raise ValueError("max_pulse_samples must be >= min_pulse_samples")

        start = int(reference_start_index)
        count = int(reference_pulse_count)
        if start < 0 or count < 1 or start + count > 32:
            raise ValueError("reference pulse range must fit indices 0..31")

        delay_cycles = int(round(
            float(trigger_delay_ns) * 1.0e-9 * TRIGGER_CLOCK_HZ
        ))
        expected_period = int(round(
            TRIGGER_CLOCK_HZ / float(trigger_frequency_hz)
        ))
        if delay_cycles >= expected_period:
            raise ValueError("external trigger delay must be shorter than one trigger period")

        self.regs.write_u32(REG_TRIGGER_FRACTION_Q16, fraction_q16(trigger_fraction))
        self.regs.write_u32(REG_TARGET_HEIGHT, self.adc_counts_from_volts(target_height_v))
        self.regs.write_u32(REG_PRE_BASELINE, int(pre_baseline_samples))
        self.regs.write_u32(REG_POST_BASELINE, int(post_baseline_samples))
        self.regs.write_u32(REG_MIN_PULSE_SAMPLES, int(min_pulse_samples))
        self.regs.write_u32(REG_MAX_PULSE_SAMPLES, int(max_pulse_samples))
        self.regs.write_u32(
            REG_ADC_SATURATION_LIMIT,
            self.adc_counts_from_volts(adc_saturation_limit_v),
        )
        self.regs.write_u32(REG_EXPECTED_PULSES, 0)
        self.regs.write_u32(REG_MIN_VALID_PULSES, 1)
        self.regs.write_u32(REG_EXTERNAL_TRIGGER_DELAY_CYCLES, delay_cycles)
        self.regs.write_u32(
            REG_EXTERNAL_TRIGGER_MIN_HIGH_POINTS,
            int(trigger_min_high_points),
        )
        self.regs.write_u32(REG_REFERENCE_TARGET_START_INDEX, start)
        self.regs.write_u32(REG_REFERENCE_TARGET_PULSE_COUNT, count)
        self.regs.write_u32(REG_FEEDBACK_SOURCE, FEEDBACK_SOURCE_SELECTED)

        negative = str(pulse_polarity).lower() == "negative"
        self._set_control_bit(CONTROL_NEGATIVE_POLARITY, negative)

        self._wait_register(REG_EXTERNAL_TRIGGER_ACTIVE_DELAY, delay_cycles)
        self._wait_register(
            REG_EXTERNAL_TRIGGER_ACTIVE_MIN_HIGH,
            int(trigger_min_high_points),
            mask=0xFFFF,
        )

    def configure_feedback(
        self,
        target_area_v_us,
        auto_target,
        deadband_fraction,
        kp,
        ki_per_s,
        kd,
        area_to_dac_gain_v_per_v_us,
        correction_sign,
        max_dac_step_v,
        integral_limit_fraction,
        trigger_frequency_hz,
    ):
        if area_to_dac_gain_v_per_v_us is None:
            return False

        dt_s = 1.0 / float(trigger_frequency_hz)
        target_counts = (
            0 if target_area_v_us is None
            else self.area_counts_from_v_us(target_area_v_us)
        )

        kp_q16 = q16(kp)
        ki_q31 = q31_nonnegative(float(ki_per_s) * dt_s)
        kd_q16 = q16(float(kd) / dt_s) if float(kd) != 0.0 else 0

        internal_gain = (
            float(area_to_dac_gain_v_per_v_us)
            * self.adc_volts_per_count
            * ADC_SAMPLE_PERIOD_US
            * self.dac_counts_per_volt
        )
        gain_q24 = q24(internal_gain)
        if gain_q24 <= 0 or gain_q24 > 0x7FFFFFFF:
            raise ValueError("AREA_TO_DAC_GAIN converts outside positive Q8.24")

        self.regs.write_s32(REG_TARGET_AREA, target_counts)
        self.regs.write_u32(REG_DEADBAND_ABS, 0)
        self.regs.write_u32(
            REG_DEADBAND_FRACTION_Q16, fraction_q16(deadband_fraction)
        )
        self.regs.write_u32(REG_DEADBAND_MODE, 1)
        self.regs.write_u32(REG_KP_Q16, kp_q16)
        self.regs.write_u32(REG_KI_Q31_PER_UPDATE, ki_q31)
        self.regs.write_u32(REG_KD_Q16_PER_UPDATE, kd_q16)
        self.regs.write_u32(REG_AREA_TO_DAC_GAIN_Q24, gain_q24)
        self.regs.write_u32(
            REG_CORRECTION_SIGN, 1 if float(correction_sign) < 0.0 else 0
        )
        self.regs.write_u32(
            REG_MAX_DAC_STEP,
            abs(self.dac_counts_from_volts(max_dac_step_v)),
        )
        self.regs.write_u32(
            REG_INTEGRAL_LIMIT_FRACTION_Q16,
            fraction_q16(integral_limit_fraction),
        )
        self.regs.write_u32(REG_INTEGRAL_FRACTION_MODE, 1)
        self._set_control_bit(CONTROL_AUTO_TARGET, bool(auto_target))
        self.clear_state()
        return True

    def measurement_on(self, source="external"):
        self.enable_feedback(False)
        self.enable_measurement(False)
        self.enable_continuous_fifo_logging(False)
        self.set_group_source("manual")
        self.clear_all()
        self.enable_measurement(True)
        time.sleep(0.001)
        self.set_group_source(source)

    def measurement_off(self):
        self.enable_feedback(False)
        self.enable_measurement(False)
        self.set_group_source("manual")
        self.clear_state()

    def feedback_on(self):
        if not (self.regs.read_u32(REG_CONTROL) & CONTROL_MEASUREMENT_ENABLE):
            raise RuntimeError("measurement must be ON before feedback")
        self.enable_feedback(True)

    def reference_config(self):
        start = self.regs.read_u32(REG_REFERENCE_TARGET_START_INDEX) & 0x1F
        count = self.regs.read_u32(REG_REFERENCE_TARGET_PULSE_COUNT) & 0x3F
        return {
            "target_start_index": start,
            "target_pulse_count": count,
        }

    def reference_config_valid(self):
        cfg = self.reference_config()
        return (
            0 <= cfg["target_start_index"] <= 31
            and 1 <= cfg["target_pulse_count"] <= 32
            and cfg["target_start_index"] + cfg["target_pulse_count"] <= 32
        )

    def read_status(self):
        raw = self.regs.read_u32(REG_STATUS)
        return {
            "raw": raw,
            "measurement_enabled": bool(raw & (1 << 0)),
            "feedback_enabled": bool(raw & (1 << 1)),
            "dac_enabled": bool(raw & (1 << 3)),
            "measurement_config_valid": bool(raw & (1 << 5)),
            "controller_config_valid": bool(raw & (1 << 6)),
            "dac_config_valid": bool(raw & (1 << 8)),
            "target_latched": bool(raw & (1 << 9)),
            "pulse_fifo_nonempty": bool(raw & (1 << 10)),
            "group_fifo_nonempty": bool(raw & (1 << 11)),
            "pulse_fifo_full": bool(raw & (1 << 12)),
            "group_fifo_full": bool(raw & (1 << 13)),
            "pulse_fifo_overflow_seen": bool(raw & (1 << 14)),
            "group_fifo_overflow_seen": bool(raw & (1 << 15)),
            "invalid_seen": bool(raw & (1 << 16)),
            "correction_saturation_seen": bool(raw & (1 << 17)),
            "integral_saturation_seen": bool(raw & (1 << 18)),
            "marker_timeout_seen": bool(raw & (1 << 19)),
            "external_trigger_overrun_seen": bool(raw & (1 << 24)),
            "reference_incomplete_seen": bool(raw & (1 << 25)),
            "reference_sum_overflow_seen": bool(raw & (1 << 26)),
            "reference_window_active": bool(raw & (1 << 27)),
        }

    def sanity_check(
        self,
        require_measurement=False,
        require_dac=False,
        require_feedback=False,
    ):
        status = self.read_status()
        problems = []
        if require_measurement and not status["measurement_config_valid"]:
            problems.append("measurement_config_valid=0")
        if require_dac and not status["dac_config_valid"]:
            problems.append("dac_config_valid=0")
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
            raise RuntimeError(
                "Channel {0} sanity check failed: {1}".format(
                    self.channel, "; ".join(problems)
                )
            )
        return status

    def read_snapshot(self):
        fifo = self.regs.read_u32(REG_FIFO_LEVELS)
        ref = self.regs.read_u32(REG_REFERENCE_STATUS)
        status = self.read_status()

        target = self.regs.read_s32(REG_TARGET_ACTIVE)
        error = self.regs.read_s32(REG_AREA_ERROR)
        p_term = self.regs.read_s32(REG_P_TERM)
        i_term = self.regs.read_s32(REG_I_TERM)
        d_term = self.regs.read_s32(REG_D_TERM)
        corr_pending = self.regs.read_s32(REG_CORRECTION_PENDING)
        corr_active = self.regs.read_s32(REG_CORRECTION_ACTIVE)
        effective_high = self.regs.read_s32(REG_EFFECTIVE_HIGH)

        return {
            "status": status,
            "status_raw": status["raw"],
            "control_raw": self.regs.read_u32(REG_CONTROL),
            "capabilities": self.regs.read_u32(REG_CAPABILITIES),
            "channel_id": self.regs.read_u32(REG_CHANNEL_ID),

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
                self.regs.read_u32(REG_INVALID_REASON) & 0xFF,
                "unknown",
            ),
            "group_count": self.regs.read_u32(REG_GROUP_COUNT),

            "target": target,
            "target_area_counts_samples": target,
            "error": error,
            "area_error_counts_samples": error,
            "p": p_term,
            "p_area": p_term,
            "i": i_term,
            "i_area": i_term,
            "d": d_term,
            "d_area": d_term,
            "correction_pending": corr_pending,
            "correction_pending_counts": corr_pending,
            "correction_active": corr_active,
            "correction_active_counts": corr_active,
            "effective_high": effective_high,
            "effective_high_counts": effective_high,

            "dac_high_min_counts": self.regs.read_s32(REG_DAC_HIGH_MIN),
            "dac_high_max_counts": self.regs.read_s32(REG_DAC_HIGH_MAX),
            "update_count": self.regs.read_u32(REG_UPDATE_COUNT),
            "processing_latency_cycles": self.regs.read_u32(
                REG_PROCESSING_LATENCY
            ),
            "pulse_fifo_level": fifo & 0xFFFF,
            "group_fifo_level": (fifo >> 16) & 0xFFFF,
            "pulse_fifo_overflow": self.regs.read_u32(
                REG_PULSE_FIFO_OVERFLOW
            ),
            "group_fifo_overflow": self.regs.read_u32(
                REG_GROUP_FIFO_OVERFLOW
            ),
            "external_trigger_count": self.regs.read_u32(
                REG_EXTERNAL_TRIGGER_COUNT
            ),
            "external_trigger_active_delay_cycles": self.regs.read_u32(
                REG_EXTERNAL_TRIGGER_ACTIVE_DELAY
            ),
            "external_trigger_active_min_high_points": (
                self.regs.read_u32(REG_EXTERNAL_TRIGGER_ACTIVE_MIN_HIGH)
                & 0xFFFF
            ),
            "last_reference_id": self.regs.read_u32(REG_LAST_REFERENCE_ID),
            "last_reference_sum": self.regs.read_s32(
                REG_LAST_REFERENCE_SUM
            ),
            "last_reference_sum_counts_samples": self.regs.read_s32(
                REG_LAST_REFERENCE_SUM
            ),
            "last_reference_count": ref & 0x3F,
            "last_reference_selected_count": ref & 0x3F,
            "last_reference_valid": bool(ref & (1 << 8)),
            "reference_window_active": bool(ref & (1 << 9)),
        }

    def read_pulse_record(self, pop=True):
        levels = self.regs.read_u32(REG_FIFO_LEVELS)
        if (levels & 0xFFFF) == 0:
            return None

        words = self.regs.read_u32_block(
            REG_PULSE_FIFO_HEAD,
            PULSE_FIFO_WORDS,
        )

        timestamp = words[0] | (words[1] << 32)
        reference_id = words[2]
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
            "group_id": reference_id,
            "pulse_id": pulse_id,
            "valid": valid,
            "invalid": bool(flags & 0x02),
            "invalid_reason": INVALID_REASONS.get(reason, "unknown"),
            "bpre_counts": bpre,
            "bpost_counts": bpost,
            "bpre_v": float(bpre) * self.adc_volts_per_count,
            "bpost_v": float(bpost) * self.adc_volts_per_count,
            "baseline_for_area_v": (
                0.5
                * float(bpre + bpost)
                * self.adc_volts_per_count
            ),
            "threshold_counts": threshold,
            "peak_raw_counts": peak_raw,
            "peak_height_counts": peak_height,
            "peak_height_v": (
                float(peak_height) * self.adc_volts_per_count
            ),
            "duration_samples": duration,
            "duration_s": duration_s,
            "area_counts_samples": area,
            "area_v_us": area_v_us,
            "pulse_height_v": pulse_height_v,
            "correction_active_counts": correction,
            "correction_active_v": self.dac_volts_from_counts(correction),
            "effective_high_counts": effective_high,
            "effective_high_v": self.dac_volts_from_counts(
                effective_high
            ),
        }

    def read_group_record(self, pop=True):
        levels = self.regs.read_u32(REG_FIFO_LEVELS)
        if ((levels >> 16) & 0xFFFF) == 0:
            return None

        words = self.regs.read_u32_block(
            REG_GROUP_FIFO_HEAD,
            GROUP_FIFO_WORDS,
        )

        flags = (words[3] >> 24) & 0xFF
        detected = words[3] & 0xFF
        valid = (words[3] >> 8) & 0xFF
        invalid = (words[3] >> 16) & 0xFF

        record = {
            "timestamp_cycles": words[0] | (words[1] << 32),
            "feedback_id": words[2],
            "detected_pulses": detected,
            "valid_pulses": valid,
            "invalid_pulses": invalid,
            "feedback_valid": bool(flags & 0x01),
            "feedback_area_counts_samples": signed32(words[4]),
            "target_area_counts_samples": signed32(words[5]),
            "area_error_counts_samples": signed32(words[6]),
            "p_area": signed32(words[7]),
            "i_area": signed32(words[8]),
            "d_area": signed32(words[9]),
            "correction_pending_counts": signed16(
                (words[10] >> 16) & 0xFFFF
            ),
            "correction_active_counts": signed16(
                words[10] & 0xFFFF
            ),
            "nominal_high_counts": signed16(
                (words[11] >> 16) & 0xFFFF
            ),
            "effective_high_counts": signed16(words[11] & 0xFFFF),
            "processing_latency_cycles": words[12],
        }

        record["time_s"] = (
            float(record["timestamp_cycles"]) / ADC_CLOCK_HZ
        )
        record["feedback_area_v_us"] = self.area_v_us_from_counts(
            record["feedback_area_counts_samples"]
        )
        record["target_area_v_us"] = self.area_v_us_from_counts(
            record["target_area_counts_samples"]
        )
        record["area_error_v_us"] = self.area_v_us_from_counts(
            record["area_error_counts_samples"]
        )

        if pop:
            self.command(COMMAND_POP_GROUP)

        if self.get_feedback_source() == "selected":
            record["reference_id"] = record["feedback_id"]
            record["reference_valid_for_feedback"] = record[
                "feedback_valid"
            ]
            record["reference_sum_counts_samples"] = record[
                "feedback_area_counts_samples"
            ]
            record["reference_sum_v_us"] = record[
                "feedback_area_v_us"
            ]
        else:
            record["group_id"] = record["feedback_id"]
            record["group_valid_for_feedback"] = record[
                "feedback_valid"
            ]
            record["group_mean_counts_samples"] = record[
                "feedback_area_counts_samples"
            ]
            record["group_mean_v_us"] = record[
                "feedback_area_v_us"
            ]

        return record

    def begin_selected_reference_snapshot(self):
        cfg = self.reference_config()
        expected_count = int(cfg["target_pulse_count"])
        if expected_count < 1:
            raise RuntimeError("reference target count is invalid")
        if self.continuous_fifo_logging_enabled():
            raise RuntimeError(
                "sparse reference snapshot requires continuous FIFO logging OFF"
            )

        self.clear_fifos()
        self.arm_reference_snapshot()
        return expected_count

    def collect_selected_reference_snapshot(self, expected_count=None):
        cfg = self.reference_config()
        if expected_count is None:
            expected_count = int(cfg["target_pulse_count"])
        expected_count = int(expected_count)

        level = self.regs.read_u32(REG_FIFO_LEVELS) & 0xFFFF
        pulses = []
        for _ in range(min(level, expected_count)):
            pulse = self.read_pulse_record(pop=True)
            if pulse is not None:
                pulses.append(pulse)

        complete = len(pulses) == expected_count
        same_reference = (
            complete
            and len(set(p["reference_id"] for p in pulses)) == 1
        )
        expected_ids = list(
            range(
                cfg["target_start_index"],
                cfg["target_start_index"] + expected_count,
            )
        )
        observed_ids = [int(p["pulse_id"]) for p in pulses]
        ids_match = complete and observed_ids == expected_ids
        reference_id = (
            pulses[0]["reference_id"]
            if pulses and same_reference
            else None
        )

        reference_sum_counts = sum(
            int(p["area_counts_samples"])
            for p in pulses
            if p["valid"]
        )
        all_valid = complete and all(p["valid"] for p in pulses)

        return {
            "complete": complete,
            "same_reference": same_reference,
            "pulse_ids_match": ids_match,
            "reference_id": reference_id,
            "pulses": pulses,
            "reference_sum_counts_samples": reference_sum_counts,
            "reference_sum_v_us": self.area_v_us_from_counts(
                reference_sum_counts
            ),
            "reference_valid": bool(
                complete
                and same_reference
                and ids_match
                and all_valid
            ),
            "expected_count": expected_count,
            "observed_count": len(pulses),
            "expected_pulse_ids": expected_ids,
            "observed_pulse_ids": observed_ids,
            "timed_out": not complete,
        }

    def read_selected_reference_snapshot(self, timeout_s=0.2):
        expected_count = self.begin_selected_reference_snapshot()
        deadline = time.monotonic() + float(timeout_s)

        while time.monotonic() < deadline:
            level = self.regs.read_u32(REG_FIFO_LEVELS) & 0xFFFF
            if level >= expected_count:
                break
            time.sleep(0.00005)

        return self.collect_selected_reference_snapshot(
            expected_count=expected_count
        )

    def configure_waveform_capture(
        self,
        length_samples=16384,
        pretrigger_samples=1024,
    ):
        length_samples = int(length_samples)
        pretrigger_samples = int(pretrigger_samples)

        if length_samples < 1 or length_samples > 16384:
            raise ValueError("waveform length must be 1..16384 samples")
        if (
            pretrigger_samples < 0
            or pretrigger_samples >= length_samples
        ):
            raise ValueError(
                "waveform pretrigger must satisfy 0 <= pretrigger < length"
            )

        status = self.get_waveform_status()
        if status["armed"] or (
            status["triggered"] and not status["capture_done"]
        ):
            raise RuntimeError(
                "abort active waveform capture before reconfiguration"
            )

        self.regs.write_u32(REG_WAVEFORM_LENGTH, length_samples)
        self.regs.write_u32(
            REG_WAVEFORM_PRETRIGGER,
            pretrigger_samples,
        )

        actual_length = (
            self.regs.read_u32(REG_WAVEFORM_LENGTH) & 0x7FFF
        )
        actual_pre = (
            self.regs.read_u32(REG_WAVEFORM_PRETRIGGER) & 0x7FFF
        )
        status = self.get_waveform_status()

        if (
            actual_length != length_samples
            or actual_pre != pretrigger_samples
        ):
            raise RuntimeError(
                "Channel {0}: waveform configuration readback mismatch".format(
                    self.channel
                )
            )
        if not status["config_valid"]:
            raise RuntimeError(
                "Channel {0}: hardware rejected waveform configuration".format(
                    self.channel
                )
            )

        return {
            "length_samples": actual_length,
            "pretrigger_samples": actual_pre,
            "sample_rate_hz": ADC_CLOCK_HZ,
            "sample_period_s": 1.0 / ADC_CLOCK_HZ,
            "duration_s": float(actual_length) / ADC_CLOCK_HZ,
        }

    def arm_waveform_capture(self):
        status = self.get_waveform_status()
        if not status["config_valid"]:
            raise RuntimeError("waveform capture configuration is invalid")
        if status["armed"] or (
            status["triggered"] and not status["capture_done"]
        ):
            raise RuntimeError("waveform capture is already active")
        self.regs.write_u32(
            REG_WAVEFORM_CONTROL,
            WAVEFORM_CONTROL_ARM,
        )

    def abort_waveform_capture(self):
        self.regs.write_u32(
            REG_WAVEFORM_CONTROL,
            WAVEFORM_CONTROL_ABORT,
        )

    def get_waveform_status(self):
        raw = self.regs.read_u32(REG_WAVEFORM_STATUS)
        timestamp = (
            self.regs.read_u32(REG_WAVEFORM_TRIGGER_TIMESTAMP_LO)
            | (
                self.regs.read_u32(
                    REG_WAVEFORM_TRIGGER_TIMESTAMP_HI
                )
                << 32
            )
        )
        return {
            "raw": raw,
            "armed": bool(raw & (1 << 0)),
            "triggered": bool(raw & (1 << 1)),
            "capture_done": bool(raw & (1 << 2)),
            "config_valid": bool(raw & (1 << 3)),
            "capture_error": bool(raw & (1 << 4)),
            "capture_id": self.regs.read_u32(
                REG_WAVEFORM_CAPTURE_ID
            ),
            "valid_sample_count": (
                self.regs.read_u32(REG_WAVEFORM_VALID_SAMPLES)
                & 0x7FFF
            ),
            "trigger_index": (
                self.regs.read_u32(REG_WAVEFORM_TRIGGER_INDEX)
                & 0x7FFF
            ),
            "trigger_timestamp_cycles": timestamp,
            "trigger_timestamp_s": float(timestamp) / ADC_CLOCK_HZ,
            "length_samples": (
                self.regs.read_u32(REG_WAVEFORM_LENGTH) & 0x7FFF
            ),
            "pretrigger_samples": (
                self.regs.read_u32(REG_WAVEFORM_PRETRIGGER)
                & 0x7FFF
            ),
        }

    def read_waveform_sample(self, index):
        index = int(index)
        status = self.get_waveform_status()
        if not status["capture_done"]:
            raise RuntimeError("waveform capture is not complete")
        if index < 0 or index >= int(status["valid_sample_count"]):
            raise IndexError(
                "waveform sample index outside completed capture"
            )

        self.regs.write_u32(REG_WAVEFORM_READ_INDEX, index)
        word = self.regs.read_u32(REG_WAVEFORM_READ_DATA)

        adc_counts = signed16(word & 0xFFFF)
        marker = bool(word & (1 << 16))
        return {
            "index": index,
            "word": word,
            "adc_counts": adc_counts,
            "adc_v": float(adc_counts) * self.adc_volts_per_count,
            "reference_window_active": marker,
        }

    def read_waveform(self):
        status = self.get_waveform_status()
        if not status["capture_done"]:
            raise RuntimeError("waveform capture is not complete")

        count = int(status["valid_sample_count"])
        adc_counts = []
        adc_v = []
        marker = []

        for index in range(count):
            sample = self.read_waveform_sample(index)
            adc_counts.append(sample["adc_counts"])
            adc_v.append(sample["adc_v"])
            marker.append(sample["reference_window_active"])

        return {
            "adc_counts": adc_counts,
            "adc_v": adc_v,
            "reference_window_active": marker,
            "sample_rate_hz": ADC_CLOCK_HZ,
            "sample_period_s": 1.0 / ADC_CLOCK_HZ,
            "trigger_index": status["trigger_index"],
            "trigger_timestamp_cycles": status[
                "trigger_timestamp_cycles"
            ],
            "trigger_timestamp_s": status["trigger_timestamp_s"],
            "capture_id": status["capture_id"],
            "valid_sample_count": count,
            "length_samples": status["length_samples"],
            "pretrigger_samples": status["pretrigger_samples"],
        }

    def acquire_waveform(
        self,
        length_samples=16384,
        pretrigger_samples=1024,
        timeout_s=1.0,
    ):
        timeout_s = float(timeout_s)
        if timeout_s <= 0.0:
            raise ValueError("waveform timeout must be positive")

        status = self.get_waveform_status()
        if status["armed"] or (
            status["triggered"] and not status["capture_done"]
        ):
            self.abort_waveform_capture()
            time.sleep(0.001)

        self.configure_waveform_capture(
            length_samples,
            pretrigger_samples,
        )
        old_capture_id = self.get_waveform_status()["capture_id"]
        self.arm_waveform_capture()

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.get_waveform_status()
            if status["capture_error"]:
                raise RuntimeError(
                    "Channel {0}: waveform capture error".format(
                        self.channel
                    )
                )
            if (
                status["capture_done"]
                and status["capture_id"] != old_capture_id
            ):
                return self.read_waveform()
            time.sleep(0.0001)

        self.abort_waveform_capture()
        raise RuntimeError(
            "Channel {0}: timed out waiting for waveform capture".format(
                self.channel
            )
        )

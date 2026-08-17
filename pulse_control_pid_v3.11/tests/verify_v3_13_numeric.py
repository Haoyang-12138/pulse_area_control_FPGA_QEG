#!/usr/bin/env python3
"""Pure-Python regression of the v3.13 Q1.31 Ki arithmetic contract.

This does not replace RTL simulation or Vivado timing.  It verifies the intended
fixed-point equations used by pulse_area_pid_controller_v3.sv.
"""

Q31 = 2147483648


def q31_ki(ki_per_s, dt_s):
    raw = int(round(float(ki_per_s) * float(dt_s) * Q31))
    if raw < 0 or raw > 0x7FFFFFFF:
        raise ValueError("Ki per update outside non-negative Q1.31 range")
    return raw


def round_q31_to_int(raw):
    """Match the RTL symmetric round-to-nearest rule."""
    raw &= (1 << 64) - 1
    negative = bool(raw & (1 << 63))
    floor_bits = raw >> 31
    if floor_bits & (1 << 32):
        floor_value = floor_bits - (1 << 33)
    else:
        floor_value = floor_bits
    fraction = raw & ((1 << 31) - 1)
    half = 1 << 30
    increment = fraction >= half if not negative else fraction > half
    return floor_value + (1 if increment else 0)


def signed64(value):
    value &= (1 << 64) - 1
    return value - (1 << 64) if value & (1 << 63) else value


def test_rounding():
    samples = [
        (0.4, 0),
        (0.5, 1),
        (0.6, 1),
        (-0.4, 0),
        (-0.5, -1),
        (-0.6, -1),
    ]
    for value, expected in samples:
        raw = int(round(value * Q31))
        got = round_q31_to_int(raw)
        assert got == expected, (value, got, expected)


def test_symmetric_error():
    dt = 1.0e-6
    error = 2831
    for ki_per_s in (1.0, 2.0, 5.0, 10.0, 15.2588):
        ki = q31_ki(ki_per_s, dt)
        acc = 0
        for _ in range(100000):
            acc = signed64(acc + error * ki)
            acc = signed64(acc - error * ki)
        assert acc == 0, (ki_per_s, acc)


def test_constant_error_symmetry():
    dt = 1.0e-6
    error = 2831
    ki = q31_ki(5.0, dt)
    n = 10000
    pos = 0
    neg = 0
    for _ in range(n):
        pos = signed64(pos + error * ki)
        neg = signed64(neg - error * ki)
    assert pos == -neg, (pos, neg)
    assert round_q31_to_int(pos) == -round_q31_to_int(neg)


def main():
    test_rounding()
    test_symmetric_error()
    test_constant_error_symmetry()
    print("v3.13 Q1.31 Ki numerical regression: PASS")
    for ki_per_s in (1.0, 2.0, 5.0, 10.0):
        raw = q31_ki(ki_per_s, 1.0e-6)
        effective = (float(raw) / Q31) / 1.0e-6
        print("Ki={0:g}/s -> raw={1} -> effective={2:.9g}/s".format(
            ki_per_s, raw, effective
        ))


if __name__ == "__main__":
    main()

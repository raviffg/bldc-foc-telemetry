#!/usr/bin/env python3
"""Torque ripple and current THD analysis from the 20 kHz shunt capture.

Inputs: phase shunt currents (ia, ib) and DC bus voltage logged during a
regen braking event. We reconstruct the third phase, compute per-electrical-
cycle THD on the phase current, and estimate torque ripple from the q-axis
current envelope.

Torque is assumed proportional to Iq (surface PM machine), so ripple_pct is
peak-to-peak Iq over mean Iq.
"""

import csv
import math
from dataclasses import dataclass
from pathlib import Path

CAPTURE = Path(__file__).resolve().parent.parent / "data" / "regen_braking_bus_capture.csv"
POLE_PAIRS = 7
NOMINAL_RPM = 2400.0


@dataclass
class Sample:
    t_us: float
    ia: float
    ib: float
    vbus: float
    rpm: float
    regen_state: int


def load_capture(path: Path) -> list[Sample]:
    rows: list[Sample] = []
    with path.open(newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            rows.append(
                Sample(
                    t_us=float(r["time_us"]),
                    ia=float(r["ia_amp"]),
                    ib=float(r["ib_amp"]),
                    vbus=float(r["vbus_v"]),
                    rpm=float(r["rotor_rpm"]),
                    regen_state=int(r["regen_state"]),
                )
            )
    return rows


def reconstruct_ic(ia: float, ib: float) -> float:
    return -(ia + ib)


def clarke_park(ia: float, ib: float, ic: float, theta_e: float) -> tuple[float, float]:
    alpha = (2.0 / 3.0) * (ia - 0.5 * ib - 0.5 * ic)
    beta = (2.0 / 3.0) * (math.sqrt(3.0) / 2.0) * (ib - ic)
    c, s = math.cos(theta_e), math.sin(theta_e)
    d = alpha * c + beta * s
    q = -alpha * s + beta * c
    return d, q


def electrical_angle(sample_index: int, dt_s: float, rpm: float) -> float:
    mech_hz = rpm / 60.0
    return 2.0 * math.pi * POLE_PAIRS * mech_hz * (sample_index * dt_s)


def rms(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def thd_percent(phase_currents: list[float], fundamental_hz: float, fs: float) -> float:
    """Single-bin DFT THD estimate. Good enough for a bench check; the real
    number comes off the scope FFT. Harmonics 2..13 relative to fundamental."""
    n = len(phase_currents)
    if n == 0 or fs <= 0.0:
        return 0.0
    fund_mag = 0.0
    harm_sq = 0.0
    for h in range(1, 14):
        f = fundamental_hz * h
        if f >= fs / 2.0:
            break
        re = im = 0.0
        for k, v in enumerate(phase_currents):
            ang = 2.0 * math.pi * f * (k / fs)
            re += v * math.cos(ang)
            im -= v * math.sin(ang)
        mag = math.hypot(re, im) * 2.0 / n
        if h == 1:
            fund_mag = mag
        else:
            harm_sq += mag * mag
    if fund_mag < 1e-9:
        return 0.0
    return 100.0 * math.sqrt(harm_sq) / fund_mag


def torque_ripple_percent(iq_values: list[float]) -> float:
    if not iq_values:
        return 0.0
    mean = sum(iq_values) / len(iq_values)
    if abs(mean) < 1e-9:
        return 0.0
    return 100.0 * (max(iq_values) - min(iq_values)) / abs(mean)


def main() -> None:
    samples = load_capture(CAPTURE)
    if len(samples) < 2:
        print("capture too short")
        return

    dt_s = (samples[1].t_us - samples[0].t_us) * 1e-6
    fs = 1.0 / dt_s if dt_s > 0 else 0.0

    phase_a: list[float] = []
    iq_values: list[float] = []
    for idx, s in enumerate(samples):
        ic = reconstruct_ic(s.ia, s.ib)
        phase_a.append(s.ia)
        theta = electrical_angle(idx, dt_s, s.rpm)
        _, q = clarke_park(s.ia, s.ib, ic, theta)
        iq_values.append(q)

    mech_hz = (sum(s.rpm for s in samples) / len(samples)) / 60.0
    fund_hz = mech_hz * POLE_PAIRS

    print(f"samples          : {len(samples)}")
    print(f"sample rate      : {fs/1000.0:.2f} kHz")
    print(f"fundamental      : {fund_hz:.2f} Hz")
    print(f"phase A rms      : {rms(phase_a):.3f} A")
    print(f"current THD      : {thd_percent(phase_a, fund_hz, fs):.2f} %")
    print(f"torque ripple    : {torque_ripple_percent(iq_values):.2f} %")

    vbus_max = max(s.vbus for s in samples)
    vbus_min = min(s.vbus for s in samples)
    print(f"bus range        : {vbus_min:.2f} .. {vbus_max:.2f} V")

    states = sorted({s.regen_state for s in samples})
    print(f"regen states seen: {states}")

    if vbus_max > 60.0:
        print("WARN bus exceeded 60 V hard limit during capture")
    if thd_percent(phase_a, fund_hz, fs) > 8.0:
        print("WARN current THD above 8 percent target")


if __name__ == "__main__":
    main()

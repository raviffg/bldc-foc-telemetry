# bldc-foc-telemetry

Field-oriented control and regen braking for a 48 V BLDC driver, plus bench
analysis tooling for the 20 kHz shunt capture.

Control chain: Clarke -> Park -> PI(d,q) -> inverse Park -> SVPWM -> TIM1.
Regen forces negative Iq, bus governor folds the command back between 56 V
and 60 V so the chopper never fires.

## Build

Firmware is built in the parent project, this repo holds the control core and
analysis scripts only.

    python scripts/torque_ripple_analyzer.py

## Layout

- `firmware/foc_svpwm.c` transforms, PI loops, SVPWM, regen state machine
- `firmware/foc_svpwm.h` structs, TIM1 registers, public API
- `scripts/torque_ripple_analyzer.py` current THD and torque ripple from capture
- `data/regen_braking_bus_capture.csv` regen event, 50 us spacing

## Notes

Fixed point path is not wired yet, everything runs in float on the G4.
Clarke assumes balanced phases, see TODO before enabling regen in production.

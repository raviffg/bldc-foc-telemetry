# Versioning and compatibility

## Scope
This note records the design considerations for 48 V BLDC field-oriented control and regenerative-braking telemetry.

## Current project context
The repository currently implements phase-current sensing, Clarke/Park transforms, PI current loops, SVPWM timing, DC-bus overvoltage during regeneration. The main code paths are `firmware/foc_svpwm.c` and `scripts/torque_ripple_analyzer.py`. The principal engineering concerns are inverter switching, current reconstruction, rotor-angle alignment, bus-voltage governor, ADC timing, thermal limits.

## Design questions
- What inputs, units, coordinate conventions, and preconditions does this part of the system require?
- Which state is authoritative, and which values are derived?
- How are invalid, stale, incomplete, or physically implausible inputs represented?
- Which assumptions are supported by source code or measured evidence, and which remain provisional?
- What observable evidence allows a reviewer to reproduce the behavior?

## Recommended design record
Document the selected approach, alternatives considered, rationale, expected failure modes, and any measurable trade-offs. Keep numerical constants linked to source or calibration evidence; do not copy unexplained values into a second source of truth.

## Verification
Pair this design note with the relevant scenarios under `docs/verification/scenarios/`. A design decision is not considered verified merely because the code compiles.

## Status
Working reference. Confirm implementation details against the source before using this note as a hardware, clinical, or production specification.

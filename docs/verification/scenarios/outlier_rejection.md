# Outlier rejection and robust-estimator behavior

## Project context
48 V BLDC field-oriented control and regenerative-braking telemetry. This scenario exercises the project's core behavior: phase-current sensing, Clarke/Park transforms, PI current loops, SVPWM timing, DC-bus overvoltage during regeneration.

## Objective
Define a repeatable check for **outlier rejection and robust-estimator behavior** without assuming unverified hardware behavior or undocumented numerical limits.

## Relevant implementation
- Primary implementation: `firmware/foc_svpwm.c`
- Supporting implementation: `scripts/torque_ripple_analyzer.py`
- Main engineering concerns: inverter switching, current reconstruction, rotor-angle alignment, bus-voltage governor, ADC timing, thermal limits.

## Procedure
1. Record the source revision, toolchain/runtime versions, and configuration.
2. Prepare a controlled input fixture and preserve its original bytes.
3. Execute the relevant function or command using the documented interface.
4. Capture raw input, output, diagnostics, and run metadata.
5. Repeat with the boundary or fault condition relevant to this scenario.
6. Compare the result with an independently reviewed expectation.

## Expected evidence
- Reproducible command or test harness invocation.
- Input fixture identifier and checksum.
- Observed output and diagnostic log.
- Pass/fail rationale tied to explicit acceptance criteria.

## Acceptance criteria
The behavior is deterministic for identical inputs, invalid data is handled according to the documented contract, and no safety-relevant failure is silently converted into a valid result.

## Open items
- Add measured acceptance limits when bench evidence is available.
- Link a checked-in fixture and automated test when implemented.
- Record unresolved assumptions rather than treating them as verified facts.

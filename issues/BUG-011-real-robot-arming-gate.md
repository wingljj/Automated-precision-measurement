# BUG-011: [Safety] Real-Robot Mode - Movement Can Be Accepted Before Explicit Arming

**Reporter:** Codex safety review agent  
**Date:** 2026-05-30  
**Severity:** Critical  
**Reproducibility:** Always  
**GitHub:** https://github.com/wingljj/Automated-precision-measurement/issues/11

---

## Summary

The plugin can switch RoboDK into real-robot mode and accept physical movement commands before a separate readiness/arming state is established.

## Expected Behavior

Physical robot movement should be rejected until robot selection, connection/readiness, stop-clear state, speed configuration, and operator arming are all explicit and visible.

## Actual Behavior

The reviewed code path treated the real-robot checkbox/TCP mode as sufficient for the movement path once robot selection, stop state, and busy state checks passed.

## Reproduction Steps

**Prerequisites:** RoboDK station with at least one robot and the plugin loaded.

1. Select a robot in the Robot Pilot UI.
2. Enable real-robot mode with the checkbox or `U_UseRobot`.
3. Send an `E_T_*`, `E_R_*`, or `E_J_*` movement command before confirming a real-robot readiness state.
4. Observe that the movement path can accept the command if the basic checks pass.

### Minimal Reproduction

```text
C_<RobotName>
U_UseRobot
E_T_0_0_0_0_0_0
```

## Environment

| Detail | Value |
| --- | --- |
| Workspace | `E:\test\PluginExample_0529\PluginExample` |
| OS | Windows, PowerShell workspace |
| Loaded skill | `filing-bug-reports` |
| Relevant GitHub issue | #11 |

## Error Output

```text
N/A - review finding from control-flow inspection.
```

## Impact

This affects real-robot operation. A movement command can be accepted before an operator has a clear armed state. Workaround: validate in simulation, set speed immediately after enabling real mode, and keep hardware emergency stop active.

## Related Files

| File | Relevance |
| --- | --- |
| `formrobotpilot.cpp` | Real-robot mode, movement dispatch, stop handling |
| `formrobotpilot.h` | Real-robot state and worker members |

## Notes

The optimization pass added a conservative movement gate requiring speed confirmation, stop-clear state, and idle worker state before real movement is queued. A full hardware readiness/arming workflow remains open in GitHub issue #11.

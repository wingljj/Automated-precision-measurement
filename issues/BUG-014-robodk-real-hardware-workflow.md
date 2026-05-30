# BUG-014: [RoboDK UI] Real-Hardware Workflow - State Is Not Explicit Enough

**Reporter:** Codex RoboDK workflow review agent  
**Date:** 2026-05-30  
**Severity:** High  
**Reproducibility:** Always  
**GitHub:** https://github.com/wingljj/Automated-precision-measurement/issues/14

---

## Summary

The UI does not present a clear real-hardware workflow state for selected, connected, speed configured, armed, moving, completed, stopped, or faulted states.

## Expected Behavior

Operators should see command state and be prevented from issuing out-of-order real-hardware commands while connection, speed, or motion is pending.

## Actual Behavior

The reviewed UI relied mainly on log messages and left key controls available during asynchronous speed and movement operations.

## Reproduction Steps

**Prerequisites:** RoboDK plugin loaded with one or two robots.

1. Select a robot.
2. Enable real-robot mode.
3. Send speed and movement commands from the UI or TCP.
4. Observe that the operator-facing state is not presented as a guided workflow.

### Minimal Reproduction

```text
C_<RobotName>
U_UseRobot
M_Speed_100_20_100_20
E_T_0_0_0_0_0_0
```

## Environment

| Detail | Value |
| --- | --- |
| Workspace | `E:\test\PluginExample_0529\PluginExample` |
| OS | Windows, PowerShell workspace |
| Loaded skill | `filing-bug-reports` |
| Relevant GitHub issue | #14 |

## Error Output

```text
N/A - review finding from UI and command-flow inspection.
```

## Impact

This affects physical robot operation and can make users repeat commands or misinterpret a blocked RoboDK call as a frozen UI. Workaround: operate step-by-step from the log and avoid pipelining speed/movement commands.

## Related Files

| File | Relevance |
| --- | --- |
| `formrobotpilot.cpp` | UI state, logs, movement and speed command flow |
| `formrobotpilot.ui` | Controls requiring explicit workflow state |

## Notes

The optimization pass added logs for speed confirmation, motion start/completion, and disables planning/execution/mode controls while a worker motion is active. A complete guided state machine remains open in GitHub issue #14.

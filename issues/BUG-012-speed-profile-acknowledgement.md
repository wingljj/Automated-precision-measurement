# BUG-012: [Speed Safety] Speed Command - Movement Can Follow Before Speed Is Confirmed

**Reporter:** Codex speed-safety review agent  
**Date:** 2026-05-30  
**Severity:** Critical  
**Reproducibility:** Always  
**GitHub:** https://github.com/wingljj/Automated-precision-measurement/issues/12

---

## Summary

In real-robot mode, speed configuration was queued to a worker while the TCP caller received an immediate success response, allowing a following movement command to be accepted before speed application was confirmed.

## Expected Behavior

Real movement should require a valid, bounded, per-robot speed profile that has been acknowledged by the worker path.

## Actual Behavior

`SetSpeed()` returned success after queueing the worker call, and the TCP path returned `Speed set` immediately.

## Reproduction Steps

**Prerequisites:** RoboDK plugin running in real-robot mode.

1. Select a robot.
2. Enable real-robot mode.
3. Send `M_Speed_100_20_100_20`.
4. Immediately send an `E_*` movement command.
5. Observe that the movement command could be accepted without waiting for speed confirmation.

### Minimal Reproduction

```text
U_UseRobot
M_Speed_100_20_100_20
E_J_0_0_0_0_0_0
```

## Environment

| Detail | Value |
| --- | --- |
| Workspace | `E:\test\PluginExample_0529\PluginExample` |
| OS | Windows, PowerShell workspace |
| Loaded skill | `filing-bug-reports` |
| Relevant GitHub issue | #12 |

## Error Output

```text
N/A - review finding from command acknowledgement flow.
```

## Impact

This affects physical robot safety because a stale or default speed profile can be used for motion. Workaround before the optimization: wait for operator-visible confirmation and use conservative speed values.

## Related Files

| File | Relevance |
| --- | --- |
| `formrobotpilot.cpp` | TCP speed command, speed validation, real movement dispatch |
| `robotworker.cpp` | Worker speed application |

## Notes

The optimization pass separated speed confirmation from movement completion and blocks real movement until the relevant worker reports speed configured. Hardware-level speed failure reporting still depends on the RoboDK API behavior.

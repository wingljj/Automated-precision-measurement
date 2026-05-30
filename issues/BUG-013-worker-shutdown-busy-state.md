# BUG-013: [Threading] Worker Shutdown - Threads Can Outlive Controller State

**Reporter:** Codex threading review agent  
**Date:** 2026-05-30  
**Severity:** High  
**Reproducibility:** Often  
**GitHub:** https://github.com/wingljj/Automated-precision-measurement/issues/13

---

## Summary

Worker threads can remain active during blocking RoboDK or hardware calls while the UI/controller object is shutting down.

## Expected Behavior

Shutdown should have deterministic ownership of in-flight commands, busy state, cancel/timeout behavior, and late callbacks.

## Actual Behavior

The destructor waits up to 5 seconds for each worker thread and then detaches the thread parent if it does not stop. Motion busy state and speed failures also shared the same completion channel before this review.

## Reproduction Steps

**Prerequisites:** Real robot or RoboDK communication path that can block in `MoveJ`.

1. Start a real-robot movement.
2. Close the plugin window while the worker is still inside the RoboDK call.
3. Observe that the destructor can time out and detach the worker thread.

### Minimal Reproduction

```text
U_UseRobot
M_Speed_100_20_100_20
E_T_0_0_0_0_0_0
```

Close the Robot Pilot window while the command is still running.

## Environment

| Detail | Value |
| --- | --- |
| Workspace | `E:\test\PluginExample_0529\PluginExample` |
| OS | Windows, PowerShell workspace |
| Loaded skill | `filing-bug-reports` |
| Relevant GitHub issue | #13 |

## Error Output

```text
Worker thread 1 did not stop within timeout
Worker thread 2 did not stop within timeout
```

## Impact

This affects real-hardware sessions and shutdown/unload paths. Workaround: stop motion and wait for completion before closing the plugin.

## Related Files

| File | Relevance |
| --- | --- |
| `formrobotpilot.cpp` | Thread creation, shutdown, busy flags |
| `robotworker.cpp` | Blocking RoboDK calls and worker signals |

## Notes

The optimization pass now requests stop before thread shutdown and separates speed confirmation from motion completion. A full deterministic cancellation model remains open in GitHub issue #13.

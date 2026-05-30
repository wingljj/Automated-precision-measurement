# Optimization Review

Review date: 2026-05-30

This review records the follow-up optimization pass after the GitHub issue backlog was created.

## Agent Allocation

| Agent | Responsibility | Result |
| --- | --- | --- |
| GitHub Triage Agent | Open actionable GitHub issues from the review backlog | Created issues #1 through #10 |
| Protocol Agent | Improve movement command acknowledgement semantics | Immediate move responses now say accepted/rejected; worker completion reports completed/failed |
| Safety Config Agent | Remove hard-coded radar controller endpoint and command writes | Radar host, port, resume command, and stop command are read from `config.ini`; writes check socket state |
| UI Validation Agent | Harden manual planning/execution input paths | Button paths now use one checked numeric parser instead of unchecked `toDouble()` |
| Code Quality Agent | Remove obvious dead code from touched paths | Removed the unused `PlanningPosition()` mode calculation and made execute failure visible in the log |

## GitHub Issues Created

- #1 `[P0] APM-001 Define a hard-stop path for active real-robot motion`
- #2 `[P0] APM-002 Serialize RoboDK API access or prove thread safety`
- #3 `[P0] APM-003 Add authentication and schema validation to TCP control protocol`
- #4 `[P1] APM-004 Return movement completion acknowledgements only after motion completes`
- #5 `[P1] APM-005 Make radar controller address and stop policy configurable`
- #6 `[P1] APM-006 Vendor or document RoboDK interface dependency`
- #7 `[P2] APM-007 Remove absolute or machine-specific release assumptions`
- #8 `[P2] APM-008 Strengthen numeric validation in UI button paths`
- #9 `[P3] APM-009 Fix or remove unused variables and dead branches`
- #10 `[P3] APM-010 Keep generated and local artifacts out of commits`

## Changes Reviewed

### Movement acknowledgement

The TCP command path no longer replies with `* move executed` immediately after queueing a real-robot worker move. The immediate reply is now `accepted` or `rejected`. `onMoveCompleted()` writes a separate `Move completed: ...` or `Move failed: ...` message to the current client socket.

Residual risk:
The protocol still has no request id, so concurrent or pipelined clients cannot reliably correlate completion messages. This remains tracked by #4.

### Radar configuration

Radar controller defaults remain compatible with the original deployment:

```ini
[RadarController]
host=169.254.0.66
port=7
resumeCommand=Light:3;
stopCommand=Light:5;
```

The plugin now reads these values from `config.ini` before opening the radar socket. Radar writes go through `SendRadarCommand()`, which checks connection state and logs when a command cannot be sent.

Residual risk:
The radar protocol is still string-based and should be wrapped in a typed adapter before wider deployment. This remains tracked by #5.

### UI numeric validation

Planning and execute button handlers now call `ReadTargetInputs()`. All six fields are checked with `toDouble(&ok)`, and invalid values are rejected before planning or movement.

Residual risk:
TCP parsing is better than before, but a future schema-based protocol should centralize all command validation. This remains tracked by #3.

## Review Findings After Optimization

No new high-severity regressions were identified in the touched code. The important remaining risks are intentionally left open:

- Active real-robot hard stop needs API/hardware confirmation (#1).
- RoboDK API thread-safety still needs a clear owner-thread model or documented proof (#2).
- TCP authentication and request ids are not implemented yet (#3 and #4).
- Clean-checkout builds still require the external `../robodk_interface` dependency (#6).

## Verification

- Text/static whitespace check passed for touched Markdown and C++ files.
- A clean compile is still blocked in this workspace by missing `../robodk_interface` headers and sources.

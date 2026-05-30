# Safety and Threading Review

Review date: 2026-05-30

This review covers the second focused pass requested for physical robot safety, worker/thread management, speed safety, and RoboDK real-hardware workflow clarity.

## Agent Allocation

| Agent | Focus | GitHub issue | Optimization result |
| --- | --- | --- | --- |
| Mechanical-arm Safety Agent | Real-robot arming and stop gates | #11 | Added a real-motion gate that rejects movement when stop is active, speed is not confirmed, or the target worker is already moving |
| Speed-safety Agent | Speed validation and acknowledgement | #12 | Added worker speed confirmation signals and blocks real movement until the target robot speed flag is confirmed |
| Thread Lifecycle Agent | Worker shutdown and busy-state ownership | #13 | Requested stop before worker shutdown and separated speed confirmation from movement completion |
| RoboDK Workflow Agent | Real-hardware UI process clarity | #14 | Added workflow logs and disables planning/execution/mode controls while worker motion is active |

## GitHub Issues Created

- #11 `[P0] APM-011 Add a real-robot arming gate before physical motion`
- #12 `[P0] APM-012 Require speed profile acknowledgement before real-robot moves`
- #13 `[P1] APM-013 Make worker shutdown and busy-state ownership deterministic`
- #14 `[P1] APM-014 Add a guided RoboDK real-hardware workflow state in the UI`

Mirrored local bug reports are stored in `issues/BUG-011-*` through `issues/BUG-014-*`.

## Changes Reviewed

### Real-robot motion gate

Real-robot movement now goes through `CanStartRealMove()` before a worker command is queued. The gate checks:

- valid robot item
- stop state is clear
- speed has been confirmed for the corresponding worker
- the worker is not already moving

Rejected commands are logged and shown to the operator before the movement request reaches the worker.

### Speed confirmation

`RobotWorker::executeSetSpeed()` now emits `speedConfigured()` instead of reusing the movement completion signal. `FormRobotPilot` records speed confirmation separately for Robot1/worker1 and Robot2/worker2. Real movement is blocked until the relevant flag is set.

Speed authorization is cleared when:

- real-robot mode is enabled
- robot selection changes
- speed validation fails
- manual stop or radar stop is triggered

### Stop and radar behavior

Manual stop and radar stop clear speed authorization so the operator must set speed again after recovering. The distance-timeout stop path now also sets the local stop flag and notifies workers, instead of only sending the radar command.

### UI workflow behavior

The UI log now records speed command pending/confirmed state, motion start, and motion completion/failure. Planning, execution, and real/sim mode controls are disabled while a worker motion is active and restored when all worker motion flags are clear.

## Remaining Review Findings

- The current real-robot gate is not a complete hardware arming model. Connection readiness, operator confirmation, and fault state should become explicit workflow states (#11, #14).
- RoboDK API thread-safety is still unresolved. Worker calls still use RoboDK item/API objects from worker threads (#2, #13).
- `setSpeed()` does not expose a rich hardware acknowledgement in the current interface, so the worker confirmation means the call returned, not that the physical controller independently verified the speed (#12).
- TCP responses still lack request ids and authentication (#3, #4).
- Clean builds still require the external `../robodk_interface` dependency (#6).

## Verification

- `git diff --check -- formrobotpilot.cpp formrobotpilot.h robotworker.cpp robotworker.h` passed.
- A full compile is still blocked in this workspace by the missing `../robodk_interface` headers and sources documented in #6.

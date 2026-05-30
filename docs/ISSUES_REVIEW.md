# Agent Team Review and Issue Backlog

Review date: 2026-05-30

This document records the review split across five virtual agent teams. The goal is to make the current risks visible, turn them into actionable issues, and give the next developer a clear first pass at hardening the RoboDK robot-pilot plugin.

## Agent Teams

| Team | Scope | Files Reviewed |
| --- | --- | --- |
| Safety Controls | Real robot mode, emergency stop, speed limits, collision checks | `formrobotpilot.cpp`, `robotworker.cpp` |
| Performance and Threading | UI responsiveness, worker routing, render frequency, blocking calls | `formrobotpilot.cpp`, `robotworker.cpp` |
| Network and Protocol | TCP command surface, parsing, command acknowledgement, model loading | `formrobotpilot.cpp` |
| Build and Release | qmake, Visual Studio, ignored artifacts, external RoboDK interface | `PluginExample.pro`, `PluginExample.vcxproj`, `.gitignore` |
| Documentation and Operability | Onboarding, runtime assumptions, known limitations | `README.md`, `docs/ISSUES_REVIEW.md` |

## Executive Review

The project is a RoboDK plugin with a Qt widget for robot selection, target planning, direct movement, speed setting, radar-assisted stop state, and a localhost TCP command protocol. The most important recent improvement is that real-robot motion no longer broadcasts every move signal to both workers; Robot1/single-robot commands now route to worker1 and Robot2 commands route to worker2.

The main remaining risk is that `RoboDK*` and `Item` API calls are made from both UI and worker threads. Even with worker routing fixed, the project should confirm RoboDK API thread-safety or serialize all RoboDK calls through one owner thread. The emergency-stop implementation also prevents queued/future moves but does not prove that a blocking `MoveJ` already in progress is interrupted at the robot controller.

## Issue Backlog

GitHub issue mapping:

| ID | GitHub Issue | Status After First Optimization Pass |
| --- | --- | --- |
| APM-001 | #1 | Open; requires RoboDK/robot-controller hard-stop API confirmation |
| APM-002 | #2 | Open; requires threading contract or command executor design |
| APM-003 | #3 | Open; authentication and schema migration not implemented |
| APM-004 | #4 | Partially optimized; accepted/rejected and completion messages added, request ids still needed |
| APM-005 | #5 | Partially optimized; radar endpoint/commands configurable, typed adapter still needed |
| APM-006 | #6 | Open; external dependency still required |
| APM-007 | #7 | Open; project output paths still need release redesign |
| APM-008 | #8 | Optimized in UI button paths; future protocol schema work remains under #3 |
| APM-009 | #9 | Optimized in touched paths |
| APM-010 | #10 | Monitoring; ignore policy is in place |

### APM-001: Define a hard-stop path for active real-robot motion

Priority: P0

Evidence:
- `Slot_BtnStop()` sets `m_Stop` and updates worker stop flags, but it does not call a RoboDK or controller stop API for an already-running motion: `formrobotpilot.cpp:1204`.
- `RobotWorker::executeMoveCartesian()` checks `m_stopFlag` before `MoveJ`, then blocks inside `robot->MoveJ(pose)`: `robotworker.cpp:48`, `robotworker.cpp:81`.
- `RobotWorker::executeMoveJoints()` has the same pattern: `robotworker.cpp:93`, `robotworker.cpp:122`.

Impact:
An emergency stop triggered while `MoveJ` is already executing may not interrupt the physical robot. The current flag is useful for preventing later queued work, but it should not be treated as a safety-rated stop.

Recommended fix:
Identify the supported RoboDK or robot-driver stop API for active motions and call it on the correct thread. Keep the physical safety circuit/radar controller as the primary safety layer. Add a manual acceptance test that starts a slow real-robot move and verifies stop latency.

### APM-002: Serialize RoboDK API access or prove it is thread-safe

Priority: P0

Evidence:
- Worker threads call `robot->MoveJ`, `robot->setSpeed`, and `m_rdk->setCollisionActive`: `robotworker.cpp:39`, `robotworker.cpp:73`, `robotworker.cpp:81`.
- The UI thread also calls `RDK->Render`, `Robot->Joints`, `Robot->Pose`, `Robot->Connect`, and `RDK->setRunMode`: `formrobotpilot.cpp:528`, `formrobotpilot.cpp:1914`, `formrobotpilot.cpp:1942`.

Impact:
If the RoboDK plugin API is not thread-safe, intermittent freezes, missed speed updates, invalid item state, or crashes can still occur.

Recommended fix:
Create a single RoboDK command executor that owns all `RDK` and `Item` calls, or protect calls with a command queue and explicit sequencing. Document the chosen threading contract in code.

### APM-003: Add authentication and schema validation to the TCP control protocol

Priority: P0

Evidence:
- The server listens on localhost port 8866: `formrobotpilot.cpp:96`.
- Any local process can send robot movement, real-robot mode, speed, model loading, and project opening commands through `onReadyRead()`: `formrobotpilot.cpp:635`.

Impact:
Loopback is better than a public listener, but malware or an unintended local tool can still command a real robot.

Recommended fix:
Add a shared secret/token or local named-pipe style permission boundary. Reject commands unless authenticated. Consider a JSON protocol with explicit command names, version, request id, units, and range constraints.

### APM-004: Return movement completion acknowledgements only after motion completes

Priority: P1

Evidence:
- TCP handlers write `"Tool move executed"`, `"Reference move executed"`, or `"Joints move executed"` immediately after queuing a worker move: `formrobotpilot.cpp:703`, `formrobotpilot.cpp:724`, `formrobotpilot.cpp:731`.
- Actual completion is later signalled by `RobotWorker::moveCompleted`: `robotworker.cpp:83`, `robotworker.cpp:85`.

Impact:
Clients may start the next measurement or movement before the robot actually reaches the target.

Recommended fix:
Introduce request ids and send `accepted` immediately, then `completed` or `failed` from `onMoveCompleted()`.

### APM-005: Make radar controller address and stop policy configurable

Priority: P1

Evidence:
- The radar controller is hard-coded as `169.254.0.66:7`: `formrobotpilot.cpp:112`.
- Stop outputs use string commands such as `Light:3;` and `Light:5;` without a typed protocol wrapper: `formrobotpilot.cpp:1220`, `formrobotpilot.cpp:1237`.

Impact:
Deployment requires source edits for different hardware, and command semantics are hidden in code.

Recommended fix:
Move host, port, thresholds, stop policy, and light commands into `config/config.ini`. Validate connection state before writes and surface connection failure in the UI.

### APM-006: Vendor or document the RoboDK interface dependency

Priority: P1

Evidence:
- `PluginExample.pro` expects `../robodk_interface/iitem.h`, `../robodk_interface/irobodk.h`, and related sources: `PluginExample.pro:117`.
- A local build attempt fails if that sibling directory is missing.

Impact:
Fresh checkout builds fail unless developers already know where to place the RoboDK plugin interface.

Recommended fix:
Add the interface as a submodule, vendor it under the repository, or document the exact directory layout and source in `README.md`.

### APM-007: Remove absolute or machine-specific release assumptions from project files

Priority: P2

Evidence:
- qmake outputs directly to `C:/RoboDK/bin/plugins`: `PluginExample.pro:26`.
- The Visual Studio project also writes to `C:\RoboDK\bin\plugins\`: `PluginExample.vcxproj:24`.

Impact:
Developers without that RoboDK path may fail to build or may accidentally overwrite a production plugin.

Recommended fix:
Use a local build output by default and provide an explicit install/copy step for RoboDK plugin deployment.

### APM-008: Strengthen numeric validation in UI button paths

Priority: P2

Evidence:
- Button handlers convert UI text with `toDouble()` without checking conversion results: `formrobotpilot.cpp:1847`, `formrobotpilot.cpp:1876`.

Impact:
Invalid UI input silently becomes zero, which can create unexpected target poses.

Recommended fix:
Use `toDouble(&ok)` for each field and show a validation error before movement or planning.

### APM-009: Fix or remove unused variables and dead branches

Priority: P3

Evidence:
- `moveSuccess` is assigned but not used in `on_btnExecute_clicked()`: `formrobotpilot.cpp:1882`.
- `mode` is computed but not used in `PlanningPosition()`: `formrobotpilot.cpp:1556`.

Impact:
Dead code makes future motion-mode fixes harder to reason about.

Recommended fix:
Remove unused variables or route them into status reporting.

### APM-010: Avoid checked-in generated or local artifacts

Priority: P3

Evidence:
- `.gitignore` now ignores generated Makefiles, `build/`, `debug/`, `release/`, `ui_*.h`, and local agent directories.

Impact:
The repository is cleaner now, but contributors should keep using the ignore policy.

Recommended fix:
Keep build artifacts out of commits. Add CI or a pre-commit check later if the repo grows.

## Suggested Review Gates

Before connecting a real robot:

1. Build from a clean checkout with the expected RoboDK interface dependency installed.
2. Run all motion commands in RoboDK simulation only.
3. Verify speed command order: set speed, then send movement, then inspect robot motion settings.
4. Verify emergency stop latency with an intentionally slow, low-risk move.
5. Verify TCP clients wait for a completion event instead of trusting the immediate accept response.
6. Confirm the physical emergency-stop chain works independently of this plugin.

## Proposed Milestones

Milestone 1: Safe Control Baseline
- APM-001
- APM-002
- APM-003
- APM-004

Milestone 2: Deployable Build
- APM-006
- APM-007
- README build verification

Milestone 3: Operator Hardening
- APM-005
- APM-008
- APM-009
- Better UI status reporting

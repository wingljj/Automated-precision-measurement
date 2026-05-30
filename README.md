# Automated Precision Measurement

RoboDK Qt plugin for automated precision-measurement workflows with robot selection, target planning, direct robot jogging, speed control, radar-assisted stop state, and a local TCP command interface.

The current plugin target is `PluginExample.dll`, loaded by RoboDK from the RoboDK plugin directory.

## What It Does

- Opens a Robot Pilot dock/window inside RoboDK.
- Selects one robot or a two-robot pair by name.
- Plans target poses and sorts target execution order through `TrajectorySortLib`.
- Executes Cartesian and joint movements in simulation or real-robot mode.
- Sets robot speed and acceleration with validation and conservative safety caps.
- Exposes a localhost TCP command server on port `8866`.
- Talks to a radar/stop controller at `169.254.0.66:7`.

## Repository Layout

```text
.
|-- PluginExample.pro           # qmake project
|-- PluginExample.vcxproj       # Visual Studio / Qt VS Tools project
|-- pluginexample.*             # RoboDK plugin entry point
|-- formrobotpilot.*            # Main Robot Pilot UI and command handling
|-- robotworker.*               # Worker object for queued robot operations
|-- formrobotpilot.ui           # Qt Designer UI
|-- include/pose_sort.h         # Trajectory sorting API header
|-- lib/TrajectorySortLib.lib   # Trajectory sorting library
|-- resources/                  # Plugin icons
|-- resources1.qrc              # Qt resource file
|-- docs/ISSUES_REVIEW.md       # Review findings and issue backlog
|-- docs/OPTIMIZATION_REVIEW.md # First optimization review
|-- docs/SAFETY_THREADING_REVIEW.md # Safety/threading follow-up review
|-- issues/                     # Mirrored bug reports for GitHub issues
`-- manifest.xml                # RoboDK plugin manifest
```

## Prerequisites

- Windows development environment.
- RoboDK installed, normally under `C:\RoboDK`.
- Qt 5 with Widgets, Network, Core, Gui, and SerialPort modules.
- Either Qt Creator/qmake or Visual Studio with Qt VS Tools.
- RoboDK plugin interface sources available as a sibling directory:

```text
PluginExample/
robodk_interface/
```

The qmake project references these files through `../robodk_interface`:

- `iitem.h`
- `irobodk.h`
- `iapprobodk.h`
- `robodktypes.h`
- `robodktools.h`
- `robodktypes.cpp`
- `robodktools.cpp`

Without that sibling directory, a clean build will fail at include resolution.

## Build

### qmake

```powershell
qmake PluginExample.pro
mingw32-make
```

For MSVC-based Qt kits, use the matching Visual Studio developer prompt and build with the generated Makefile or Qt Creator.

### Visual Studio

Open `PluginExample.vcxproj` with Visual Studio and Qt VS Tools installed. The project currently targets Qt `5.15.2_msvc2019_64` and outputs to the RoboDK plugin directories:

- Release: `C:\RoboDK\bin\plugins\`
- Debug: `C:\RoboDK\bind\plugins\`

Adjust those paths if RoboDK is installed elsewhere.

## Run in RoboDK

1. Build the plugin DLL.
2. Copy or build `PluginExample.dll` into the RoboDK plugin directory.
3. Start RoboDK.
4. Load plugins with one of these options:

```powershell
C:\RoboDK\bin\RoboDK.exe -PLUGINSLOAD
C:\RoboDK\bin\RoboDK.exe -PLUGINLOAD=C:\RoboDK\bin\plugins\PluginExample.dll
```

5. Open the Robot Pilot form from the plugin toolbar/menu.
6. Select a robot before planning or moving.
7. Validate all motions in simulation before enabling real-robot mode.

## Runtime Configuration

The plugin reads and writes:

```text
<RoboDK application dir>/config/config.ini
```

Current keys:

```ini
[Distance]
distance=600

[EStop]
stoptype=1

[RadarComMap]
ComNumRobot1=
ComNumRobot2=

[RadarController]
host=169.254.0.66
port=7
resumeCommand=Light:3;
stopCommand=Light:5;
```

`RadarController` values default to the controller endpoint and light commands used by the original deployment. Override them in `config.ini` for each cell instead of editing source code.

## TCP Command Protocol

The plugin listens on:

```text
127.0.0.1:8866
```

Commands are underscore-delimited. End commands with `\n` or `\r\n` when possible.

### Robot Selection

```text
C_<RobotName>
C_<Robot1Name>_<Robot2Name>
```

### Mode

```text
U_UseRobot
U_Simulate
```

Any `U_*` command other than `U_UseRobot` switches back to simulation mode.

### Movement

```text
E_T_<x>_<y>_<z>_<rx>_<ry>_<rz>     # Tool-frame Cartesian move, active robot
E_T1_<x>_<y>_<z>_<rx>_<ry>_<rz>    # Tool-frame Cartesian move, Robot1
E_T2_<x>_<y>_<z>_<rx>_<ry>_<rz>    # Tool-frame Cartesian move, Robot2
E_R_<x>_<y>_<z>_<rx>_<ry>_<rz>     # Reference-frame Cartesian move
E_J_<j1>_<j2>_<j3>_<j4>_<j5>_<j6>  # Joint move
```

Movement commands return an immediate `accepted` or `rejected` response. Real-robot worker completion is reported later as `Move completed: ...` or `Move failed: ...`.

### Planning

```text
P_T_<x>_<y>_<z>_<rx>_<ry>_<rz>
```

### Speed

```text
M_Speed_<linearSpeed>_<jointSpeed>_<linearAccel>_<jointAccel>
```

In real-robot mode, the immediate TCP response is `Speed command accepted`. The worker then reports speed confirmation separately. Real movement is rejected until the relevant robot/worker has confirmed speed after entering real mode, changing robot selection, or recovering from stop.

The current hard caps are:

- Linear speed: `2000`
- Joint speed: `360`
- Linear acceleration: `10000`
- Joint acceleration: `720`

Tune these limits for the deployed robot model before production use.

### Read Position

```text
G_T_Pos
G_T1_Pos
G_T2_Pos
G_J_Pos
G_J1_Pos
G_J2_Pos
G_All_Pos
```

### Model and Project Commands

```text
M_Load_<path>
M_OpenProject_<path>
M_Hide_<itemName>
M_Show_<itemName>
M_GetPose_<itemName>
M_Rotate_<itemName>_<rx>_<ry>_<rz>
M_Move_<itemName>_<x>_<y>_<z>
M_SetPose_<itemName>_<x>_<y>_<z>_<rx>_<ry>_<rz>
M_Clear
M_List
```

## Safety Notes

- Treat this software as an operator aid, not as a safety-rated controller.
- Always validate movement in RoboDK simulation first.
- After enabling real-robot mode, set speed before sending any physical movement command.
- Manual stop, radar stop, robot reselection, and real-mode entry clear the speed authorization state.
- Keep physical emergency-stop hardware in the loop for real robots.
- The current TCP server is bound to localhost, but it does not authenticate clients.
- Immediate movement responses mean the command was accepted or rejected. Completion is reported separately when the worker finishes.
- Planning, execute, and real/sim mode controls are disabled while worker motion is active.
- Active `MoveJ` interruption depends on RoboDK/robot-controller behavior and needs a dedicated hard-stop implementation.

## Known Issues

See `docs/ISSUES_REVIEW.md` and `docs/SAFETY_THREADING_REVIEW.md` for the current review backlog. Highest-priority items:

- Define and test a real hard-stop path for active robot motion.
- Serialize or otherwise prove safe RoboDK API access across UI and worker threads.
- Add a complete real-robot arming/workflow state around RoboDK physical communication.
- Improve hardware-level speed acknowledgement beyond worker-return confirmation.
- Add authentication and request/response ids to the TCP protocol.
- Vendor or document the required `robodk_interface` dependency.

## Development Notes

- Build outputs, generated Qt files, local IDE state, and local agent tooling are ignored by `.gitignore`.
- `TrajectorySortLib.lib` is checked in because the plugin links against it directly.
- The generated `ui_formrobotpilot.h` should be regenerated by Qt tooling, not edited manually.

## License

See `LICENSE.md`.

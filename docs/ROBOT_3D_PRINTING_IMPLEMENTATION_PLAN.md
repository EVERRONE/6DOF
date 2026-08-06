# Robot 3D Printing Implementation Plan

**Version:** 1.0  
**Date:** March 24, 2026  
**Status:** Draft for implementation  
**Project scope:** Extend the existing 6DoF robot arm into a reliable planar 3D-printing system with G-code interpretation, temperature control, extrusion control, print safety, and calibration workflows.

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Current Baseline](#current-baseline)
3. [Target Architecture](#target-architecture)
4. [Scope and Non-Goals](#scope-and-non-goals)
5. [Slicer Strategy](#slicer-strategy)
6. [Hardware Plan](#hardware-plan)
7. [Electrical and Safety Plan](#electrical-and-safety-plan)
8. [Firmware Strategy](#firmware-strategy)
9. [Software Architecture Plan](#software-architecture-plan)
10. [G-code Interpretation Plan](#g-code-interpretation-plan)
11. [Calibration Plan](#calibration-plan)
12. [Execution and Synchronization Plan](#execution-and-synchronization-plan)
13. [UI and Operator Workflow Plan](#ui-and-operator-workflow-plan)
14. [Implementation Phases](#implementation-phases)
15. [Validation and Test Plan](#validation-and-test-plan)
16. [Acceptance Criteria](#acceptance-criteria)
17. [Risks and Mitigations](#risks-and-mitigations)
18. [Reference Material](#reference-material)

---

## Executive Summary

### Goal

Turn the current robot arm platform into a practical 3D printer that:

- interprets standard slicer-generated G-code,
- heats and regulates a hotend and heated bed,
- controls extrusion and fans,
- converts print toolpaths into robot TCP motion,
- prints safely and repeatably.

### Recommended System Architecture

Use a **dual-controller architecture**:

- **Teensy 4.1** remains the real-time motion controller for the 6 robot joints.
- **SKR Mini E3 V2.0** becomes the printhead controller for:
  - hotend heater,
  - hotend thermistor,
  - heated bed,
  - bed thermistor,
  - extruder stepper,
  - part cooling fan,
  - hotend heatsink fan.
- The **existing React/Web Serial host** becomes the supervisory print host that:
  - parses G-code,
  - manages modal state,
  - transforms slicer coordinates into robot coordinates,
  - uploads robot trajectories to the Teensy,
  - streams printhead commands to the SKR,
  - enforces preflight and safety checks.

### Why this architecture

- The current robot stack already has strong IK, Cartesian planning, and queue-based execution.
- The SKR board is a good fit for temperature, fans, bed, and extruder control.
- Trying to move the 6DoF arm into standard printer firmware is the wrong abstraction and would waste time.
- This approach keeps each controller responsible for the domain it is already good at.

---

## Current Baseline

### Already present in this repository

- Stable inverse kinematics stack.
- URDF-based forward and inverse kinematics.
- Cartesian planning and path interpolation.
- Queue-based joint trajectory execution via `TQ`.
- Browser-based control app over Web Serial.
- Teensy firmware with step generation, homing, queue upload, and motion diagnostics.

### Existing local integration points

- `docs/KINEMATICS.md`
  - documents `pose_lock` and queue-driven Cartesian execution.
- `robot-arm-control/src/services/cartesian/TrajectoryExecutionService.ts`
  - uploads queue points to the Teensy.
- `robot-arm-control/src/communication/SerialManager.ts`
  - host-side serial abstraction for the arm controller.
- `firmware/README.md`
  - documents the current `TQ` protocol.

### Missing today

- No printhead controller integration.
- No G-code parser or modal interpreter.
- No concept of extrusion state.
- No hotend or heated bed control in the host.
- No print job coordinator.
- No calibration workflow for bed plane, nozzle TCP, or print coordinate frames.
- No print-specific safety layer.

---

## Target Architecture

## Control topology

```text
Slicer G-code
    |
    v
Robot Arm Web App / Print Host
    |- G-code parser
    |- modal state machine
    |- print planner
    |- coordinate transforms
    |- job state machine
    |- safety interlocks
    |
    +--> Teensy 4.1 (6-axis arm motion)
    |      |- homing
    |      |- IK-driven queue execution
    |      |- motion diagnostics
    |
    +--> SKR Mini E3 V2.0 (print process control)
           |- extruder stepper
           |- hotend heater + thermistor
           |- heated bed + thermistor
           |- fans
           |- printer-side thermal safety
```

## Mechanical operating model

Version 1 should behave like a **planar 3-axis printer carried by a robot arm**:

- The slicer generates ordinary layer-by-layer planar G-code.
- The robot holds a fixed nozzle orientation during printing.
- J4/J5/J6 maintain tool orientation and cable-safe posture rather than performing 5-axis deposition.

This is the correct first milestone. Non-planar or 5-axis printing is a later extension, not a first implementation target.

---

## Scope and Non-Goals

### In scope for V1

- Planar 3D printing with fixed nozzle orientation.
- Single hotend, single extruder.
- Heated bed support.
- Part cooling and hotend fan control.
- Host-side parsing of common slicer G-code.
- Host-side conversion from G-code XYZ to robot TCP poses.
- Host-side print job management.
- Manual or semi-automatic bed-plane calibration.
- Safety interlocks and staged bring-up.

### Explicit non-goals for V1

- Native 5-axis slicing.
- Non-planar deposition.
- Toolchanging.
- Multi-material.
- Perfect pause/resume from arbitrary mid-print state.
- Closed-loop extrusion.
- Full printer firmware replacement for robot motion.

---

## Slicer Strategy

## 1. Selection criteria

The slicer must be suitable for a robot-driven printer host, not just for a normal gantry printer.

### Required slicer capabilities

- Custom printer profile creation.
- Custom bed dimensions and origin.
- Custom start and end G-code.
- Clean standard G-code output.
- Single-extruder FFF workflow.
- Easy control over temperatures, retraction, speeds, and fan behavior.
- Predictable output that is simple for the host parser to validate.

### Strongly preferred capabilities

- Custom bed model or custom bed shape support.
- Macro or placeholder support in custom G-code.
- G-code preview that makes debugging easier.
- Minimal reliance on printer-vendor-specific commands.

## 2. Recommended slicer for V1: PrusaSlicer

### Recommendation

Use **PrusaSlicer** as the primary slicer for V1.

### Why PrusaSlicer is the best fit here

- Official documentation shows it supports custom bed models, which is useful when the slicer profile must reflect your actual print bed and bed origin.
- Official documentation shows it has a macro language for conditional and calculated custom G-code, which is valuable for building a controlled robot-specific start and end sequence.
- Official documentation shows its G-code viewer understands enriched comments and that PrusaSlicer itself adds comments such as `;TYPE:` and `;LAYER_CHANGE`, which can help debugging, preview, and future host-side semantic parsing.
- It is a good fit for creating a clean custom machine profile without assuming the machine itself is doing the robot kinematics.

### Why it is preferred over Cura for this project

This is an engineering judgment based on the official docs:

- Cura is clearly viable for robot printing and is used in official RoboDK examples.
- However, PrusaSlicer exposes the bed-model workflow and macro/custom G-code workflow more directly for a custom machine profile, which matches this project better.
- For this robot architecture, predictable and customizable output is more important than marketplace integrations or brand-specific printer presets.

## 3. Secondary supported slicer: Cura

### Recommendation

Support **Cura** as a fallback and interoperability option.

### Why Cura should still be supported

- RoboDK’s official robot-printing documentation explicitly demonstrates a robot 3D-printing workflow using Cura and a `Custom FFF Printer`.
- Cura officially supports machine start and end G-code, as well as extruder start and end G-code.
- If a collaborator already uses Cura, it is reasonable to accept Cura-generated G-code as long as it passes the host validator.

### When to choose Cura

- When the operator is already experienced with Cura.
- When you want quick adoption with familiar FFF workflows.
- When you want to align with RoboDK-style robot-printing examples.

## 4. Third option: OrcaSlicer

### Recommendation

Treat **OrcaSlicer** as an advanced alternative, not the primary V1 baseline.

### Why OrcaSlicer is relevant

- Its official wiki documents machine G-code hooks.
- Its official wiki documents custom printable space settings, including custom STL bed shapes, origin, excluded areas, and printable height.
- It offers a lot of tuning functionality that may become useful later.

### Why it is not the primary choice

- It is more feature-dense and easier to over-configure.
- Many common Orca workflows are tuned around modern printer ecosystems rather than a custom robot host stack.
- For V1, the project benefits more from a narrower and more deterministic profile baseline.

## 5. Slicer profile policy for this project

Create one official machine preset:

- `6DoF Robot Planar Printer V1`

### Machine profile rules

- Bed size in the slicer must match the **actual printable bed area**, not the full robot reach.
- The slicer coordinate origin must match the bed-frame convention expected by the host.
- Use single extruder only.
- Use the actual nozzle diameter installed on the toolhead.
- Use the actual filament diameter.
- Heated bed must reflect the real hardware.
- Printer flavor should match the SKR firmware assumptions.

### Start G-code rules

The machine start G-code must remain minimal.

Allowed pattern:

- `G21`
- `G90`
- `M82` or `M83`
- `M104`
- `M109`
- `M140`
- `M190`
- optional comments and metadata markers

Forbidden in the slicer profile for V1:

- `G28`
- `G29`
- purge lines that assume a Cartesian printer front edge
- parking moves that assume fixed gantry geometry
- vendor-specific printer macros not supported by the host stack

Reason:

- Homing and printer preparation belong to the host-controlled print workflow, not to a generic slicer script.
- Cartesian assumptions in start G-code are unsafe on a robot.

### End G-code rules

Allowed pattern:

- heater shutdown commands,
- fan shutdown or standby commands,
- optional `M104 S0`, `M140 S0`, `M107`,
- optional metadata comments.

Forbidden:

- Cartesian park moves,
- gantry-specific cooldown motion,
- unsupported vendor macros.

## 6. Preferred output format and conventions

### Preferred conventions

- Millimeters only.
- Plain `.gcode` output.
- Single extruder.
- Relative extrusion preferred for V1 when practical.
- No arc commands in the first supported slicer preset.

### Why relative extrusion is preferred

- It simplifies chunked execution.
- It reduces the risk of large accumulated `E` values across long prints.
- It maps more naturally to a host-side segmented execution model.

The parser must still support absolute extrusion because Cura or imported files may use it.

## 7. First official slicer preset

Start with a conservative default process profile:

- nozzle: `0.4 mm`
- filament: `1.75 mm`
- layer height: `0.20 mm`
- line width: `0.42 mm`
- wall speed: low and conservative
- infill speed: low and conservative
- travel speed: moderate, not aggressive
- cooling: conservative for early calibration prints
- retraction: conservative Bowden baseline if Bowden is selected

### Initial speed guidance

The robot is not a rigid Cartesian printer, so first profiles should prioritize repeatability over throughput.

Recommended starting band for first hot-print experiments:

- print speed: `15-25 mm/s`
- travel speed: `30-60 mm/s`

These are intentionally conservative and should be tuned upward only after:

- first-layer consistency,
- extrusion stability,
- path accuracy,
- and wrist vibration behavior are verified.

## 8. Host validation rules tied to slicer choice

The host should identify the slicer family when possible from comments and file structure.

### For PrusaSlicer

The parser should be able to exploit comments such as:

- `;TYPE:`
- `;LAYER_CHANGE`
- `;CUSTOM_GCODE`

### For Cura

The parser should accept Cura-generated files, but should not rely on Cura-specific metadata being present for correctness.

### General rule

Motion correctness must never depend on slicer comments alone. Comments are an optimization for diagnostics and preview, not a safety-critical dependency.

---

## Hardware Plan

## 1. Printhead mechanical assembly

### Required components

- Ender 3 compatible hotend/nozzle assembly.
- Heater cartridge matching the selected supply voltage.
- Hotend thermistor.
- Hotend heatsink fan.
- Part cooling fan.
- Printhead mount rigidly attached to the robot wrist.
- Cable strain relief and cable routing.

### Recommended first build choice

Use a **Bowden-style layout** for V1:

- Mount the extruder stepper on the frame, not on the wrist.
- Route filament through PTFE tube to the hotend.

Reason:

- Lowers moving mass on the arm.
- Reduces wrist inertia.
- Improves print stability while IK and extrusion synchronization are still new.
- Simplifies cable management.

Direct drive can be revisited later if the arm proves stiff enough.

### Mechanical requirements

- Rigid mount with minimal flex under acceleration.
- Nozzle tip must be visible and measurable relative to the robot TCP frame.
- Fan ducts must not collide with the bed or printed part.
- Cable bundle must not load J5/J6 significantly.
- Hot surfaces must be shielded from wire insulation and printed plastic parts.

## 2. Bed assembly

### Required components

- Heated print bed.
- Bed thermistor.
- Rigid bed mount fixed in the robot workcell.
- Bed surface suited to the intended materials.
- Adjustable tramming scheme, preferably 3-point.

### Bed requirements

- Bed position relative to robot base must remain stable over time.
- The bed surface plane must be measurable in robot coordinates.
- The bed mounting must survive repeated heating cycles without drift.

### Important architectural rule

Do **not** rely on printer-side bed leveling features in Marlin for robot motion compensation.

Reason:

- The robot controller owns the actual motion.
- Any bed leveling or plane compensation must happen in the host/robot coordinate pipeline, not inside the SKR.

## 3. Additional hardware that may still be needed

- PTFE tube and pneumatic fittings if Bowden is used.
- Silicone sock for the hotend.
- Purge/wipe area near the print bed.
- Physical nozzle cleaning brush or wipe station.
- Bed insulation.
- Power distribution block or fused outputs.
- External bed MOSFET / relay if bed current exceeds what the onboard path safely supports.
- Thermal fuse or secondary overtemperature cut-off for the bed.
- Emergency stop that removes power from both motion and heating.
- USB cable management and strain relief.

## 4. Sensors and probing

### Minimum viable setup

- Hotend thermistor.
- Bed thermistor.
- Existing arm homing endstops.
- Manual bed-plane probing using the nozzle or a feeler-gauge workflow.

### Recommended upgrade path

Add a dedicated probing strategy later:

- conductive nozzle touch-off,
- mechanical probe mounted near nozzle,
- or temporary calibration probe tool.

This is more valuable than ordinary printer ABL because the robot needs the bed plane in robot coordinates.

---

## Electrical and Safety Plan

## 1. Power architecture

### Required actions

- Confirm supply voltage for:
  - hotend heater,
  - heated bed,
  - fans,
  - extruder stepper,
  - arm motors.
- Calculate worst-case continuous current draw.
- Size the PSU with safety margin.
- Separate high-current heater wiring from signal wiring.

### Recommended topology

- One controlled 24V power domain for print peripherals.
- Separate logic/USB isolation from noisy heater wiring where possible.
- Shared protective earth and safe chassis grounding where applicable.

## 2. Wiring ownership split

### Teensy side

- J1-J6 step/dir/enable.
- Arm homing endstops.
- Existing robot safety and E-stop integration.

### SKR side

- Extruder stepper.
- Hotend heater.
- Hotend thermistor.
- Heated bed output.
- Bed thermistor.
- Part cooling fan.
- Hotend heatsink fan.

### Host side

- USB serial connection to Teensy.
- USB serial connection to SKR.

## 3. Hard safety requirements

- Physical emergency stop must cut motion and heaters.
- Thermal protection must be enabled in printer firmware.
- Heater enable must fail closed.
- Bed current path must be verified before energizing.
- No print should start unless both controllers are connected and healthy.
- Hotend should never be allowed to heat without thermistor validation.
- Hotend fan must default to a safe-on behavior when the hotend is hot.

## 4. Recommended safety checklist

- Fuse the heater circuits.
- Inspect connector temperature under load.
- Keep bed and hotend wires away from robot joints.
- Add strain relief at the hotend and bed.
- Verify no cable rub at all reachable joint configurations used for printing.
- Add timeout-based shutdown in the host if either controller stops responding.

---

## Firmware Strategy

## 1. Arm controller firmware: keep current architecture

The Teensy firmware remains the robot motion controller.

### Immediate firmware goals

- Keep current `TQ` queue execution path.
- Preserve current homing and joint limit behavior.
- Preserve existing safety and queue diagnostics.

### Arm firmware changes recommended for print support

- Add an explicit print-safe pause command later if needed.
- Add a more explicit "queue idle" or "execution finished" query if current signals are insufficient.
- Expose fault states more clearly if the host needs tighter print-state coordination.

### Arm firmware changes not required for V1

- No heater support.
- No extruder support.
- No printer-style G-code parser on the Teensy.

## 2. Print controller firmware: Marlin recommended for V1

### Recommendation

Run **Marlin** on the SKR Mini E3 V2.0 for V1.

### Why Marlin for this project

- Direct serial G/M-code control fits the existing browser host model.
- Good support for heaters, thermistors, fans, and extruder control.
- Thermal protection, PID tuning, EEPROM, and common maintenance commands are mature.
- No extra Linux host layer is required.

### Why not make Klipper the first choice

- Klipper is strong, but it expects a different host/control model.
- This project already has its own host and motion planner.
- For this use case, Klipper adds integration complexity without solving the robot-motion problem.

## 3. Required Marlin capabilities on the SKR

Enable and validate:

- hotend thermal protection,
- bed thermal protection,
- PID tuning for hotend and bed,
- EEPROM settings,
- serial firmware info,
- temperature reporting,
- fan control,
- extruder motion,
- cold extrusion prevention for normal operation.

### SKR command set the host should rely on

- `M115` - identify firmware and capabilities.
- `M105` - request temperature reports.
- `M104` - set hotend temp, no wait.
- `M109` - wait for hotend temp.
- `M140` - set bed temp, no wait.
- `M190` - wait for bed temp.
- `M106` / `M107` - part cooling fan control.
- `M400` - wait for printer-side queued moves to complete.
- `M303` - PID autotune.
- `M500`, `M501`, `M502`, `M503` - settings persistence and reporting.

### Commands intentionally not used for robot motion

- `G28`, `G29`, printer-side bed leveling, or printer XYZ kinematics.

Those belong to printer architectures, not this robot control stack.

---

## Software Architecture Plan

## 1. Core design principle

The browser application becomes a **supervisory print host**. It owns:

- the print job,
- modal G-code state,
- coordinate transforms,
- motion segmentation,
- controller synchronization,
- preflight,
- calibration data,
- fault handling.

## 2. New software modules

### Recommended new folders

```text
robot-arm-control/src/gcode/
robot-arm-control/src/services/printing/
robot-arm-control/src/calibration/
```

### Recommended new files

```text
robot-arm-control/src/communication/PrintControllerSerialManager.ts
robot-arm-control/src/communication/printControllerTypes.ts

robot-arm-control/src/gcode/types.ts
robot-arm-control/src/gcode/tokenize.ts
robot-arm-control/src/gcode/parser.ts
robot-arm-control/src/gcode/modalState.ts
robot-arm-control/src/gcode/interpreter.ts
robot-arm-control/src/gcode/segmentPlanner.ts

robot-arm-control/src/services/printing/types.ts
robot-arm-control/src/services/printing/PrintJobService.ts
robot-arm-control/src/services/printing/PrintExecutionCoordinator.ts
robot-arm-control/src/services/printing/ExtrusionScheduler.ts
robot-arm-control/src/services/printing/PrintSafetyService.ts
robot-arm-control/src/services/printing/PrintCalibrationService.ts
robot-arm-control/src/services/printing/SlicerProfileService.ts

robot-arm-control/src/components/PrintControlPanel.tsx
robot-arm-control/src/components/PrintJobPanel.tsx
robot-arm-control/src/components/PrintCalibrationPanel.tsx
```

### Recommended existing files likely to change

```text
robot-arm-control/src/store/robotStore.ts
robot-arm-control/src/communication/types.ts
robot-arm-control/src/types/robot.ts
robot-arm-control/src/App.tsx
```

## 3. Job state machine

Create an explicit print job state model:

- `idle`
- `preflight`
- `heating`
- `homing`
- `bed_calibration`
- `priming`
- `printing`
- `pausing`
- `paused`
- `aborting`
- `completed`
- `fault`

### Required rule

Printing must not enter `printing` until:

- both controllers are connected,
- arm is homed,
- printhead temperatures are valid,
- bed calibration exists,
- the operator has acknowledged preflight.

## 4. Data model additions

Add persistent configuration for:

- print bed dimensions,
- bed frame to robot base transform,
- nozzle TCP offset,
- default print orientation pose,
- slicer profile assumptions,
- extruder calibration values,
- nozzle diameter,
- filament diameter,
- retraction settings,
- purge settings,
- temperature presets,
- print-safe park pose.

---

## G-code Interpretation Plan

## 1. Parser philosophy

The host must implement a real modal interpreter, not a naive line reader.

### Reasons

- G-code is stateful.
- Motion lines inherit modal state.
- Absolute vs relative position matters.
- Absolute vs relative extrusion matters.
- Feedrate persists between blocks.
- Unit system persists between blocks.

## 2. Minimum supported command set for V1

### Motion and positioning

- `G0`
- `G1`
- `G4`
- `G21`
- `G90`
- `G91`
- `G92`

### Extrusion mode

- `M82`
- `M83`

### Print process control

- `M104`
- `M105`
- `M106`
- `M107`
- `M109`
- `M140`
- `M190`

### Operational support

- comments
- empty lines
- line numbers if present

## 3. Initial restrictions on slicer output

For V1, enforce these slicer profile rules:

- millimeters only,
- planar layers only,
- no arcs (`G2` / `G3`) in the initial implementation,
- no printer-specific auto-bed-leveling start code,
- no printer homing commands in the sliced file,
- single extruder only.

### Recommended startup profile assumptions

- `G21`
- `G90`
- `M83`

The parser should still support `M82`, but `M83` is a cleaner first profile for chunked execution.

## 4. Fail-closed behavior

If a print file contains unsupported commands or unsafe assumptions:

- reject the file before motion starts,
- report exact offending line numbers,
- show which commands are unsupported.

Do not silently ignore unknown commands that could affect print outcome.

---

## Calibration Plan

## 1. Coordinate frames that must exist

Define and store these frames explicitly:

- `robot_base_frame`
- `tool_frame`
- `nozzle_tip_frame`
- `bed_frame`
- `slicer_frame`

## 2. Critical calibration items

### A. Nozzle TCP offset

Measure the transform from the robot wrist/tool mount to the nozzle tip:

- X offset,
- Y offset,
- Z offset,
- nozzle orientation relative to tool frame.

### B. Print orientation pose

Define the nominal printing orientation:

- nozzle vertical to bed,
- cable-safe wrist posture,
- preferred J6 alignment if relevant.

This pose should be locked during V1 printing.

### C. Bed frame calibration

Measure bed position in robot coordinates.

Minimum implementation:

- probe or teach at least 3 non-collinear bed points,
- fit a plane,
- derive origin + plane normal,
- build a bed-frame transform.

### D. Nozzle-to-bed Z zero

Establish a repeatable method to define first-layer Z reference:

- feeler gauge,
- paper method,
- electrical contact touch-off,
- or dedicated probe tool.

### E. Extruder calibration

Calibrate:

- steps per mm,
- direction,
- retraction baseline,
- flow multiplier baseline.

### F. Thermal calibration

Calibrate:

- hotend PID,
- bed PID,
- safe printing temperature ranges,
- fan interaction effects on temperature stability.

## 3. Recommended calibration order

1. Cold robot kinematics verification
2. Tool mount and nozzle TCP measurement
3. Bed physical tramming
4. Bed frame plane fit
5. Z zero and first layer reference
6. Extruder steps per mm
7. Hotend PID
8. Bed PID
9. First layer tuning
10. Flow and retraction tuning

---

## Execution and Synchronization Plan

## 1. Motion planning pipeline

Each printable move should flow through this pipeline:

1. Parse G-code block.
2. Update modal state.
3. Determine whether the line is:
   - travel move,
   - print move,
   - heating/control command,
   - dwell.
4. Convert slicer XYZ into bed-frame coordinates.
5. Convert bed-frame coordinates into robot TCP targets.
6. Apply fixed print orientation.
7. Run IK and path quality validation.
8. Generate robot trajectory points.
9. Compute extrusion delta and extrusion timing.
10. Dispatch to both controllers through a coordinated execution model.

## 2. Recommended synchronization model for V1

Use **chunked execution with barriers**.

### Chunk model

- Group motion into small chunks of printable path segments.
- For each chunk:
  - upload robot trajectory to the Teensy,
  - send matching extruder moves to the SKR,
  - start both sides,
  - wait for both to finish,
  - continue to next chunk.

### Why this is the right V1 tradeoff

- Good enough for an initial integrated printer.
- Avoids redesigning the robot motion firmware immediately.
- Keeps synchronization errors bounded.
- Makes pause/abort logic simpler.

### Known limitation

This is not perfect shared-clock synchronization. It is a pragmatic first system.

## 3. Phase 2 synchronization upgrade

If print quality demands tighter sync later, extend the motion system so that:

- extrusion becomes a first-class scheduled channel,
- or the arm controller gains an auxiliary synchronized output/extrusion interface,
- or a dedicated print-time coordinator with tighter timestamp control is added.

This should only happen after V1 prints successfully.

## 4. Travel moves vs print moves

### Travel move

- no extrusion,
- optionally higher speed,
- safe clearance above part if needed.

### Print move

- extrusion required,
- speed limited by:
  - arm path quality,
  - hotend throughput,
  - extrusion stability,
  - corner behavior,
  - robot rigidity.

## 5. Pause and abort behavior

### V1 pause behavior

- stop new chunk dispatch,
- wait for active chunk to finish or safe-stop,
- retract or pressure-relieve if needed,
- park robot at safe pose if possible,
- maintain or reduce temperatures based on operator choice.

### V1 abort behavior

- emergency stop if needed,
- heaters off or to safe standby,
- fans to safe state,
- robot to safe stop,
- mark print as non-resumable unless explicitly requalified.

---

## UI and Operator Workflow Plan

## 1. New operator surfaces

### Print Control Panel

- connect/disconnect SKR controller,
- temperature setpoints,
- fan controls,
- prime/retract buttons,
- print start/pause/abort,
- current job state,
- fault display.

### G-code Job Panel

- load G-code file,
- validate file,
- preview metadata,
- estimate job length,
- show unsupported commands before execution.

### Print Calibration Panel

- bed frame calibration,
- nozzle TCP calibration,
- Z zero calibration,
- extrusion calibration,
- temperature tuning helpers.

## 2. Preflight checklist in UI

Before every print, require operator confirmation of:

- correct nozzle installed,
- filament loaded,
- bed surface prepared,
- hotend clear,
- no cable interference,
- correct bed/frame calibration selected,
- print area clear,
- E-stop reachable.

## 3. Recommended print workflow

1. Connect to Teensy.
2. Connect to SKR.
3. Run controller health check.
4. Home the arm.
5. Load or verify calibration set.
6. Load G-code.
7. Run file validation.
8. Heat bed and hotend.
9. Prime extruder.
10. Start print.
11. Monitor temperatures, queue status, and print progress.
12. On finish, cool down and park.

---

## Implementation Phases

## Phase 0: Architecture freeze and hardware survey

### Deliverables

- Final decision on dual-controller architecture.
- Confirmed BOM and missing hardware list.
- Wiring ownership diagram.
- Confirmed printhead mechanical concept.

### Tasks

- Confirm heater voltage.
- Confirm presence/type of thermistors.
- Confirm extruder layout: Bowden vs direct drive.
- Confirm bed current path and whether external MOSFET is required.
- Confirm available PSU and wiring gauge.

## Phase 1: SKR bring-up as printhead controller

### Deliverables

- SKR flashed and responding over USB.
- Hotend heater and thermistor working.
- Bed heater and thermistor working.
- Fans controllable.
- Extruder stepper calibrated.

### Tasks

- Prepare Marlin config for this print-only use.
- Validate `M115`, `M105`, `M104`, `M109`, `M140`, `M190`, `M106`, `M107`, `M400`.
- Run PID autotune.
- Save validated settings.

## Phase 2: Host integration for second controller

### Deliverables

- New serial manager for the SKR.
- UI can connect to both controllers.
- Temperature and fan telemetry visible.

### Tasks

- Add `PrintControllerSerialManager`.
- Add temperature polling and parsing.
- Add SKR connection state to the store.
- Add heater/fan controls.

## Phase 3: G-code parser and validator

### Deliverables

- Modal parser with validation.
- Unsupported command reporting.
- File metadata extraction.

### Tasks

- Implement tokenizer.
- Implement parser.
- Implement modal state machine.
- Implement validation rules.
- Add unit tests with representative slicer files.

## Phase 4: Bed frame and print calibration workflows

### Deliverables

- Bed-plane calibration stored in app config.
- Nozzle TCP calibration stored in app config.
- Z zero workflow available.

### Tasks

- Add calibration data structures.
- Add UI flows.
- Implement plane fitting and transform generation.
- Verify repeatability.

## Phase 5: Print path mapping and dry-run execution

### Deliverables

- G-code motion mapped to robot TCP path.
- Dry-run without heating succeeds.
- Print preview in robot coordinates.

### Tasks

- Convert XYZ to bed frame.
- Convert bed frame to robot frame.
- Apply fixed print orientation.
- Validate IK on representative toolpaths.
- Test with marker/pen or air-print dry-runs.

## Phase 6: Extrusion synchronization and first hot prints

### Deliverables

- Chunked coordinated print execution.
- First successful extrusion tests.
- First single-layer and multi-layer prints.

### Tasks

- Implement chunk dispatch.
- Compute extrusion per segment.
- Tune chunk size and timing.
- Validate corner behavior and retraction.

## Phase 7: Reliability, recovery, and operator polish

### Deliverables

- Fault handling and operator messaging.
- Safer pause/abort behavior.
- Stable first-layer workflow.
- Documentation for setup and maintenance.

### Tasks

- Improve preflight.
- Improve fault reporting.
- Add print-specific logs and diagnostics.
- Add maintenance checklist and wiring diagrams.

---

## Validation and Test Plan

## 1. Electrical validation

- Verify thermistor readings at room temperature.
- Verify heater outputs remain off until commanded.
- Verify fan outputs behave as expected.
- Verify connector temperature under load.
- Verify bed current path does not overheat.

## 2. Motion validation without heat

- Run bed-space test paths with pen or dry nozzle.
- Validate reachability over intended print area.
- Validate cable clearance across full planned print envelope.
- Validate no wrist collisions with bed or printed part.

## 3. Thermal validation

- Heat hotend only and validate stability.
- Heat bed only and validate stability.
- Validate thermal protection behavior.
- Validate cooldown behavior.

## 4. Extrusion-only validation

- Calibrate 100 mm filament feed.
- Validate prime and retract.
- Validate direction and skipped-step resistance.

## 5. Integrated print validation

Recommended print order:

1. straight bead lines,
2. single-layer square,
3. first-layer calibration pattern,
4. single-wall cube,
5. small solid calibration object,
6. multi-layer test part.

## 6. Software testing

- Unit tests for parser and modal state.
- Unit tests for coordinate transforms.
- Unit tests for planner chunking.
- Integration tests for dual-controller state machine.
- Manual operator tests for pause, abort, disconnect, and overtemperature scenarios.

---

## Acceptance Criteria

The V1 implementation is acceptable when all of the following are true:

- A standard sliced planar G-code file can be loaded and validated in the app.
- The app can connect to both Teensy and SKR in the same session.
- The app can heat the bed and hotend and read stable temperatures.
- The app can calibrate and persist bed frame and nozzle offsets.
- The app can convert printable G-code motion to robot trajectories without quality violations in the intended print area.
- The app can execute chunked coordinated printing without uncontrolled drift or unsafe queue underruns.
- The system can print a repeatable single-wall test object with acceptable first-layer adhesion and layer stacking.
- Abort and cooldown behave predictably.
- Operator workflow is documented and repeatable.

---

## Risks and Mitigations

## 1. Extrusion and robot motion drift

### Risk

The robot and extruder are controlled by separate planners and may not stay perfectly synchronized.

### Mitigation

- Start with small chunk sizes.
- Use barrier-based coordination.
- Keep speeds conservative.
- Upgrade to tighter synchronization only after baseline printing works.

## 2. Excess wrist mass and poor path quality

### Risk

A heavy direct-drive head will reduce print quality and increase oscillation.

### Mitigation

- Use Bowden first.
- Keep the wrist payload low.
- Limit print acceleration and speed.

## 3. Bed thermal drift relative to robot frame

### Risk

Heating can shift the bed plane or frame.

### Mitigation

- Use rigid bed mounting.
- Recheck plane at operating temperature if needed.
- Store calibration metadata with temperature context.

## 4. Unsafe heater integration

### Risk

Bad thermistor wiring or overloaded bed current path can create a fire hazard.

### Mitigation

- Verify all heater wiring before power-up.
- Enable printer-side thermal protection.
- Use external power switching if needed.
- Require E-stop and fused power.

## 5. Coordinate-frame mistakes

### Risk

Small transform errors will ruin first-layer height and part geometry.

### Mitigation

- Make frames explicit in software.
- Separate bed-frame calibration from nozzle TCP calibration.
- Add visualization for all frames and offsets.

## 6. Resume complexity

### Risk

True print resume after faults is much harder on an open-loop robot than on a fixed-frame printer.

### Mitigation

- Treat resume as out of scope for V1.
- Prioritize safe abort over unreliable resume.

---

## Reference Material

### Official documentation and references used for planning

- BIGTREETECH SKR Mini E3 repository  
  https://github.com/bigtreetech/BIGTREETECH-SKR-mini-E3

- Marlin configuration and kinematics overview  
  https://marlinfw.org/docs/configuration/configuration.html

- Marlin G-code index  
  https://marlinfw.org/meta/gcode/

- Marlin `G0` / `G1`  
  https://marlinfw.org/docs/gcode/G000-G001.html

- Marlin `M104`  
  https://marlinfw.org/docs/gcode/M104.html

- Marlin `M105`  
  https://marlinfw.org/docs/gcode/M105.html

- Marlin `M109`  
  https://marlinfw.org/docs/gcode/M109.html

- Marlin `M140`  
  https://marlinfw.org/docs/gcode/M140.html

- Marlin `M190`  
  https://marlinfw.org/docs/gcode/M190.html

- Marlin `M400`  
  https://marlinfw.org/docs/gcode/M400.html

- Marlin `M500`, `M501`, `M502`, `M503`  
  https://marlinfw.org/docs/gcode/M500.html  
  https://marlinfw.org/docs/gcode/M501.html  
  https://marlinfw.org/docs/gcode/M502.html  
  https://marlinfw.org/docs/gcode/M503.html

- Marlin `M115` firmware capabilities  
  https://marlinfw.org/docs/gcode/M115.html

- Marlin PID autotune `M303`  
  https://marlinfw.org/docs/gcode/M303.html

- Klipper configuration reference  
  https://www.klipper3d.org/Config_Reference.html

- LinuxCNC G-code modal groups overview  
  https://www.linuxcnc.org/docs/2.9/html/gcode/overview.html

- RoboDK robot 3D printing workflow  
  https://robodk.com/doc/en/Robot-Machining-Robot-3D-Printing-Project.html

- PrusaSlicer custom bed models  
  https://help.prusa3d.com/article/custom-bed-models_158431

- PrusaSlicer macros  
  https://help.prusa3d.com/article/macros_1775

- PrusaSlicer G-code viewer and slicer-specific comments  
  https://help.prusa3d.com/article/prusaslicer-g-code-viewer_193152

- Cura start/end G-code customization  
  https://github.com/Ultimaker/Cura/wiki/Start-End-G%E2%80%90Code

- Cura machine profile definition notes  
  https://github.com/Ultimaker/Cura/wiki/Adding-new-machine-profiles-to-Cura

- OrcaSlicer printable space and custom bed model settings  
  https://www.orcaslicer.com/wiki/printer_settings/basic%20information/printer_basic_information_printable_space.html

- OrcaSlicer machine G-code  
  https://www.orcaslicer.com/wiki/printer_settings/machine%20gcode/printer_machine_gcode.html

---

## Next Action After This Plan

Implement **Phase 0** first and do not start software work before these hardware unknowns are resolved:

- hotend heater voltage,
- hotend thermistor type,
- bed voltage and current draw,
- PSU capability,
- Bowden vs direct-drive decision,
- bed current switching method,
- E-stop power-cut scheme.

Once those are confirmed, Phase 1 can begin immediately.

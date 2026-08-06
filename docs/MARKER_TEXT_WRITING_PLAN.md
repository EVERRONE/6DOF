# Marker Text Writing Plan

**Version:** 1.0  
**Date:** March 24, 2026  
**Status:** Recommended V1 approach

---

## Goal

Enable the existing 6DoF arm to write recognizable text with a marker on a flat surface with good repeatability and low integration risk.

---

## Executive Recommendation

The best V1 implementation is **planar pen-plotting**, not freehand 6DoF handwriting.

Use this architecture:

- a **lightweight spring-loaded marker holder** on the wrist,
- a **calibrated writing plane** taught in robot coordinates,
- a **fixed writing orientation** held with existing `pose_lock` Cartesian planning,
- **stroke-based vector text** (single-line font paths),
- **queue-based execution (`TQ`) in bounded chunks**, not point-by-point `J` moves.

This is the shortest path to reliable text output with the current stack.

---

## Why This Is The Right Approach

### 1. The current stack already supports the hard part

The repository already has:

- URDF-based FK/IK and Cartesian planning,
- `pose_lock` motion for holding tool orientation along a Cartesian path,
- trajectory-queue upload and execution (`TQ`),
- queue progress and completion events,
- a path-planning UI and 3D preview baseline.

That means the missing work is mostly:

- writing-plane calibration,
- text-to-stroke conversion,
- draw/travel path generation,
- multi-segment queue execution for plotting jobs.

### 2. Writing is a planar process

Writing text with a marker is effectively a **2.5D task**:

- move in X/Y on the page while drawing,
- lift to a safe Z for travel moves,
- maintain nearly constant orientation relative to the page.

Trying to use the full 6DoF freedom while writing adds solver complexity without improving text quality.

### 3. Contact force matters more than perfect Z

A rigid marker mount will amplify:

- bed/page unevenness,
- calibration drift,
- backlash,
- small FK/IK errors.

A **compliant pen holder** absorbs those errors and gives far better practical results than trying to solve contact force in software.

---

## Recommended Mechanical Setup

### Pen Tool

Use:

- a **fine felt-tip or fineliner marker**,
- mounted in a **spring-loaded vertical holder**,
- with low moving mass and clear nozzle/TCP visibility.

Recommended V1 design goals:

- 3-8 mm compliance travel,
- adjustable preload,
- easy marker replacement,
- rigid wrist mount,
- visible tip for manual teaching.

### Pen Up / Pen Down Strategy

Recommended V1:

- **pen up/down by Z motion of the robot**,
- while the holder provides compliance during pen-down contact.

Operationally:

- `pen up` = move to `surface_z + safe_lift`
- `pen down` = move to `surface_z - compression_offset`

Where:

- `safe_lift` is typically a few millimeters,
- `compression_offset` is a small negative offset that compresses the spring consistently.

### What Not To Use First

Do **not** use these as the primary V1 mechanism:

- **J6 rotation as pen up/down**
  - with the current TCP model, J6 mainly changes orientation, not contact height.
  - it is the wrong abstraction for reliable pen lift.
- **Dedicated servo as the first solution**
  - viable later, but unnecessary for initial validation.
  - adds firmware and wiring complexity before the core plotting pipeline is proven.

If later you want faster pen lifts without re-solving IK, a separate pen-lift actuator can be added as a Phase 2 refinement.

---

## Recommended Software Architecture

## V1 Data Flow

```text
Text input
  -> stroke font expansion
  -> local text polylines
  -> writing-plane transform
  -> draw/travel Cartesian segments
  -> bounded queue chunks
  -> Teensy TQ execution
```

### Text Representation

Use **stroke fonts**, not filled fonts and not raster text.

Best V1 choice:

- uppercase single-stroke glyphs,
- line segments only,
- optional later support for bezier/SVG flattening.

Reasons:

- predictable output,
- easy preview,
- small parser/planner scope,
- no fill strategy needed,
- natural mapping to pen plotting.

Good V1 output types:

- a simple built-in stroke alphabet,
- imported SVG polylines,
- later: simple pen-plotter G-code.

### Motion Model

Represent text as two primitive move classes:

- `draw_move`
  - tool touches the page with spring compression
  - slower speed
  - fixed orientation
- `travel_move`
  - tool lifted above the page
  - faster speed
  - same orientation

This is much simpler and safer than exposing arbitrary 6DoF text writing.

### Execution Model

Use **queue-based chunked execution**:

- concatenate contiguous draw/travel samples into a queue payload,
- keep each payload below firmware point budget,
- upload with `TQ CLEAR -> TQ PT* -> TQ? -> TQ RUN`,
- wait for `TQ DONE`,
- continue to next chunk.

This is required because:

- the current firmware queue limit is finite,
- plotting text creates many more samples than a single Cartesian move,
- the existing generic waypoint executor still sends point-by-point `J` commands.

For text writing, the generic `executeTrajectory()` path should not be the primary runner.

---

## Calibration Requirements

Text writing needs a **surface frame**, not just robot FK/IK.

### Required Frames

- `robot_base_frame`
- `tool_frame`
- `marker_tip_frame`
- `writing_plane_frame`
- `text_frame`

### Minimum V1 Calibration Workflow

1. Teach three non-collinear points on the paper/board.
2. Fit the writing plane.
3. Define page origin and X direction.
4. Teach pen-up safe height.
5. Teach pen-down compression offset.
6. Save the calibration set.

### Practical Note

This is the same family of problem as the existing 3D-printing plan's bed-frame calibration, just simpler:

- no extrusion,
- no thermal drift,
- no dual-controller sync.

So marker writing is the correct stepping stone before full print-toolpath execution.

---

## Implementation Recommendation

## Phase 1: Mechanical and Calibration Baseline

Deliverables:

- spring-loaded marker mount,
- rigid writing board,
- manual surface calibration flow,
- manual pen-up / pen-down verification.

Validation:

- can repeatedly touch the same point without damaging the marker,
- can lift cleanly without dragging.

## Phase 2: Plotting Geometry Pipeline

Add host-side modules such as:

```text
src/services/plotting/StrokeFontService.ts
src/services/plotting/TextLayoutService.ts
src/services/plotting/WritingPlaneService.ts
src/services/plotting/PlotPathPlanner.ts
src/services/plotting/PlotExecutionService.ts
src/services/plotting/types.ts
```

Responsibilities:

- `StrokeFontService`
  - convert text into glyph strokes
- `TextLayoutService`
  - spacing, scale, alignment, multiline layout
- `WritingPlaneService`
  - map local text XY into robot Cartesian targets
- `PlotPathPlanner`
  - generate draw/travel paths with Z lifts
- `PlotExecutionService`
  - batch and upload chunks through `TQ`

## Phase 3: UI

Add a dedicated writing panel:

- text input,
- font height,
- character spacing,
- line spacing,
- alignment,
- write speed,
- travel speed,
- safe lift,
- pen compression,
- preview,
- execute / pause / abort.

Do not hide this inside the existing waypoint UI. Writing is a separate workflow.

## Phase 4: Quality and Import Extensions

After baseline text works:

- SVG polyline import,
- basic pen-plotter G-code import (`G0`, `G1`, `M3/M5` style semantics),
- per-surface saved calibration profiles,
- optional separate pen-lift actuator.

---

## Current-Codebase Implications

### Reuse As-Is

- Cartesian FK/IK stack
- `pose_lock` path planning
- `PathInterpolator.interpolateCartesianSpace()`
- `TrajectoryExecutionService.uploadAndRunTrajectoryQueue()`
- `TQ` protocol and firmware queue diagnostics

### Do Not Reuse As Primary Plot Runner

- the current generic waypoint `executeTrajectory()` path

Reason:

- it still sends sequential `moveToAngles()` commands,
- that is not the smooth execution model needed for writing letters.

### Existing Legacy Plan To Correct

Older planning notes mention:

- J6-based pen actuation,
- servo-based pen control,
- broad G-code-first drawing support.

For the current repository state, the better order is:

1. calibrated plane
2. compliant marker tool
3. stroke text
4. chunked queue execution
5. SVG/G-code import later

---

## Initial Operating Envelope

Start conservatively:

- text height: 15-30 mm
- writing speed: 10-20 mm/s
- travel speed: 20-40 mm/s
- uppercase only
- short text strings
- flat rigid board clipped in a known location

Do not optimize speed first. Optimize:

- stroke closure,
- line straightness,
- repeatability,
- clean pen lifts,
- no paper gouging.

---

## Validation Plan

Use this order:

1. dot repeatability test
2. 50 mm square
3. diagonal cross
4. simple letters: `L`, `H`, `E`, `A`
5. short word: `HELLO`
6. multi-word line

Acceptance targets for V1:

- recognizable text,
- corners close consistently,
- no unintended pen marks during travel,
- no queue underrun faults in nominal jobs,
- repeated 50 mm square stays visually aligned within a few millimeters.

---

## Recommended Next Build Step

If you want the fastest path to a real result, implement in this order:

1. build the spring-loaded marker mount,
2. add writing-plane calibration in the app,
3. add a built-in stroke font and text preview,
4. add queue-chunk execution for multi-segment plot jobs,
5. validate with square and `HELLO` before any SVG or G-code import work.

That path gives you the highest chance of getting usable text quickly with the system you already have.

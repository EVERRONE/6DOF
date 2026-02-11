# 6DOF Robot Arm Control Software - Project Overview

## Project Goal

Develop comprehensive control software for a DIY 6DOF robot arm with the following capabilities:
1. **Forward Kinematics (FK)** - Calculate end-effector position from joint angles
2. **Inverse Kinematics (IK)** - Calculate joint angles from desired end-effector position
3. **Path Following** - Execute smooth trajectories through multiple waypoints
4. **Writing/Drawing** - Trace patterns, import G-code, draw SVG files

**Platform Requirements**: Web application (primary) with optional Windows desktop version (Electron wrapper)

---

## Hardware Specifications

### Controller & Drivers
- **MCU**: Teensy 4.1 (3.3V logic, USB Serial)
- **Stepper Drivers**: 6× TMC2209 (no UART) on 2× CNC Shield V3
- **Power**: 24V for motors (VMOT), 3.3V logic from Teensy
- **Microstepping**: 1/16 configured (TMC interpolates internally to 1/256)
- **Enable Pin**: Pin 8 (active-low, common to all drivers)
- **E-Stop**: Physical switch in series with 24V supply

### Motor Configuration

| Joint | Motor Model | Current | Gear Ratio | Resolution (µsteps/°) |
|-------|-------------|---------|------------|----------------------|
| J1 | Wantai 2.6A 1.8° | 2.6A | 16:100 | 55.556 |
| J2 | 17HS13-0404S-PG5 | 0.4A | 16:80 × 5:1 | 222.222 |
| J3 | 17HS16-2004S1 | 2.0A | 16:100 | 55.556 |
| J4 | 14HS08-0404S | 0.4A | 16:60 | 33.333 |
| J5 | 14HR05-0504S | 0.5A | 17:778 |
| J6 | 8HS11-0204S | 0.2A | 1:1 | 8.889 |

### Pin Mapping (Teensy 4.1)

**Step/Direction Pins:**
- J1: STEP=2, DIR=5
- J2: STEP=3, DIR=6
- J3: STEP=4, DIR=7
- J4: STEP=22, DIR=23
- J5: STEP=24, DIR=25
- J6: STEP=26, DIR=27

**Endstop Pins** (Omron D2F-L switches, INPUT_PULLUP, LOW=triggered):
- J2: Pin 30
- J3: Pin 31
- J4: Pin 32
- J5: Pin 33
- J1, J6: No physical endstops (teach-home method)

### Joint Limits & Homing

**Soft Limits** (degrees from home position):
- J1: -40° to +30°
- J2: 0° to +60°
- J3: 0° to +70°
- J4: 0° to +274°
- J5: 0° to +280°
- J6: -360° to +360°

**Direction Inversion**:
- INVERT_DIR: J1=false, J2=false, J3=false, J4=true, J5=true, J6=false

**Homing Configuration**:
- HOME_TOWARD_MIN: J2=true, J3=true, J4=true, J5=true
- Home position = 0° at endstop trigger point
- Post-home pose: J2=5°, J3=55°, J4=129°, J5=220°

### Kinematics
- **URDF File Available**: Complete kinematic chain with link lengths and joint axes
- **STL Files Available**: 3D models for visualization

---

## Software Architecture

### System Layers

```
┌─────────────────────────────────────────────┐
│         USER INTERFACE LAYER                │
│  • 3D visualization (robot + workspace)     │
│  • Joint sliders / XYZ position input       │
│  • Path designer / G-code import            │
│  • Real-time position feedback              │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────▼───────────────────────────┐
│      KINEMATICS & PLANNING ENGINE           │
│  • Forward Kinematics (DH parameters)       │
│  • Inverse Kinematics (analytical/numerical)│
│  • Trajectory planning & interpolation      │
│  • Collision detection & soft limits        │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────▼───────────────────────────┐
│       MOTION CONTROL LAYER                  │
│  • Step/dir pulse generation               │
│  • Velocity profiles (trapezoidal/S-curve)  │
│  • Coordinated multi-axis motion           │
│  • Emergency stop handling                  │
└─────────────────┬───────────────────────────┘
                  │
┌─────────────────▼───────────────────────────┐
│     COMMUNICATION / FIRMWARE                │
│  • Serial protocol (USB to Teensy 4.1)     │
│  • Command buffering                        │
│  • Position feedback                        │
│  • Endstop monitoring                       │
└─────────────────────────────────────────────┘
```

### Technology Stack

**Frontend (Web Application)**
- **Framework**: React + TypeScript
- **3D Rendering**: Three.js + React Three Fiber
- **UI Components**: shadcn/ui or Material-UI
- **State Management**: Zustand or Redux
- **Serial Communication**: Web Serial API (Chrome/Edge)

**Computation & Kinematics**
- **URDF Parsing**: urdf-loader (Three.js compatible)
- **Mathematics**: gl-matrix or mathjs for matrix operations
- **Kinematics**: Custom FK/IK implementation or robotics-js
- **Path Planning**: Custom trajectory generation algorithms

**Desktop Version (Optional)**
- **Electron**: Wrap web app for Windows distribution
- **Serial Communication**: Node.js serialport for broader device support

**Teensy Firmware**
- **Language**: C++ (Arduino/Teensyduino)
- **Libraries**: AccelStepper or custom step generation
- **Protocol**: Custom binary/ASCII protocol over USB Serial

---

## Development Phases (Priority Order)

### Phase 1: Core Infrastructure & Manual Control ⭐⭐⭐⭐⭐
**Priority**: Highest

**Teensy Firmware:**
- Serial command protocol (ASCII format)
- Step/direction pulse generation for 6 motors
- Endstop monitoring and homing routines
- Position tracking (microsteps → degrees conversion)
- Emergency stop handling (software + hardware)

**Web Application:**
- React project setup with TypeScript
- Serial connection manager (Web Serial API)
- Command queue system
- Status monitoring (connection state, position, endstops)

**3D Visualization:**
- URDF file parser and loader
- Three.js scene with robot model
- Camera controls (orbit, pan, zoom)
- Joint angle visualization (animated updates)
- Real-time position sync with Teensy

**Manual Control UI:**
- 6 sliders for joint control (with degree labels)
- Respect soft limits per joint
- Homing buttons (individual + all-axis sequence)
- Emergency stop button
- Position display (joint angles + estimated XYZ)
- Speed control slider for manual movements

**Deliverables:**
- ✅ Functional Teensy firmware with serial communication
- ✅ Web app with 3D robot visualization
- ✅ Manual control of all 6 joints
- ✅ Homing capability for J2-J5
- ✅ Real-time position feedback

---

### Phase 2: Forward & Inverse Kinematics ⭐⭐⭐⭐
**Priority**: High

**Forward Kinematics:**
- Extract DH parameters from URDF file
- Implement FK solver: joint angles → end-effector pose (4×4 matrix)
- Validation against known positions
- Workspace visualization (reachable volume sphere/ellipsoid)

**Inverse Kinematics:**
- Choose approach:
  - **Analytical IK** (preferred if arm has spherical wrist)
  - **Numerical IK** (Jacobian-based, fallback option)
- Handle multiple solutions (elbow up/down, wrist flip configurations)
- Singularity detection and avoidance
- Solution filtering (prefer closest to current pose)

**Cartesian Control UI:**
- XYZ position input fields (Cartesian coordinates)
- Orientation control (Euler angles: roll, pitch, yaw)
- "Move To" button with real-time IK solving
- Solution selector dropdown (when multiple IK solutions exist)
- Collision checking against soft limits
- Visual indicator: reachable (green) vs. unreachable (red) positions

**Deliverables:**
- ✅ FK solver validated against URDF
- ✅ IK solver with multiple solution support
- ✅ UI for Cartesian coordinate control
- ✅ Ability to command arm to specific XYZ positions
- ✅ Solution preview before execution

---

### Phase 3: Trajectory Planning & Path Following ⭐⭐⭐
**Priority**: Medium-High

**Trajectory Generation:**
- Linear interpolation in Cartesian space
- Circular/arc interpolation (optional)
- Velocity profiling:
  - Trapezoidal acceleration (simpler)
  - S-curve acceleration (smoother, preferred)
- Coordinated multi-joint motion (all joints finish simultaneously)
- Path preview in 3D visualization

**Path Designer UI:**
- Waypoint list editor (add, remove, reorder, edit)
- Path visualization (lines/curves between waypoints)
- Teach mode: "Add Current Position" button
- Save/load path files (JSON format)
- Parameters: speed, acceleration, blend radius
- Time estimation for path execution

**Execution Engine:**
- Path compilation: waypoints → joint trajectories
- Time synchronization across all 6 joints
- Smooth transitions between segments (blend zones)
- Real-time progress indicator
- Playback controls: Start, Pause, Resume, Stop, Step
- Loop mode for repetitive tasks

**Deliverables:**
- ✅ Multi-waypoint path planning
- ✅ Smooth coordinated motion
- ✅ Path save/load functionality
- ✅ Visual path preview
- ✅ Execution controls with pause/resume

---

### Phase 4: Writing & Drawing Capability ⭐⭐
**Priority**: Medium

**G-code Support:**
- G-code parser (G0, G1, G2, G3 basic commands)
- Coordinate systems (G54, G55, etc.)
- Feed rate control (F parameter)
- Tool offset (G43 for pen height)
- Supported commands:
  - G0: Rapid positioning
  - G1: Linear interpolation
  - G2/G3: Circular interpolation
  - M3/M5: Pen down/up (or J6 control)

**Drawing Tools:**
- SVG file import → G-code conversion
- Simple shape generator:
  - Circles, rectangles, polygons
  - Text rendering (vector fonts)
- Pen up/down control (J6 rotation or separate servo)
- Z-height calibration wizard for writing surface

**Writing UI:**
- Canvas for freehand drawing → path conversion
- G-code file upload with preview
- Drawing surface calibration:
  - 3-point or 4-point bed leveling
  - Automatic Z-offset calculation
- Simulation mode (dry run without pen contact)
- Scale and positioning controls

**Deliverables:**
- ✅ G-code interpreter
- ✅ SVG to robot path conversion
- ✅ Writing surface calibration
- ✅ Demo: signature, shapes, imported graphics

---

## File Structure

```
robot-arm-control/
├── firmware/                    # Teensy C++ code
│   ├── main.ino                # Main firmware loop
│   ├── config.h                # Pin mappings, limits, gear ratios
│   ├── stepper_control.h/.cpp  # Step pulse generation
│   ├── serial_protocol.h/.cpp  # Communication handler
│   ├── homing.h/.cpp           # Homing routines
│   └── motion.h/.cpp           # Velocity profiles
│
├── web-app/
│   ├── public/
│   │   └── models/
│   │       ├── robot.urdf      # URDF kinematic description
│   │       └── *.stl           # 3D mesh files
│   │
│   ├── src/
│   │   ├── components/
│   │   │   ├── Viewer3D/       # Three.js 3D visualization
│   │   │   │   ├── RobotModel.tsx
│   │   │   │   ├── Scene.tsx
│   │   │   │   └── CameraControls.tsx
│   │   │   │
│   │   │   ├── JointControl/   # Manual joint control
│   │   │   │   ├── JointSliders.tsx
│   │   │   │   ├── HomingPanel.tsx
│   │   │   │   └── PositionDisplay.tsx
│   │   │   │
│   │   │   ├── CartesianControl/ # XYZ position input
│   │   │   │   ├── PositionInput.tsx
│   │   │   │   ├── OrientationInput.tsx
│   │   │   │   └── SolutionSelector.tsx
│   │   │   │
│   │   │   ├── PathPlanner/    # Waypoint trajectory editor
│   │   │   │   ├── WaypointList.tsx
│   │   │   │   ├── PathVisualizer.tsx
│   │   │   │   └── ExecutionControls.tsx
│   │   │   │
│   │   │   ├── GCodeUploader/  # G-code & drawing interface
│   │   │   │   ├── FileUpload.tsx
│   │   │   │   ├── SVGConverter.tsx
│   │   │   │   └── CalibrationWizard.tsx
│   │   │   │
│   │   │   └── StatusBar/      # Connection & system status
│   │   │       ├── SerialStatus.tsx
│   │   │       └── EndstopIndicators.tsx
│   │   │
│   │   ├── kinematics/
│   │   │   ├── ForwardKinematics.ts    # FK solver
│   │   │   ├── InverseKinematics.ts    # IK solver
│   │   │   ├── DHParameters.ts         # DH table extraction
│   │   │   ├── URDFParser.ts           # URDF file parser
│   │   │   └── Workspace.ts            # Reachability analysis
│   │   │
│   │   ├── motion/
│   │   │   ├── TrajectoryPlanner.ts    # Waypoint → trajectory
│   │   │   ├── PathInterpolator.ts     # Cartesian interpolation
│   │   │   ├── VelocityProfile.ts      # Trapezoidal/S-curve
│   │   │   └── Coordinator.ts          # Multi-axis sync
│   │   │
│   │   ├── communication/
│   │   │   ├── SerialManager.ts        # Web Serial API wrapper
│   │   │   ├── CommandQueue.ts         # Buffering & sequencing
│   │   │   ├── Protocol.ts             # Message encoding/decoding
│   │   │   └── types.ts                # Command/response types
│   │   │
│   │   ├── utils/
│   │   │   ├── math.ts                 # Matrix operations
│   │   │   ├── transforms.ts           # Rotation conversions
│   │   │   ├── gcode.ts                # G-code parser
│   │   │   └── svg.ts                  # SVG path extraction
│   │   │
│   │   ├── store/
│   │   │   └── robotState.ts           # Global state (Zustand/Redux)
│   │   │
│   │   ├── App.tsx                     # Main application
│   │   └── index.tsx                   # Entry point
│   │
│   ├── package.json
│   ├── tsconfig.json
│   └── vite.config.ts          # Build configuration
│
├── docs/
│   ├── SERIAL_PROTOCOL.md      # Communication specification
│   ├── CALIBRATION.md          # Setup & tuning procedures
│   ├── API.md                  # Software API documentation
│   └── URDF_GUIDE.md           # Kinematic model explanation
│
└── README.md                   # Project overview
```

---

## Serial Communication Protocol

### Command Format (ASCII, newline-terminated)

```
J <j1> <j2> <j3> <j4> <j5> <j6> <speed>   # Move to joint angles (degrees)
H <joints>                                 # Home specified joints (e.g., "H 2345")
S                                          # Emergency stop (immediate)
Q                                          # Query current position
E <0|1>                                    # Enable (1) or disable (0) motors
M <speed>                                  # Set max speed (degrees/sec)
A <accel>                                  # Set acceleration (degrees/sec²)
```

### Response Format

```
OK <j1> <j2> <j3> <j4> <j5> <j6>          # Position update (degrees)
ENDSTOP <joint> <0|1>                      # Endstop state (0=open, 1=triggered)
ERROR <code> <message>                     # Error condition
HOMED <joint>                              # Homing complete for joint
READY                                      # Firmware ready after init
MOVING                                     # Motion in progress
IDLE                                       # Motion complete, ready for next command
```

### Example Communication Sequence

```
→ E 1                    # Enable motors
← OK 0 5 55 129 220 0    # Position after enable

→ H 2345                 # Home joints 2, 3, 4, 5
← HOMED 2
← HOMED 3
← HOMED 4
← HOMED 5
← OK 0 5 55 129 220 0    # Post-home pose

→ J 10 20 30 150 200 45 100   # Move to position at 100°/sec
← MOVING
← OK 10 20 30 150 200 45       # Final position reached
← IDLE
```

---

## Key Algorithms

### Forward Kinematics (DH Convention)

```typescript
interface DHParam {
  theta: number;   // Joint angle (variable for revolute)
  d: number;       // Link offset (along previous Z)
  a: number;       // Link length (along X)
  alpha: number;   // Link twist (rotation about X)
}

function computeDHTransform(theta: number, d: number, a: number, alpha: number): Matrix4 {
  // Returns 4×4 homogeneous transformation matrix
  const ct = Math.cos(theta);
  const st = Math.sin(theta);
  const ca = Math.cos(alpha);
  const sa = Math.sin(alpha);
  
  return [
    [ct,    -st*ca,  st*sa,   a*ct],
    [st,     ct*ca, -ct*sa,   a*st],
    [0,      sa,     ca,      d   ],
    [0,      0,      0,       1   ]
  ];
}

function forwardKinematics(jointAngles: number[]): Pose {
  let T = identityMatrix4();
  
  for (let i = 0; i < 6; i++) {
    const dh = dhParams[i];
    const Ti = computeDHTransform(
      jointAngles[i] + dh.theta_offset,
      dh.d,
      dh.a,
      dh.alpha
    );
    T = multiplyMatrix4(T, Ti);
  }
  
  // Extract position and orientation from T
  return {
    position: [T[0][3], T[1][3], T[2][3]],
    rotation: extractRotationMatrix(T)
  };
}
```

### Inverse Kinematics (Analytical Approach)

```typescript
interface IKSolution {
  joints: number[];  // [j1, j2, j3, j4, j5, j6]
  valid: boolean;
  score: number;     // Distance from current pose (prefer close solutions)
}

function inverseKinematics(
  targetPos: Vector3,
  targetRot: Matrix3,
  currentJoints?: number[]
): IKSolution[] {
  const solutions: IKSolution[] = [];
  
  // Step 1: Decouple position and orientation
  // Calculate wrist center position (target - tool offset along approach axis)
  const wristCenter = calculateWristCenter(targetPos, targetRot);
  
  // Step 2: Solve first 3 joints (position IK)
  // Geometric approach for anthropomorphic arm
  const j1Solutions = solveJ1(wristCenter);  // Base rotation (2 solutions)
  
  for (const j1 of j1Solutions) {
    const j23Solutions = solveJ2J3(wristCenter, j1);  // Elbow up/down (2 solutions)
    
    for (const [j2, j3] of j23Solutions) {
      // Step 3: Solve last 3 joints (orientation IK)
      // Spherical wrist decoupling
      const R03 = forwardKinematicsRotation([j1, j2, j3]);
      const R36 = multiplyMatrix3(transpose(R03), targetRot);
      
      const j456Solutions = solveSphericalWrist(R36);  // 2 solutions (wrist flip)
      
      for (const [j4, j5, j6] of j456Solutions) {
        const joints = [j1, j2, j3, j4, j5, j6];
        
        // Validate against soft limits
        if (isWithinLimits(joints)) {
          solutions.push({
            joints,
            valid: true,
            score: currentJoints ? calculateDistance(joints, currentJoints) : 0
          });
        }
      }
    }
  }
  
  // Sort by score (prefer closest to current pose)
  return solutions.sort((a, b) => a.score - b.score);
}
```

### Trajectory Planning (Trapezoidal Velocity Profile)

```typescript
interface Segment {
  startJoints: number[];
  endJoints: number[];
  duration: number;
  samples: JointState[];  // Time-stamped joint positions
}

function planTrajectory(
  waypoints: Pose[],
  maxVel: number,      // degrees/sec
  maxAccel: number,    // degrees/sec²
  dt: number = 0.01    // sampling interval (10ms)
): Segment[] {
  const segments: Segment[] = [];
  
  for (let i = 0; i < waypoints.length - 1; i++) {
    const start = waypoints[i];
    const end = waypoints[i + 1];
    
    // 1. Solve IK for start and end poses
    const startJoints = inverseKinematics(start.position, start.rotation)[0].joints;
    const endJoints = inverseKinematics(end.position, end.rotation)[0].joints;
    
    // 2. Calculate motion for each joint
    const jointMotions = [];
    let maxDuration = 0;
    
    for (let j = 0; j < 6; j++) {
      const delta = endJoints[j] - startJoints[j];
      const motion = calculateTrapezoidalProfile(delta, maxVel, maxAccel);
      jointMotions.push(motion);
      maxDuration = Math.max(maxDuration, motion.duration);
    }
    
    // 3. Time-stretch all joints to finish simultaneously
    const samples = [];
    for (let t = 0; t <= maxDuration; t += dt) {
      const jointState = jointMotions.map(motion => 
        motion.positionAtTime(t * motion.duration / maxDuration)
      );
      samples.push({ time: t, joints: jointState });
    }
    
    segments.push({
      startJoints,
      endJoints,
      duration: maxDuration,
      samples
    });
  }
  
  return segments;
}

function calculateTrapezoidalProfile(
  distance: number,
  maxVel: number,
  maxAccel: number
): VelocityProfile {
  // Calculate acceleration, cruise, and deceleration phases
  const accelTime = maxVel / maxAccel;
  const accelDist = 0.5 * maxAccel * accelTime * accelTime;
  
  if (2 * accelDist >= Math.abs(distance)) {
    // Triangle profile (no cruise phase)
    const peakVel = Math.sqrt(Math.abs(distance) * maxAccel);
    return new TriangleProfile(distance, peakVel, maxAccel);
  } else {
    // Trapezoidal profile (with cruise phase)
    const cruiseDist = Math.abs(distance) - 2 * accelDist;
    const cruiseTime = cruiseDist / maxVel;
    return new TrapezoidalProfile(distance, maxVel, maxAccel, cruiseTime);
  }
}
```

### G-code Parser

```typescript
interface GCodeCommand {
  type: 'G0' | 'G1' | 'G2' | 'G3' | 'M3' | 'M5';
  x?: number;
  y?: number;
  z?: number;
  i?: number;  // Arc center offset X
  j?: number;  // Arc center offset Y
  f?: number;  // Feed rate
}

function parseGCode(gcode: string): GCodeCommand[] {
  const commands: GCodeCommand[] = [];
  const lines = gcode.split('\n');
  
  for (const line of lines) {
    const cleaned = line.split(';')[0].trim();  // Remove comments
    if (!cleaned) continue;
    
    const match = cleaned.match(/^([GM])(\d+)/);
    if (!match) continue;
    
    const command: GCodeCommand = {
      type: `${match[1]}${match[2]}` as any
    };
    
    // Extract parameters
    const xMatch = cleaned.match(/X([-\d.]+)/);
    const yMatch = cleaned.match(/Y([-\d.]+)/);
    const zMatch = cleaned.match(/Z([-\d.]+)/);
    const iMatch = cleaned.match(/I([-\d.]+)/);
    const jMatch = cleaned.match(/J([-\d.]+)/);
    const fMatch = cleaned.match(/F([-\d.]+)/);
    
    if (xMatch) command.x = parseFloat(xMatch[1]);
    if (yMatch) command.y = parseFloat(yMatch[1]);
    if (zMatch) command.z = parseFloat(zMatch[1]);
    if (iMatch) command.i = parseFloat(iMatch[1]);
    if (jMatch) command.j = parseFloat(jMatch[1]);
    if (fMatch) command.f = parseFloat(fMatch[1]);
    
    commands.push(command);
  }
  
  return commands;
}

function gcodeToRobotPath(commands: GCodeCommand[]): Pose[] {
  const waypoints: Pose[] = [];
  let currentPos = { x: 0, y: 0, z: 0 };
  let penDown = false;
  
  for (const cmd of commands) {
    switch (cmd.type) {
      case 'G0':  // Rapid move (pen up)
      case 'G1':  // Linear move
        if (cmd.x !== undefined) currentPos.x = cmd.x;
        if (cmd.y !== undefined) currentPos.y = cmd.y;
        if (cmd.z !== undefined) currentPos.z = cmd.z;
        
        waypoints.push({
          position: [currentPos.x, currentPos.y, currentPos.z],
          rotation: identityMatrix3(),  // Fixed orientation for drawing
          penDown: penDown
        });
        break;
        
      case 'G2':  // Clockwise arc
      case 'G3':  // Counter-clockwise arc
        const arcWaypoints = interpolateArc(
          currentPos,
          { x: cmd.x, y: cmd.y, z: cmd.z || currentPos.z },
          { i: cmd.i || 0, j: cmd.j || 0 },
          cmd.type === 'G3'  // CCW flag
        );
        waypoints.push(...arcWaypoints);
        break;
        
      case 'M3':  // Pen down
        penDown = true;
        break;
        
      case 'M5':  // Pen up
        penDown = false;
        break;
    }
  }
  
  return waypoints;
}
```

---

## Testing & Calibration

### Phase 1 Testing
1. **Firmware Communication**: Send commands, verify responses
2. **Step Generation**: Measure actual steps vs. commanded
3. **Homing Accuracy**: Repeat homing 10 times, check consistency (< 0.1°)
4. **Manual Control**: Move each joint through full range
5. **Emergency Stop**: Verify immediate halt on button press

### Phase 2 Testing
1. **FK Validation**: Compare calculated positions with URDF model
2. **IK Round-Trip**: Verify FK(IK(pose)) ≈ pose (tolerance < 1mm)
3. **Multiple Solutions**: Test all IK solutions reach same end position
4. **Reachability**: Verify workspace boundary detection

### Phase 3 Testing
1. **Path Smoothness**: Record actual trajectory, analyze jerk
2. **Timing Accuracy**: Verify execution time matches prediction
3. **Blend Zones**: Check smooth transitions between waypoints
4. **Pause/Resume**: Test mid-path interruption and continuation

### Phase 4 Testing
1. **G-code Parsing**: Validate against standard test files
2. **Drawing Accuracy**: Draw grid pattern, measure deviation (< 2mm)
3. **Calibration**: 4-point bed leveling, verify flatness compensation
4. **Complex Shapes**: Test circles, text, SVG imports

### Calibration Procedures
1. **Gear Ratio Verification**: Command 360°, measure actual rotation
2. **DH Parameter Tuning**: Adjust link lengths for position accuracy
3. **Step Resolution**: Fine-tune µsteps/° if systematic error observed
4. **Zero Position**: Set reference pose, verify repeatability

---

## Success Criteria

### Phase 1 Complete
- ✅ Robot renders accurately in 3D viewer
- ✅ All 6 joints controllable via sliders
- ✅ Homing sequence completes without errors
- ✅ Position feedback updates in real-time (< 100ms latency)
- ✅ Emergency stop halts motion within 1 step

### Phase 2 Complete
- ✅ IK solutions found for 95% of reachable workspace
- ✅ End-effector reaches commanded XYZ within 5mm
- ✅ Orientation error < 2° for all axes
- ✅ Multiple solutions displayed and selectable

### Phase 3 Complete
- ✅ 10-waypoint path executes smoothly without stuttering
- ✅ Path execution time within 5% of prediction
- ✅ Teach mode captures 50+ waypoints for complex path
- ✅ Saved paths reload and execute identically

### Phase 4 Complete
- ✅ Standard G-code test file (e.g., "Hello World") executes
- ✅ Drawing accuracy: ±1mm for 100mm shapes
- ✅ SVG import produces recognizable output
- ✅ Pen up/down transitions occur precisely at waypoints

---

## Known Challenges & Solutions

### Challenge 1: IK Multiple Solutions
**Problem**: 6DOF arm has up to 8 IK solutions for same end pose
**Solution**: 
- Implement solution scoring (distance from current pose)
- UI displays top 3 solutions with preview
- Auto-select closest unless user overrides

### Challenge 2: Singularities
**Problem**: Near singularities, small Cartesian moves cause large joint velocities
**Solution**:
- Detect singularities in IK solver (Jacobian determinant < threshold)
- Warn user when approaching singular configuration
- Limit Cartesian velocity near singularities

### Challenge 3: Web Serial API Limitations
**Problem**: Not all browsers support Web Serial (Safari, Firefox)
**Solution**:
- Detect browser capability on load
- Display compatibility message for unsupported browsers
- Recommend Chrome/Edge or Electron desktop version

### Challenge 4: Real-Time Motion
**Problem**: JavaScript not real-time, step timing critical
**Solution**:
- Teensy handles all real-time step generation
- Web app sends high-level commands (target position + velocity)
- Firmware implements motion profiles in hardware timers

### Challenge 5: URDF Parsing Complexity
**Problem**: URDF XML can be complex with nested transforms
**Solution**:
- Use urdf-loader library (tested with ROS URDF files)
- Validate DH parameters extracted match manual calculations
- Provide override mechanism if auto-parsing fails

---

## Future Enhancements (Post-MVP)

### Advanced Features
- **Vision Integration**: Camera feedback for position correction
- **Force Control**: Torque sensors for compliant motion
- **Teach Pendant**: Physical controller for manual teaching
- **Multi-Robot**: Control multiple arms simultaneously
- **Simulation Mode**: Test programs without hardware connection

### Software Improvements
- **Web API**: RESTful API for external control
- **WebSocket Streaming**: Real-time position broadcast
- **Python Bindings**: Scripting interface for automation
- **ROS Integration**: Publish/subscribe to ROS topics
- **Mobile App**: iOS/Android version with touch controls

### UI/UX Enhancements
- **Dark Mode**: Theme switcher
- **Preset Positions**: Save/load favorite poses
- **Animation Timeline**: Keyframe-based motion editor
- **AR Overlay**: Augmented reality workspace preview
- **Multi-Language**: Internationalization (i18n)

---

## Resources & References

### Robotics Theory
- **Book**: "Introduction to Robotics: Mechanics and Control" by John J. Craig
- **DH Parameters**: Standard convention for kinematic chains
- **IK Methods**: Analytical vs. numerical approaches
- **Trajectory Planning**: Trapezoidal and S-curve profiles

### Libraries & Tools
- **Three.js**: https://threejs.org/
- **React Three Fiber**: https://docs.pmnd.rs/react-three-fiber/
- **urdf-loader**: https://github.com/gkjohnson/urdf-loaders
- **Web Serial API**: https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API
- **AccelStepper**: https://www.airspayce.com/mikem/arduino/AccelStepper/

### Similar Projects
- **MeArm**: Simple 4DOF arm with open-source software
- **Thor Robot**: 6DOF arm with ROS integration
- **Dexter**: High-precision 5DOF arm with advanced kinematics

---

## Contact & Support

**Project Lead**: Emilien
**Hardware**: DIY 6DOF arm with Teensy 4.1 + TMC2209 drivers
**Documentation**: URDF + STL files available
**Development Environment**: Windows, web-based primary target

---

## Change Log

- **2025-02-09**: Initial project specification created
  - Hardware specs documented from PDF
  - Architecture defined (4-layer system)
  - Development phases prioritized (1-4)
  - Technology stack selected
  - Serial protocol designed
  - Key algorithms outlined (FK, IK, trajectory planning, G-code)

---

## Next Steps for AI Implementation

1. **Read URDF File**: Parse kinematic chain, extract DH parameters
2. **Setup React Project**: Initialize with TypeScript, Three.js, Web Serial
3. **Implement Teensy Firmware**: Serial protocol, step generation, homing
4. **Build 3D Viewer**: Load URDF, render robot, animate joints
5. **Create Manual Control**: Sliders → serial commands → firmware → motion
6. **Develop FK/IK**: Implement solvers, validate with test cases
7. **Add Cartesian Control**: XYZ input → IK → motion execution
8. **Implement Path Planning**: Waypoints → trajectories → coordinated motion
9. **Add G-code Support**: Parser → robot paths → execution
10. **Testing & Calibration**: Validate each phase, tune parameters

**Note**: Each phase builds on the previous. Ensure Phase N is fully functional before starting Phase N+1.

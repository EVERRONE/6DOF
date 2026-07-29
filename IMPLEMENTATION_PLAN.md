# 6DOF Robot Arm Control System - Implementation Plan

> [!NOTE]
> **Historical document.** This was written against an earlier version of the
> project and no longer describes the code. The DH-based kinematics and the
> single-target step generator it refers to have both been replaced, and several
> of its claims about behaviour and test coverage no longer hold.
>
> It is kept for the design history. For how the system works now see
> [docs/KINEMATICS.md](docs/KINEMATICS.md),
> [docs/SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md) and
> [firmware/README.md](firmware/README.md).


**Version**: 1.0
**Date**: February 9, 2026
**Project Duration**: 8-10 weeks
**Target**: Production-ready web-based robot control system

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Project Structure & Setup](#project-structure--setup)
3. [Phase 1: Foundation & Basic Control](#phase-1-foundation--basic-control)
4. [Phase 2: Kinematics Implementation](#phase-2-kinematics-implementation)
5. [Phase 3: 3D Visualization](#phase-3-3d-visualization)
6. [Phase 4: Advanced Motion Control](#phase-4-advanced-motion-control)
7. [Phase 5: Drawing & G-code](#phase-5-drawing--g-code)
8. [Phase 6: Polish & Production](#phase-6-polish--production)
9. [Risk Management](#risk-management)
10. [Quality Assurance](#quality-assurance)
11. [Success Metrics](#success-metrics)

---

## Executive Summary

### Project Goal
Develop a professional-grade web application for controlling a 6DOF robot arm with full kinematics, path planning, and drawing capabilities.

### Core Deliverables
1. ✅ Teensy 4.1 firmware with step generation and homing
2. ✅ React/TypeScript web application with Web Serial API
3. ✅ Forward and Inverse Kinematics solvers
4. ✅ Real-time 3D visualization (Three.js + URDF)
5. ✅ Trajectory planning with velocity profiles
6. ✅ G-code parser and drawing capabilities

### Technology Stack Rationale

**Firmware (C++):**
- **Teensy 4.1**: Real-time step generation (600 MHz ARM Cortex-M7)
- **Custom implementation**: Full control over timing (no AccelStepper dependency)
- **Justification**: Critical timing requirements demand bare-metal control

**Frontend (React + TypeScript):**
- **React**: Component-based architecture for complex UI
- **TypeScript**: Type safety for kinematics calculations
- **Web Serial API**: Native browser communication (Chrome/Edge)
- **Three.js + React Three Fiber**: Industry-standard 3D rendering
- **Zustand**: Lightweight state management (simpler than Redux)

**Why Web App over Desktop:**
1. No installation required - instant access
2. Cross-platform by default
3. Easy updates and version control
4. Can wrap in Electron later if needed
5. Web Serial API is mature and performant

---

## Project Structure & Setup

### Directory Layout

```
6DoF/
├── firmware/                      # Teensy C++ code
│   ├── firmware.ino              # Main sketch
│   ├── config.h                  # Hardware configuration
│   ├── types.h                   # Data structures
│   ├── StepperController.h/.cpp  # Motor control
│   ├── HomingController.h/.cpp   # Endstop & homing
│   └── SerialProtocol.h/.cpp     # Communication
│
├── web-app/                       # React application
│   ├── public/
│   │   └── models/
│   │       ├── robot.urdf        # URDF file (MUST be provided)
│   │       └── *.stl             # 3D meshes (MUST be provided)
│   │
│   ├── src/
│   │   ├── components/
│   │   │   ├── ConnectionPanel.tsx
│   │   │   ├── JointControlPanel.tsx
│   │   │   ├── CartesianControlPanel.tsx
│   │   │   ├── Viewer3D.tsx
│   │   │   ├── PathPlanner.tsx
│   │   │   ├── GCodeUploader.tsx
│   │   │   ├── StatusBar.tsx
│   │   │   └── EmergencyStop.tsx
│   │   │
│   │   ├── communication/
│   │   │   ├── SerialManager.ts
│   │   │   ├── CommandQueue.ts
│   │   │   └── types.ts
│   │   │
│   │   ├── kinematics/
│   │   │   ├── DHParameters.ts
│   │   │   ├── ForwardKinematics.ts
│   │   │   ├── InverseKinematics.ts
│   │   │   └── URDFParser.ts
│   │   │
│   │   ├── motion/
│   │   │   ├── TrajectoryPlanner.ts
│   │   │   ├── VelocityProfile.ts
│   │   │   └── PathInterpolator.ts
│   │   │
│   │   ├── utils/
│   │   │   ├── math.ts
│   │   │   ├── gcode.ts
│   │   │   └── transforms.ts
│   │   │
│   │   ├── store/
│   │   │   └── robotStore.ts
│   │   │
│   │   ├── types/
│   │   │   └── robot.ts
│   │   │
│   │   ├── App.tsx
│   │   └── index.tsx
│   │
│   ├── package.json
│   ├── tsconfig.json
│   └── vite.config.ts
│
├── docs/
│   ├── SERIAL_PROTOCOL.md
│   ├── CALIBRATION_GUIDE.md
│   └── USER_MANUAL.md
│
├── tests/
│   ├── firmware/                  # Arduino test sketches
│   └── web-app/                   # Jest unit tests
│
├── visualisation.md               # Project overview
├── QUICK_START_CHECKLIST.md       # Implementation guide
├── ROBOT_ARM_CODING_SPEC.md       # Detailed code spec
└── IMPLEMENTATION_PLAN.md         # This file
```

### Initial Setup Tasks

**Week 0: Prerequisites** (1-2 days)

1. **Hardware Verification**
   - [ ] Verify all motors are connected
   - [ ] Test endstops manually (multimeter continuity test)
   - [ ] Measure driver Vref settings
   - [ ] Confirm 24V power supply capacity (≥5A recommended)
   - [ ] Test E-stop circuit

2. **Software Environment**
   - [ ] Install Arduino IDE 2.x or PlatformIO
   - [ ] Install Teensyduino
   - [ ] Install Node.js 18+ LTS
   - [ ] Install Chrome or Edge browser
   - [ ] Install VS Code with extensions:
     - C/C++ (Microsoft)
     - Arduino
     - ESLint
     - Prettier
     - TypeScript

3. **URDF & STL Files**
   - [ ] Obtain robot URDF file (CRITICAL - can't proceed without this)
   - [ ] Obtain STL mesh files for 3D visualization
   - [ ] Validate URDF structure (check with online URDF viewer)

**CRITICAL DECISION POINT**: Do we have URDF/STL files?
- **YES** → Proceed with plan
- **NO** → Need to create them first (see Appendix A)

---

## Phase 1: Foundation & Basic Control

**Duration**: Week 1-2 (10-14 days)
**Priority**: ⭐⭐⭐⭐⭐ (Highest)
**Goal**: Establish working firmware and basic web app communication

### 1.1 Firmware Implementation

#### Step 1.1.1: Create Project Structure (Day 1)

**Files to create:**
```
firmware/
├── firmware.ino
├── config.h
├── types.h
├── StepperController.h
├── StepperController.cpp
├── HomingController.h
├── HomingController.cpp
├── SerialProtocol.h
└── SerialProtocol.cpp
```

**Implementation order:**
1. Start with `config.h` - defines all hardware constants
2. Create `types.h` - data structures
3. Implement `StepperController` - motor control core
4. Implement `HomingController` - endstop handling
5. Implement `SerialProtocol` - communication layer
6. Wire everything in `firmware.ino`

**Code source**: Use ROBOT_ARM_CODING_SPEC.md sections 2.2-2.10

#### Step 1.1.2: Upload and Test Firmware (Day 1-2)

**Testing procedure:**
```cpp
// Test 1: Compile and upload
// Expected: No errors, "OK Robot arm ready" in Serial Monitor

// Test 2: Enable motors
// Send: E 1
// Expected: "OK Motors enabled", LED on pin 8 goes LOW

// Test 3: Move single joint
// Send: J 10 0 0 0 0 0 20
// Expected: "OK Moving", J1 rotates ~10 degrees

// Test 4: Home joint 2
// Send: H 2
// Expected: Motor moves until endstop triggers, backs off, "HOMED 2"

// Test 5: Emergency stop
// Send: S
// Expected: Immediate stop, "OK Emergency stop"
```

**Debugging checklist:**
- [ ] Serial baud rate = 115200
- [ ] Enable pin wired correctly (active LOW)
- [ ] Step pulses visible on oscilloscope (5µs width)
- [ ] Endstops read LOW when triggered
- [ ] Position updates every 100ms

**Success criteria:**
- ✅ All 6 motors can move independently
- ✅ Homing works for J2-J5
- ✅ Position feedback is accurate (±1 degree)
- ✅ E-stop halts motion immediately

#### Step 1.1.3: Calibration & Tuning (Day 2)

**Microsteps per degree verification:**
```cpp
// For each joint:
// 1. Home the joint
// 2. Command 360° rotation: J 360 0 0 0 0 0 20
// 3. Measure actual rotation with protractor/encoder
// 4. Adjust USTEPS_PER_DEG if error > 5°
```

**Known values (from spec):**
- J1: 55.556 µsteps/°
- J2: 222.222 µsteps/°
- J3: 55.556 µsteps/°
- J4: 33.333 µsteps/°
- J5: 17.778 µsteps/°
- J6: 8.889 µsteps/°

**IMPORTANT**: Verify these empirically - gear ratios can vary!

### 1.2 Web App Foundation

#### Step 1.2.1: React Project Setup (Day 3)

```bash
# Create project
npx create-react-app robot-arm-control --template typescript
cd robot-arm-control

# Install core dependencies
npm install three @react-three/fiber @react-three/drei
npm install urdf-loader
npm install zustand
npm install mathjs

# Install UI libraries
npm install @mui/material @emotion/react @emotion/styled
npm install lucide-react  # For icons

# Install dev dependencies
npm install -D @types/three
npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p
```

**Configure Tailwind** (tailwind.config.js):
```javascript
module.exports = {
  content: ["./src/**/*.{js,jsx,ts,tsx}"],
  theme: { extend: {} },
  plugins: [],
}
```

#### Step 1.2.2: Implement Core Types & Store (Day 3)

**Files to create:**
1. `src/types/robot.ts` - All TypeScript interfaces
2. `src/store/robotStore.ts` - Zustand state management

**Code source**: ROBOT_ARM_CODING_SPEC.md sections 3.3 and 3.6

**Testing:**
```typescript
// Test store initialization
import { useRobotStore } from './store/robotStore';

// Should initialize with default state
const state = useRobotStore.getState();
console.log(state.connectionStatus); // 'disconnected'
console.log(state.currentAngles); // All zeros
```

#### Step 1.2.3: Implement Serial Communication (Day 4)

**Files:**
1. `src/communication/types.ts`
2. `src/communication/SerialManager.ts`

**Code source**: ROBOT_ARM_CODING_SPEC.md sections 3.4 and 3.5

**Critical implementation notes:**
```typescript
// Web Serial API permissions
// User MUST click a button to trigger requestPort()
// Cannot auto-connect on page load!

async connect() {
  try {
    // This MUST be called from user gesture (button click)
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: 115200 });
    // ...
  } catch (error) {
    // User cancelled or error
  }
}
```

**Browser compatibility check:**
```typescript
// Add to App.tsx
useEffect(() => {
  if (!SerialManager.isSupported()) {
    alert('Web Serial API not supported. Please use Chrome or Edge browser.');
  }
}, []);
```

#### Step 1.2.4: Build Connection UI (Day 4)

**File:** `src/components/ConnectionPanel.tsx`

**Code source**: ROBOT_ARM_CODING_SPEC.md section 3.7

**UI features:**
- Status indicator (colored dot: red/yellow/green)
- "Connect to Robot" button
- "Disconnect" button
- Connection error messages

**Testing:**
1. Click "Connect to Robot"
2. Select Teensy COM port
3. Should see status change: disconnected → connecting → connected
4. Should see position updates in console

#### Step 1.2.5: Build Joint Control Panel (Day 5)

**File:** `src/components/JointControlPanel.tsx`

**Code source**: ROBOT_ARM_CODING_SPEC.md section 3.8

**Features:**
- 6 sliders for joint angles (with min/max from config)
- Current vs. Target angle display
- Speed slider (5-100 °/s)
- "Move to Target" button
- Real-time position feedback

**Layout:**
```
┌─────────────────────────────────────┐
│  Manual Joint Control               │
├─────────────────────────────────────┤
│  Speed: [====●========] 30°/s       │
├─────────────────────────────────────┤
│  J1  Current: 0.0°  Target: 10.0°   │
│      [-40] ─────●──────────── [30]  │
│                                      │
│  J2  Current: 5.0°  Target: 20.0°   │
│      [0] ───────────●─────── [60]   │
│  ...                                 │
├─────────────────────────────────────┤
│  [   Move to Target   ]              │
└─────────────────────────────────────┘
```

#### Step 1.2.6: Build Status Bar & E-Stop (Day 5)

**Files:**
- `src/components/StatusBar.tsx`
- `src/components/EmergencyStop.tsx`

**Status Bar content:**
- Connection status
- Robot state (IDLE/MOVING/HOMING/ERROR)
- Current XYZ position (from FK - implement basic version)
- Endstop indicators (6 LEDs)

**E-Stop button:**
- Large red button
- Always visible (sticky position)
- Sends 'S' command immediately
- Confirmation dialog before re-enabling

#### Step 1.2.7: Assemble Main App (Day 6)

**File:** `src/App.tsx`

**Layout:**
```
┌────────────────────────────────────────────┐
│  ConnectionPanel                           │
├──────────────┬─────────────────────────────┤
│              │                             │
│  JointControl│      3D Viewer              │
│              │   (Placeholder for now)     │
│              │                             │
│              │                             │
│              │                             │
├──────────────┴─────────────────────────────┤
│  StatusBar                                 │
└────────────────────────────────────────────┘
              [E-STOP]
```

**App structure:**
```typescript
function App() {
  return (
    <div className="h-screen flex flex-col">
      <ConnectionPanel />
      <div className="flex-1 flex">
        <div className="w-1/3 overflow-y-auto border-r">
          <JointControlPanel />
        </div>
        <div className="flex-1 bg-gray-100">
          {/* 3D Viewer placeholder */}
          <div className="flex items-center justify-center h-full">
            <p className="text-gray-500">3D Viewer (Coming in Phase 3)</p>
          </div>
        </div>
      </div>
      <StatusBar />
      <EmergencyStop />
    </div>
  );
}
```

### 1.3 Integration Testing (Day 7)

**End-to-end test sequence:**

```markdown
## Integration Test Protocol

### Test 1: Connection
1. Start web app (npm start)
2. Click "Connect to Robot"
3. Select COM port
✅ Status shows "Connected"
✅ Position updates appear in console
✅ Status bar shows current joint angles

### Test 2: Manual Joint Control
1. Enable motors (may need UI toggle)
2. Move J1 slider to 15°
3. Click "Move to Target"
✅ Robot moves to 15°
✅ Current angle updates to ~15°
✅ Status changes: IDLE → MOVING → IDLE

### Test 3: Multi-Joint Movement
1. Set target: J1=10, J2=20, J3=30
2. Click "Move to Target"
✅ All 3 joints move simultaneously
✅ All finish at same time (coordinated motion)

### Test 4: Homing
1. Add homing buttons to UI (H 2, H 3, H 4, H 5, H ALL)
2. Click "Home J2"
✅ Motor moves toward endstop
✅ Stops when triggered
✅ Backs off and re-approaches slowly
✅ Sets zero position
✅ Moves to post-home angle (5°)

### Test 5: Emergency Stop
1. Start a long movement (J1: -40 to 30)
2. Click E-Stop mid-motion
✅ Motors stop immediately
✅ Target position updated to current
✅ Robot state = ESTOPPED

### Test 6: Limits
1. Try to move J1 to -50° (beyond limit of -40°)
✅ Firmware clamps to -40°
✅ No error message (graceful handling)
```

### Phase 1 Deliverables

✅ **Firmware:**
- All 6 motors controlled via serial commands
- Homing functional for J2-J5
- Position feedback (100ms updates)
- E-stop working

✅ **Web App:**
- Connection to robot via Web Serial
- Manual joint control with sliders
- Real-time position display
- Status monitoring
- E-stop button

✅ **Documentation:**
- Serial protocol examples
- Connection troubleshooting guide

**EXIT CRITERIA:**
- [ ] Can connect to robot in <10 seconds
- [ ] All 6 joints move smoothly
- [ ] Position error <1° after movement
- [ ] Homing repeatable within 0.5°
- [ ] E-stop response time <100ms
- [ ] No crashes or disconnects during 30min stress test

---

## Phase 2: Kinematics Implementation

**Duration**: Week 3-4 (10-14 days)
**Priority**: ⭐⭐⭐⭐
**Goal**: Enable Cartesian coordinate control

### 2.1 URDF Parsing & DH Parameters (Day 8-9)

#### Critical Requirement: URDF File

**URDF file structure check:**
```xml
<?xml version="1.0"?>
<robot name="6dof_arm">
  <link name="base_link">...</link>

  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.698" upper="0.524" .../>
  </joint>

  <link name="link1">...</link>
  <!-- ... more joints and links ... -->
</robot>
```

**Task 2.1.1: Place URDF in project**
```bash
# Copy URDF and STL files
cp /path/to/robot.urdf web-app/public/models/
cp /path/to/*.stl web-app/public/models/

# Verify file structure
web-app/public/models/
├── robot.urdf
├── base_link.stl
├── link1.stl
├── link2.stl
├── link3.stl
├── link4.stl
├── link5.stl
└── link6.stl
```

**Task 2.1.2: Extract DH Parameters**

Create: `src/kinematics/URDFParser.ts`

**Code source**: ROBOT_ARM_CODING_SPEC.md section 4.3

**Manual DH parameter extraction:**
```typescript
// If URDF parsing is complex, manually define DH parameters
// Measure from URDF or physical robot

export const DH_PARAMETERS = [
  // J1 - Base rotation
  {
    theta: 0,           // Variable
    d: 0.100,          // Base height (measured from URDF)
    a: 0,              // No X offset
    alpha: Math.PI/2,  // 90° twist
    jointType: 'revolute',
    offsetAngle: 0
  },
  // J2 - Shoulder
  {
    theta: 0,
    d: 0,
    a: 0.150,          // Upper arm length
    alpha: 0,
    jointType: 'revolute',
    offsetAngle: 0
  },
  // ... continue for J3-J6
];
```

**Validation method:**
```typescript
// Test: Does zero configuration give expected position?
const fk = new ForwardKinematics();
const result = fk.compute({ J1:0, J2:0, J3:0, J4:0, J5:0, J6:0 });

// Manually calculate expected position:
// X = a2 + a3 + a4 + tool_length
// Y = 0 (straight forward)
// Z = d1 (base height)

console.log(`Expected: X=${expectedX}, Y=0, Z=${d1}`);
console.log(`Actual: X=${result.position.x}, Y=${result.position.y}, Z=${result.position.z}`);

// Tolerance: <5mm error acceptable for now
```

### 2.2 Forward Kinematics (Day 10-11)

**Files to create:**
1. `src/kinematics/DHParameters.ts`
2. `src/kinematics/ForwardKinematics.ts`
3. `src/utils/math.ts` (matrix operations)

**Code source**: ROBOT_ARM_CODING_SPEC.md sections 4.1 and 4.2

**Implementation steps:**

1. **Implement DH transformation matrix**
   ```typescript
   // Key formula:
   // T = Rot_Z(θ) * Trans_Z(d) * Trans_X(a) * Rot_X(α)
   ```

2. **Implement FK solver**
   ```typescript
   // Multiply 6 transformation matrices
   // T_total = T1 * T2 * T3 * T4 * T5 * T6
   ```

3. **Extract position and orientation**
   ```typescript
   // Position: [T[0,3], T[1,3], T[2,3]]
   // Orientation: Extract Euler angles from rotation part
   ```

**Testing FK:**

```typescript
// Test cases (define based on robot geometry)
const testCases = [
  {
    name: "Zero position",
    angles: { J1:0, J2:0, J3:0, J4:0, J5:0, J6:0 },
    expectedPos: { x: 0.3, y: 0, z: 0.1 }  // Adjust based on robot
  },
  {
    name: "J1 = 90°",
    angles: { J1:90, J2:0, J3:0, J4:0, J5:0, J6:0 },
    expectedPos: { x: 0, y: 0.3, z: 0.1 }  // Rotated 90° in XY plane
  },
  {
    name: "J2 = 45°",
    angles: { J1:0, J2:45, J3:0, J4:0, J5:0, J6:0 },
    expectedPos: { x: 0.25, y: 0, z: 0.25 }  // Lifted and pulled back
  }
];

testCases.forEach(test => {
  const result = fk.compute(test.angles);
  const error = Math.sqrt(
    (result.position.x - test.expectedPos.x)**2 +
    (result.position.y - test.expectedPos.y)**2 +
    (result.position.z - test.expectedPos.z)**2
  );
  console.log(`${test.name}: Error = ${error}mm`);
  assert(error < 5, "FK error too large");
});
```

**Integration with UI:**

Update `StatusBar.tsx` to show XYZ position:
```typescript
const { currentAngles } = useRobotStore();
const fk = new ForwardKinematics();
const cartesianPose = fk.compute(currentAngles);

return (
  <div>
    Position: X={cartesianPose.position.x.toFixed(3)}m
              Y={cartesianPose.position.y.toFixed(3)}m
              Z={cartesianPose.position.z.toFixed(3)}m
  </div>
);
```

### 2.3 Inverse Kinematics (Day 12-15)

**File:** `src/kinematics/InverseKinematics.ts`

**Code source**: ROBOT_ARM_CODING_SPEC.md section 5.1

**IK Implementation Strategy:**

**Option 1: Analytical IK (Preferred if possible)**
- Faster and more reliable
- Returns multiple solutions (elbow up/down, wrist configurations)
- Requires specific robot geometry (spherical wrist ideal)

**Option 2: Numerical IK (Fallback)**
- Works for any robot
- Slower (iterative)
- May not converge near singularities

**Implementation phases:**

#### Phase 2.3.1: Analytical IK for Position (Day 12-13)

```typescript
// Solve for J1, J2, J3 (position only)
// Decouple wrist position from end-effector position

// Step 1: Compute wrist center
const wristCenter = targetPos - R_target * [0, 0, wrist_offset];

// Step 2: Solve J1 (base rotation)
// J1 = atan2(wc_y, wc_x)
// Note: Two solutions (±180°)

// Step 3: Solve J2, J3 (2-link planar arm in vertical plane)
// Use law of cosines
// J3 = acos((d² - L2² - L3²) / (2*L2*L3))
// J2 = atan2(s, r) - atan2(L3*sin(J3), L2 + L3*cos(J3))
```

**Testing position IK:**
```typescript
// Test: FK → IK → FK round-trip
const originalAngles = { J1:10, J2:20, J3:30, J4:0, J5:0, J6:0 };
const fkResult = fk.compute(originalAngles);

const ikSolutions = ik.solvePosition(
  fkResult.position.x,
  fkResult.position.y,
  fkResult.position.z
);

// Should find at least one solution close to original
const bestSolution = ikSolutions[0];
console.log("Original:", originalAngles);
console.log("IK solution:", bestSolution.angles);

// Tolerance: ±5° acceptable
```

#### Phase 2.3.2: Analytical IK for Orientation (Day 14)

```typescript
// Solve for J4, J5, J6 (spherical wrist)
// R_target = R_0_3(J1,J2,J3) * R_3_6(J4,J5,J6)
// R_3_6 = R_0_3^T * R_target

// Extract Euler angles from R_3_6
// J5 = acos(r33)
// J4 = atan2(r23/sin(J5), r13/sin(J5))
// J6 = atan2(r32/sin(J5), -r31/sin(J5))
```

**Singularity handling:**
```typescript
// Singularity when J5 = 0° or 180°
// In this case, J4 and J6 are not uniquely determined
// Choose J4 = 0 and solve for J6

if (Math.abs(Math.sin(j5)) < 0.01) {
  console.warn("Near singularity - solution may be unstable");
  j4 = 0;  // Arbitrary choice
  j6 = /* solve with j4=0 */;
}
```

#### Phase 2.3.3: Numerical IK (Day 15)

Implement Jacobian-based numerical IK as fallback:

```typescript
// Iterative algorithm:
// 1. Compute current pose: p_current = FK(q_current)
// 2. Compute error: Δp = p_target - p_current
// 3. Compute Jacobian: J = ∂FK/∂q
// 4. Update joints: Δq = J^T * Δp (Jacobian transpose method)
//    OR: Δq = J^+ * Δp (pseudoinverse - more stable)
// 5. q_new = q_current + α * Δq (α = step size)
// 6. Repeat until error < tolerance or max iterations
```

**When to use numerical IK:**
- Analytical IK fails to find solution
- Near singularities
- Complex target orientations

### 2.4 Cartesian Control UI (Day 16-17)

**File:** `src/components/CartesianControlPanel.tsx`

**Features:**
1. XYZ position input (numeric fields or sliders)
2. Orientation input (Euler angles or quaternion)
3. "Move to Position" button
4. Solution selector (if multiple IK solutions)
5. Reachability indicator (green/red)

**UI Layout:**
```
┌─────────────────────────────────────┐
│  Cartesian Control                  │
├─────────────────────────────────────┤
│  Position (meters)                  │
│  X: [  0.250  ] ◀ ▶                │
│  Y: [  0.000  ] ◀ ▶                │
│  Z: [  0.150  ] ◀ ▶                │
│                                      │
│  Orientation (degrees)               │
│  Roll:  [  0  ] ◀ ▶                 │
│  Pitch: [  0  ] ◀ ▶                 │
│  Yaw:   [  0  ] ◀ ▶                 │
├─────────────────────────────────────┤
│  IK Solutions: ● Elbow-up           │
│                ○ Elbow-down         │
│                ○ Alternative        │
├─────────────────────────────────────┤
│  Status: ✅ Reachable               │
│  [ Move to Position ]               │
└─────────────────────────────────────┘
```

**Implementation:**
```typescript
const CartesianControlPanel: React.FC = () => {
  const [targetPosition, setTargetPosition] = useState({ x: 0.25, y: 0, z: 0.15 });
  const [targetOrientation, setTargetOrientation] = useState({ x: 0, y: 0, z: 0 });
  const [ikSolutions, setIkSolutions] = useState<IKSolution[]>([]);
  const [selectedSolution, setSelectedSolution] = useState(0);

  const { currentAngles, moveToTarget } = useRobotStore();

  const handleSolveIK = () => {
    const ik = new InverseKinematics(new ForwardKinematics());
    const solutions = ik.solve({
      position: targetPosition,
      orientation: targetOrientation
    });

    setIkSolutions(solutions);

    if (solutions.length > 0 && solutions[0].reachable) {
      // Preview first solution
      setSelectedSolution(0);
    }
  };

  const handleMove = async () => {
    if (ikSolutions.length === 0) return;

    const solution = ikSolutions[selectedSolution];
    if (!solution.reachable) {
      alert("Target position unreachable!");
      return;
    }

    // Send to robot
    await moveToTarget(solution.angles);
  };

  // Auto-solve when position changes (debounced)
  useEffect(() => {
    const timeout = setTimeout(handleSolveIK, 500);
    return () => clearTimeout(timeout);
  }, [targetPosition, targetOrientation]);

  return (/* ... UI ... */);
};
```

**Workspace visualization:**
```typescript
// Add workspace boundary check
const isReachable = (pos: Vector3): boolean => {
  const distance = Math.sqrt(pos.x**2 + pos.y**2 + pos.z**2);
  const maxReach = L2 + L3 + L4 + L5;  // Sum of link lengths
  const minReach = Math.abs(L2 - L3);   // Minimum reach

  return distance >= minReach && distance <= maxReach;
};

// Display reachability
<div className={isReachable(targetPosition) ? 'text-green-600' : 'text-red-600'}>
  {isReachable(targetPosition) ? '✅ Reachable' : '❌ Out of workspace'}
</div>
```

### 2.5 Phase 2 Testing & Validation (Day 18)

**Test suite:**

```markdown
## IK/FK Validation Tests

### Test 1: FK Accuracy
1. Home all joints
2. Move to 10 known positions
3. Measure actual position with caliper/ruler
4. Compare with FK calculation
✅ Error <5mm for all positions

### Test 2: IK Round-Trip
For 20 random joint configurations:
1. Compute FK: pose = FK(angles)
2. Compute IK: solutions = IK(pose)
3. Take best solution
4. Compute FK again: pose2 = FK(solutions[0].angles)
5. Compare pose vs pose2
✅ Position error <1mm
✅ Orientation error <1°

### Test 3: Multiple Solutions
1. Choose target with multiple IK solutions
2. Verify all solutions reach same end-effector pose
3. Test switching between solutions
✅ All solutions valid
✅ Can select preferred configuration

### Test 4: Cartesian UI
1. Enter target position in UI: (0.2, 0.1, 0.15)
2. Verify IK solution displayed
3. Click "Move to Position"
✅ Robot reaches target within 5mm
✅ Position stable (no oscillation)

### Test 5: Workspace Limits
1. Try position at max reach
2. Try position beyond max reach
✅ At max reach: solution found
✅ Beyond reach: "Unreachable" message

### Test 6: Singularities
1. Move to singularity configuration (e.g., J5=0)
2. Command small Cartesian move
✅ Warning displayed
✅ Fallback to numerical IK if needed
```

### Phase 2 Deliverables

✅ **Kinematics:**
- Forward kinematics validated (<5mm error)
- Inverse kinematics with multiple solutions
- Numerical IK fallback
- Jacobian computation

✅ **UI:**
- Cartesian position control
- IK solution selector
- Reachability indicator

✅ **Testing:**
- FK/IK test suite
- Calibration procedure documented

**EXIT CRITERIA:**
- [ ] IK success rate >95% within workspace
- [ ] FK error <5mm across full range
- [ ] Round-trip error <2mm
- [ ] Can reach commanded XYZ within 5mm
- [ ] UI updates in <100ms

---

## Phase 3: 3D Visualization

**Duration**: Week 5 (5-7 days)
**Priority**: ⭐⭐⭐
**Goal**: Real-time 3D visualization of robot

### 3.1 Three.js Scene Setup (Day 19-20)

**File:** `src/components/Viewer3D.tsx`

**Code source**: ROBOT_ARM_CODING_SPEC.md Appendix A

**Dependencies check:**
```bash
npm list three @react-three/fiber @react-three/drei urdf-loader

# Should see:
# ├── three@0.150.0
# ├── @react-three/fiber@8.x
# ├── @react-three/drei@9.x
# └── urdf-loader@0.x
```

**Implementation steps:**

1. **Create basic scene**
   ```typescript
   import { Canvas } from '@react-three/fiber';
   import { OrbitControls } from '@react-three/drei';

   export const Viewer3D: React.FC = () => {
     return (
       <Canvas camera={{ position: [1, 1, 1], fov: 50 }}>
         <ambientLight intensity={0.5} />
         <directionalLight position={[5, 5, 5]} />
         <OrbitControls />
         {/* Robot will go here */}
       </Canvas>
     );
   };
   ```

2. **Add grid and axes**
   ```typescript
   import { Grid, axesHelper } from '@react-three/drei';

   <Grid args={[10, 10]} cellSize={0.1} />
   <primitive object={new THREE.AxesHelper(0.5)} />
   ```

3. **Test rendering**
   - Replace placeholder in App.tsx with Viewer3D
   - Should see empty 3D scene with grid
   - Mouse controls: rotate (left drag), pan (right drag), zoom (scroll)

### 3.2 URDF Loading (Day 20-21)

**Critical: URDF file location**
```
web-app/public/models/robot.urdf  ← Must be here!
```

**URDF Loader implementation:**
```typescript
import URDFLoader from 'urdf-loader';

const RobotModel: React.FC = () => {
  const groupRef = useRef<THREE.Group>(null);
  const [robot, setRobot] = useState<any>(null);

  useEffect(() => {
    const loader = new URDFLoader();

    // Set mesh loader for STL files
    loader.packages = {
      '': '/models/'  // Base path for mesh files
    };

    loader.load(
      '/models/robot.urdf',
      (loadedRobot) => {
        console.log('Robot loaded:', loadedRobot);
        setRobot(loadedRobot);

        if (groupRef.current) {
          groupRef.current.add(loadedRobot);
        }
      },
      undefined,
      (error) => {
        console.error('URDF load error:', error);
        alert('Failed to load robot model. Check console for details.');
      }
    );
  }, []);

  return <group ref={groupRef} />;
};
```

**Troubleshooting URDF loading:**

Common issues:
1. **CORS errors**: URDF file must be in public folder
2. **STL not found**: Check mesh file paths in URDF
3. **Joint names mismatch**: Verify joint names match spec

**URDF validation:**
```bash
# Install URDF checker (Python)
pip install urdfpy

# Validate URDF
python -c "import urdfpy; urdfpy.URDF.load('robot.urdf')"

# Should print robot structure without errors
```

### 3.3 Joint Animation (Day 21-22)

**Update robot joints in real-time:**

```typescript
const RobotModel: React.FC = () => {
  const { currentAngles } = useRobotStore();
  const [robot, setRobot] = useState<any>(null);

  // ... URDF loading code ...

  // Update joint angles when currentAngles changes
  useEffect(() => {
    if (!robot || !robot.joints) return;

    // Map joint angles to URDF joint names
    const jointMapping = {
      J1: 'joint1',  // Adjust names to match URDF
      J2: 'joint2',
      J3: 'joint3',
      J4: 'joint4',
      J5: 'joint5',
      J6: 'joint6'
    };

    Object.entries(jointMapping).forEach(([key, jointName]) => {
      const joint = robot.joints[jointName];
      if (joint) {
        const angleRad = currentAngles[key as keyof JointAngles] * Math.PI / 180;
        joint.setJointValue(angleRad);
      }
    });

    // Force re-render
    robot.updateMatrixWorld(true);

  }, [currentAngles, robot]);

  return <group ref={groupRef} />;
};
```

**Performance optimization:**
```typescript
// Use frame-based updates instead of every state change
import { useFrame } from '@react-three/fiber';

useFrame(() => {
  if (!robot) return;

  // Update joint values
  // This runs at 60fps but only updates if values changed
  Object.entries(jointMapping).forEach(([key, jointName]) => {
    const joint = robot.joints[jointName];
    if (joint) {
      const targetAngle = currentAngles[key as keyof JointAngles] * Math.PI / 180;
      const currentAngle = joint.angle || 0;

      // Smooth interpolation
      const newAngle = currentAngle + (targetAngle - currentAngle) * 0.1;
      joint.setJointValue(newAngle);
    }
  });
});
```

### 3.4 Enhanced Visualization (Day 23)

**Add visualization features:**

1. **End-effector position marker**
   ```typescript
   const EndEffectorMarker: React.FC<{ position: Vector3 }> = ({ position }) => {
     return (
       <mesh position={position}>
         <sphereGeometry args={[0.02, 16, 16]} />
         <meshStandardMaterial color="red" emissive="red" emissiveIntensity={0.5} />
       </mesh>
     );
   };
   ```

2. **Workspace boundary**
   ```typescript
   const WorkspaceBoundary: React.FC = () => {
     const maxReach = 0.5;  // meters
     return (
       <mesh>
         <sphereGeometry args={[maxReach, 32, 32]} />
         <meshBasicMaterial color="blue" opacity={0.1} transparent wireframe />
       </mesh>
     );
   };
   ```

3. **Path visualization**
   ```typescript
   const PathLine: React.FC<{ points: Vector3[] }> = ({ points }) => {
     const geometry = new THREE.BufferGeometry().setFromPoints(points);
     return (
       <line geometry={geometry}>
         <lineBasicMaterial color="green" linewidth={2} />
       </line>
     );
   };
   ```

4. **Joint axes**
   ```typescript
   // Show rotation axis for each joint
   const JointAxes: React.FC = () => {
     const { robot } = useRobotModel();

     return (
       <>
         {Object.values(robot.joints).map((joint, i) => {
           const axis = joint.axis;  // From URDF
           return (
             <arrowHelper
               key={i}
               args={[axis, joint.position, 0.1, 0xff0000]}
             />
           );
         })}
       </>
     );
   };
   ```

### 3.5 Camera Controls & Presets (Day 23)

**Enhanced camera controls:**

```typescript
import { OrbitControls, PerspectiveCamera } from '@react-three/drei';

const Viewer3D: React.FC = () => {
  const cameraRef = useRef();

  const cameraPresets = {
    front: { position: [1, 0, 0.5], target: [0, 0, 0.2] },
    side: { position: [0, 1, 0.5], target: [0, 0, 0.2] },
    top: { position: [0, 0, 1.5], target: [0, 0, 0] },
    iso: { position: [0.7, 0.7, 0.7], target: [0, 0, 0.2] }
  };

  const setCamera = (preset: keyof typeof cameraPresets) => {
    const { position, target } = cameraPresets[preset];
    // Animate camera to preset
    // (use tween library or manual interpolation)
  };

  return (
    <div className="relative w-full h-full">
      {/* Camera preset buttons */}
      <div className="absolute top-4 right-4 z-10 space-x-2">
        <button onClick={() => setCamera('front')}>Front</button>
        <button onClick={() => setCamera('side')}>Side</button>
        <button onClick={() => setCamera('top')}>Top</button>
        <button onClick={() => setCamera('iso')}>Iso</button>
      </div>

      <Canvas>
        <PerspectiveCamera ref={cameraRef} position={[1,1,1]} />
        <OrbitControls
          enableDamping
          dampingFactor={0.05}
          minDistance={0.3}
          maxDistance={3}
        />
        {/* ... scene content ... */}
      </Canvas>
    </div>
  );
};
```

### 3.6 Phase 3 Testing (Day 24)

**Visual testing checklist:**

```markdown
## 3D Visualization Tests

### Test 1: Model Loading
✅ Robot model loads without errors
✅ All links visible
✅ Meshes textured/colored correctly
✅ No missing geometries

### Test 2: Joint Animation
1. Move J1 slider from -40° to +30°
✅ Base link rotates correctly
✅ Smooth animation (no jerking)
✅ Direction correct (CW/CCW)

2. Repeat for all 6 joints
✅ Each joint moves independently
✅ Child links follow parent correctly

### Test 3: Real-time Sync
1. Move robot to known position
2. Compare 3D view with physical robot
✅ Position matches within visual tolerance
✅ Update latency <100ms

### Test 4: Camera Controls
✅ Orbit: Drag to rotate
✅ Pan: Right-click drag
✅ Zoom: Scroll wheel
✅ Preset buttons work
✅ Auto-focus on robot

### Test 5: Performance
✅ Maintains 60 FPS with robot moving
✅ No lag when updating joint angles
✅ Smooth camera movement
```

### Phase 3 Deliverables

✅ **3D Viewer:**
- URDF-based robot model
- Real-time joint updates
- Camera controls (orbit, pan, zoom)
- Camera presets

✅ **Visual Features:**
- Grid and axes
- End-effector marker
- Workspace boundary (optional)
- Joint axes display (optional)

**EXIT CRITERIA:**
- [ ] Robot model renders correctly
- [ ] All 6 joints animate smoothly
- [ ] Position sync with physical robot <100ms lag
- [ ] 60 FPS performance
- [ ] Camera controls intuitive

---

## Phase 4: Advanced Motion Control

**Duration**: Week 6-7 (10-12 days)
**Priority**: ⭐⭐⭐
**Goal**: Path planning and trajectory execution

### 4.1 Trajectory Planning (Day 25-27)

**File:** `src/motion/TrajectoryPlanner.ts`

**Code source**: ROBOT_ARM_CODING_SPEC.md section 6.1

**Implementation components:**

1. **Velocity Profile Generator**
   - Trapezoidal profile (simpler)
   - S-curve profile (smoother - implement later)

2. **Path Interpolator**
   - Linear interpolation in joint space
   - Linear interpolation in Cartesian space
   - Circular arcs (for G-code)

3. **Multi-joint Coordinator**
   - Time-scale all joints to finish simultaneously
   - Maintain coordinated motion

**Trapezoidal profile implementation:**

```typescript
class VelocityProfile {
  private accelTime: number;
  private constTime: number;
  private decelTime: number;
  private totalTime: number;

  constructor(
    distance: number,
    maxVel: number,
    maxAccel: number
  ) {
    // Calculate profile parameters
    this.accelTime = maxVel / maxAccel;
    const accelDist = 0.5 * maxAccel * this.accelTime**2;

    if (2 * accelDist >= Math.abs(distance)) {
      // Triangle profile (no constant velocity phase)
      const peakVel = Math.sqrt(Math.abs(distance) * maxAccel);
      this.accelTime = peakVel / maxAccel;
      this.constTime = 0;
      this.decelTime = this.accelTime;
    } else {
      // Trapezoidal profile
      this.constTime = (Math.abs(distance) - 2 * accelDist) / maxVel;
      this.decelTime = this.accelTime;
    }

    this.totalTime = this.accelTime + this.constTime + this.decelTime;
  }

  // Get position at time t
  getPosition(t: number, totalDistance: number): number {
    const sign = Math.sign(totalDistance);
    const dist = Math.abs(totalDistance);

    if (t <= this.accelTime) {
      // Acceleration phase: x = 0.5 * a * t²
      const a = dist / (this.accelTime * (this.accelTime + this.constTime));
      return sign * 0.5 * a * t**2;
    }

    if (t <= this.accelTime + this.constTime) {
      // Constant velocity phase
      const v = dist / (this.accelTime + this.constTime);
      const xAccel = v * this.accelTime / 2;
      return sign * (xAccel + v * (t - this.accelTime));
    }

    // Deceleration phase
    const tDecel = t - this.accelTime - this.constTime;
    const v = dist / (this.accelTime + this.constTime);
    const xAccel = v * this.accelTime / 2;
    const xConst = v * this.constTime;
    const a = v / this.decelTime;
    return sign * (xAccel + xConst + v * tDecel - 0.5 * a * tDecel**2);
  }

  getDuration(): number {
    return this.totalTime;
  }
}
```

**Testing velocity profiles:**
```typescript
// Test: Distance = 100°, MaxVel = 50°/s, MaxAccel = 100°/s²
const profile = new VelocityProfile(100, 50, 100);
const duration = profile.getDuration();  // Should be ~2.5s

// Sample at 10Hz
for (let t = 0; t <= duration; t += 0.1) {
  const pos = profile.getPosition(t, 100);
  console.log(`t=${t.toFixed(1)}s, pos=${pos.toFixed(2)}°`);
}

// Verify:
// - Position at t=0 is 0
// - Position at t=duration is 100
// - Velocity increases linearly in accel phase
// - Position curve is smooth (continuous second derivative)
```

### 4.2 Waypoint System (Day 28-29)

**File:** `src/components/PathPlanner.tsx`

**Features:**
1. Waypoint list (add, delete, reorder)
2. Teach mode (capture current position)
3. Path visualization in 3D
4. Execution controls (play, pause, stop)

**UI Layout:**
```
┌────────────────────────────────────┐
│  Path Planner                      │
├────────────────────────────────────┤
│  Waypoints:                        │
│  #1 ● X:0.25 Y:0.00 Z:0.15  [×][↑]│
│  #2 ● X:0.30 Y:0.10 Z:0.15  [×][↓]│
│  #3 ● X:0.20 Y:0.10 Z:0.20  [×]   │
│                                     │
│  [ Teach Current Position ]         │
│  [ Clear All ]                      │
├────────────────────────────────────┤
│  Speed: [=====●===] 50 mm/s        │
│  Accel: [===●=====] 100 mm/s²      │
├────────────────────────────────────┤
│  Duration: ~12.5s                   │
│  Distance: 0.45m                    │
├────────────────────────────────────┤
│  [▶ Execute] [⏸ Pause] [⏹ Stop]    │
│  Progress: ████████░░░ 75%          │
└────────────────────────────────────┘
```

**State management:**
```typescript
interface Waypoint {
  id: string;
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number };
  speed: number;  // mm/s or deg/s
}

const PathPlanner: React.FC = () => {
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [isExecuting, setIsExecuting] = useState(false);
  const [progress, setProgress] = useState(0);

  const { currentAngles, moveToTarget } = useRobotStore();
  const fk = new ForwardKinematics();

  const handleTeach = () => {
    // Capture current position
    const currentPose = fk.compute(currentAngles);

    const newWaypoint: Waypoint = {
      id: Date.now().toString(),
      position: currentPose.position,
      orientation: currentPose.orientation,
      speed: 50  // Default speed
    };

    setWaypoints([...waypoints, newWaypoint]);
  };

  const handleExecute = async () => {
    setIsExecuting(true);

    const planner = new TrajectoryPlanner(
      new ForwardKinematics(),
      new InverseKinematics(new ForwardKinematics())
    );

    // Plan trajectory
    const segments = planner.planPath(
      waypoints.map(wp => ({
        pose: { position: wp.position, orientation: wp.orientation },
        speed: wp.speed
      })),
      currentAngles
    );

    // Execute each segment
    for (let i = 0; i < segments.length; i++) {
      const segment = segments[i];

      // Send waypoints to robot at 10Hz
      for (const point of segment.points) {
        if (!isExecuting) break;  // Check for stop

        await moveToTarget(point.angles);
        await new Promise(resolve => setTimeout(resolve, 100));  // 10Hz

        setProgress((i + point.time / segment.duration) / segments.length * 100);
      }
    }

    setIsExecuting(false);
    setProgress(0);
  };

  return (/* ... UI ... */);
};
```

### 4.3 Path Visualization (Day 29-30)

**Visualize path in 3D viewer:**

```typescript
// Add to Viewer3D component
const PathVisualization: React.FC<{ waypoints: Waypoint[] }> = ({ waypoints }) => {
  const points = waypoints.map(wp =>
    new THREE.Vector3(wp.position.x, wp.position.y, wp.position.z)
  );

  const geometry = new THREE.BufferGeometry().setFromPoints(points);

  return (
    <>
      {/* Path line */}
      <line geometry={geometry}>
        <lineBasicMaterial color="green" linewidth={3} />
      </line>

      {/* Waypoint markers */}
      {waypoints.map((wp, i) => (
        <mesh key={wp.id} position={[wp.position.x, wp.position.y, wp.position.z]}>
          <sphereGeometry args={[0.015, 16, 16]} />
          <meshStandardMaterial
            color={i === 0 ? 'blue' : i === waypoints.length - 1 ? 'red' : 'green'}
          />
          {/* Label */}
          <Html position={[0, 0.03, 0]}>
            <div className="bg-white px-2 py-1 rounded text-xs">#{i+1}</div>
          </Html>
        </mesh>
      ))}

      {/* Current target indicator */}
      <mesh position={currentTarget}>
        <ringGeometry args={[0.03, 0.04, 32]} />
        <meshBasicMaterial color="yellow" />
      </mesh>
    </>
  );
};
```

### 4.4 Advanced Features (Day 31-32)

**1. Blend zones (smooth transitions):**
```typescript
// Instead of stopping at each waypoint, blend through
// This creates smoother motion but deviates from exact waypoint

const blendRadius = 0.02;  // 2cm

// When approaching waypoint, start transitioning to next segment
// when distance < blendRadius
```

**2. Path save/load:**
```typescript
const savePath = () => {
  const pathData = {
    name: "Test Path 1",
    waypoints: waypoints,
    speed: defaultSpeed,
    createdAt: new Date().toISOString()
  };

  const json = JSON.stringify(pathData, null, 2);
  const blob = new Blob([json], { type: 'application/json' });
  const url = URL.createObjectURL(blob);

  // Trigger download
  const a = document.createElement('a');
  a.href = url;
  a.download = 'robot-path.json';
  a.click();
};

const loadPath = (file: File) => {
  const reader = new FileReader();
  reader.onload = (e) => {
    const pathData = JSON.parse(e.target?.result as string);
    setWaypoints(pathData.waypoints);
  };
  reader.readAsText(file);
};
```

**3. Loop mode:**
```typescript
// Execute path repeatedly
const executeLoop = async (iterations: number) => {
  for (let i = 0; i < iterations; i++) {
    await executePath();
    if (!isExecuting) break;  // Check for stop
  }
};
```

### 4.5 Phase 4 Testing (Day 33)

**Path execution tests:**

```markdown
## Trajectory Planning Tests

### Test 1: Single Segment
1. Create 2 waypoints (A → B)
2. Execute path
✅ Smooth acceleration
✅ Constant velocity phase (if distance > threshold)
✅ Smooth deceleration
✅ Reaches target within 5mm

### Test 2: Multi-Segment Path
1. Create 5 waypoints
2. Execute path
✅ All joints finish each segment simultaneously
✅ Smooth transitions between waypoints
✅ No jerking or stuttering
✅ Execution time matches prediction (±10%)

### Test 3: Teach Mode
1. Manually move robot to 3 positions
2. Click "Teach" at each position
3. Execute taught path
✅ Waypoints captured accurately
✅ Robot revisits taught positions within 5mm

### Test 4: Path Visualization
1. Create path in UI
✅ Path line visible in 3D view
✅ Waypoint markers at correct positions
✅ Current target indicator updates during execution

### Test 5: Pause/Resume
1. Start path execution
2. Click pause mid-motion
3. Click resume
✅ Motion stops immediately
✅ Resumes from paused position
✅ No position loss

### Test 6: Emergency Stop
1. Execute long path
2. Click E-stop mid-motion
✅ Stops immediately
✅ Path execution aborted
✅ Can restart from beginning
```

### Phase 4 Deliverables

✅ **Motion Planning:**
- Trapezoidal velocity profiles
- Multi-joint coordination
- Path interpolation

✅ **Waypoint System:**
- Add/delete/reorder waypoints
- Teach mode
- Path save/load

✅ **Visualization:**
- Path preview in 3D
- Progress indicator
- Current target marker

**EXIT CRITERIA:**
- [ ] Can execute 10+ waypoint paths smoothly
- [ ] Position error at waypoints <5mm
- [ ] Motion smooth (no jerking)
- [ ] Execution time accurate (±10%)
- [ ] Pause/resume works reliably

---

## Phase 5: Drawing & G-code

**Duration**: Week 7-8 (7-10 days)
**Priority**: ⭐⭐
**Goal**: G-code support and drawing capabilities

### 5.1 G-code Parser (Day 34-35)

**File:** `src/utils/gcode.ts`

**Code source**: ROBOT_ARM_CODING_SPEC.md section 7.1

**Supported G-code commands:**
- **G0**: Rapid positioning (pen up)
- **G1**: Linear interpolation (pen down)
- **G2**: Clockwise circular arc
- **G3**: Counter-clockwise circular arc
- **M3**: Pen down (or activate tool)
- **M5**: Pen up (or deactivate tool)

**Implementation:**

```typescript
export class GCodeParser {
  // ... (use code from spec) ...

  // Additional: Validate G-code before execution
  validate(commands: GCodeCommand[]): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    commands.forEach((cmd, i) => {
      // Check for valid coordinates
      if ((cmd.type === 'G0' || cmd.type === 'G1') &&
          cmd.x === undefined && cmd.y === undefined && cmd.z === undefined) {
        errors.push(`Line ${i}: No coordinates specified`);
      }

      // Check for arc parameters
      if ((cmd.type === 'G2' || cmd.type === 'G3') &&
          (cmd.i === undefined || cmd.j === undefined)) {
        errors.push(`Line ${i}: Arc missing I/J parameters`);
      }

      // Check workspace bounds
      if (cmd.x !== undefined && Math.abs(cmd.x) > 0.3) {
        warnings.push(`Line ${i}: X coordinate may be out of reach`);
      }
    });

    return { valid: errors.length === 0, errors, warnings };
  }
}
```

**Testing G-code parser:**

```typescript
// Test G-code snippet
const testGcode = `
G0 X0 Y0 Z5           ; Move to start (pen up)
M3                    ; Pen down
G1 X10 Y0 Z0 F100     ; Line to (10,0)
G1 X10 Y10 Z0         ; Line to (10,10)
G1 X0 Y10 Z0          ; Line to (0,10)
G1 X0 Y0 Z0           ; Line to (0,0)
M5                    ; Pen up
`;

const parser = new GCodeParser();
const commands = parser.parse(testGcode);

console.log(`Parsed ${commands.length} commands`);
commands.forEach(cmd => console.log(cmd));

// Expected output:
// G0: {x:0, y:0, z:5}
// M: {m:3}
// G1: {x:10, y:0, z:0, f:100}
// ... etc
```

### 5.2 Coordinate Transformation (Day 35-36)

**Map G-code coordinates to robot workspace:**

```typescript
interface DrawingSurface {
  origin: { x: number; y: number; z: number };  // Robot coordinates
  rotation: number;  // Rotation around Z axis (degrees)
  scale: number;     // Scale factor (G-code units → meters)
}

class CoordinateTransformer {
  constructor(private surface: DrawingSurface) {}

  // Transform G-code coords to robot coords
  transform(gcodeX: number, gcodeY: number, gcodeZ: number): Vector3 {
    // Apply scale
    let x = gcodeX * this.surface.scale;
    let y = gcodeY * this.surface.scale;
    let z = gcodeZ * this.surface.scale;

    // Apply rotation
    const rad = this.surface.rotation * Math.PI / 180;
    const xRot = x * Math.cos(rad) - y * Math.sin(rad);
    const yRot = x * Math.sin(rad) + y * Math.cos(rad);

    // Apply translation
    return new Vector3(
      xRot + this.surface.origin.x,
      yRot + this.surface.origin.y,
      z + this.surface.origin.z
    );
  }
}
```

**Surface calibration wizard:**

```typescript
// 3-point or 4-point calibration
const calibrateSurface = async (): Promise<DrawingSurface> => {
  // 1. Move to 3 corners of drawing surface
  // 2. User confirms position at each corner
  // 3. Calculate surface plane and orientation

  const point1 = await teachPoint("Move to ORIGIN (0,0) and click OK");
  const point2 = await teachPoint("Move to X-axis point (100,0) and click OK");
  const point3 = await teachPoint("Move to Y-axis point (0,100) and click OK");

  // Calculate surface parameters
  const xAxis = point2.sub(point1).normalize();
  const yAxis = point3.sub(point1).normalize();
  const zAxis = xAxis.cross(yAxis).normalize();

  // Rotation = angle of X-axis
  const rotation = Math.atan2(xAxis.y, xAxis.x) * 180 / Math.PI;

  // Scale = actual distance / G-code distance
  const actualDist = point1.distanceTo(point2);
  const gcodeDist = 100;  // 100mm in G-code
  const scale = actualDist / gcodeDist;

  return {
    origin: point1,
    rotation,
    scale
  };
};
```

### 5.3 G-code Uploader UI (Day 36-37)

**File:** `src/components/GCodeUploader.tsx`

**Code source**: ROBOT_ARM_CODING_SPEC.md section 7.2

**Enhanced features:**

```typescript
const GCodeUploader: React.FC = () => {
  const [gcodeContent, setGcodeContent] = useState('');
  const [commands, setCommands] = useState<GCodeCommand[]>([]);
  const [waypoints, setWaypoints] = useState<CartesianPose[]>([]);
  const [surface, setSurface] = useState<DrawingSurface | null>(null);
  const [validation, setValidation] = useState<ValidationResult | null>(null);

  const handleFileUpload = (file: File) => {
    const reader = new FileReader();
    reader.onload = (e) => {
      const content = e.target?.result as string;
      setGcodeContent(content);

      // Parse G-code
      const parser = new GCodeParser();
      const cmds = parser.parse(content);
      setCommands(cmds);

      // Validate
      const val = parser.validate(cmds);
      setValidation(val);

      // If valid and surface calibrated, convert to waypoints
      if (val.valid && surface) {
        const transformer = new CoordinateTransformer(surface);
        const wps = parser.toWaypoints(cmds, transformer);
        setWaypoints(wps);
      }
    };
    reader.readAsText(file);
  };

  return (
    <div className="p-4">
      <h2>G-code Upload</h2>

      {/* Surface calibration */}
      {!surface && (
        <button onClick={() => calibrateSurface().then(setSurface)}>
          Calibrate Drawing Surface
        </button>
      )}

      {surface && (
        <div className="bg-green-100 p-2 mb-4">
          ✅ Surface calibrated: Origin ({surface.origin.x.toFixed(3)}, ...)
        </div>
      )}

      {/* File upload */}
      <input type="file" accept=".gcode,.nc,.txt" onChange={handleFileUpload} />

      {/* Validation results */}
      {validation && (
        <div className={validation.valid ? 'text-green-600' : 'text-red-600'}>
          {validation.valid ? '✅ Valid G-code' : '❌ Errors found'}
          {validation.errors.map(err => <div key={err}>{err}</div>)}
          {validation.warnings.map(warn => <div key={warn}>⚠️ {warn}</div>)}
        </div>
      )}

      {/* Preview */}
      {waypoints.length > 0 && (
        <div>
          <p>{waypoints.length} waypoints generated</p>
          <button onClick={executeGcode}>Execute G-code</button>
        </div>
      )}

      {/* G-code viewer */}
      <pre className="bg-gray-100 p-2 font-mono text-sm">
        {gcodeContent}
      </pre>
    </div>
  );
};
```

### 5.4 Pen Control (Day 37-38)

**Options for pen up/down:**

**Option 1: Use J6 rotation**
```typescript
// Rotate end-effector to lower/raise pen
const penDown = async () => {
  const currentAngles = useRobotStore.getState().currentAngles;
  await moveToTarget({
    ...currentAngles,
    J6: currentAngles.J6 + 90  // Rotate 90° to lower pen
  });
};

const penUp = async () => {
  const currentAngles = useRobotStore.getState().currentAngles;
  await moveToTarget({
    ...currentAngles,
    J6: currentAngles.J6 - 90  // Rotate back to raise pen
  });
};
```

**Option 2: Add servo to firmware**
```cpp
// Add to firmware (if using separate servo for pen)
#define PEN_SERVO_PIN 9

Servo penServo;

void setup() {
  penServo.attach(PEN_SERVO_PIN);
  penServo.write(90);  // Pen up
}

// Serial command: P 1 (down) or P 0 (up)
void handlePenCommand(String cmd) {
  int state = cmd.substring(2).toInt();
  penServo.write(state ? 60 : 90);  // Adjust angles as needed
}
```

**Option 3: Z-axis lift**
```typescript
// Move Z up/down by small amount
const penDown = async () => {
  const pose = getCurrentPose();
  pose.position.z -= 0.005;  // Lower 5mm
  const ik = solveIK(pose);
  await moveToTarget(ik.angles);
};
```

### 5.5 Drawing Tests (Day 38-39)

**Test drawings:**

1. **Square test** (basic accuracy)
   ```gcode
   G0 X0 Y0 Z5
   M3
   G1 X50 Y0 Z0 F100
   G1 X50 Y50
   G1 X0 Y50
   G1 X0 Y0
   M5
   ```
   ✅ Corners meet within 2mm
   ✅ Lines straight
   ✅ Pen up/down works

2. **Circle test** (arc interpolation)
   ```gcode
   G0 X25 Y0 Z5
   M3
   G2 X25 Y0 I-25 J0 Z0 F100
   M5
   ```
   ✅ Circle closes (start=end within 2mm)
   ✅ Smooth arc (no faceting visible)

3. **Complex shape** (combination)
   - Import SVG → G-code converter online
   - Test with simple logo or text
   ✅ Recognizable output
   ✅ No crashes or errors

### Phase 5 Deliverables

✅ **G-code Support:**
- Parser for G0, G1, G2, G3, M3, M5
- Coordinate transformation
- Arc interpolation

✅ **Drawing Features:**
- Surface calibration
- G-code upload and validation
- Preview in 3D
- Pen up/down control

✅ **Testing:**
- Accuracy tests (square, circle)
- Complex shape test
- Repeatability test

**EXIT CRITERIA:**
- [ ] Can parse standard G-code files
- [ ] Drawing accuracy ±2mm for 100mm shapes
- [ ] Pen up/down reliable (100% success rate)
- [ ] Surface calibration repeatable
- [ ] Can draw recognizable text/shapes

---

## Phase 6: Polish & Production

**Duration**: Week 8-9 (7-10 days)
**Priority**: ⭐⭐
**Goal**: Production-ready application

### 6.1 UI/UX Polish (Day 40-42)

**Improvements:**

1. **Layout refinement**
   - Collapsible panels
   - Responsive design (works on different screen sizes)
   - Dark mode toggle

2. **Error handling**
   ```typescript
   // Wrap all robot commands in try-catch
   const moveToTarget = async (angles: JointAngles) => {
     try {
       await serialManager.moveToAngles(angles, speed);
     } catch (error) {
       toast.error(`Movement failed: ${error.message}`);
       // Optionally: retry logic
     }
   };

   // Add toast notifications (npm install react-hot-toast)
   import toast, { Toaster } from 'react-hot-toast';

   <Toaster position="top-right" />
   ```

3. **Loading states**
   ```typescript
   const [isConnecting, setIsConnecting] = useState(false);
   const [isMoving, setIsMoving] = useState(false);

   // Show spinners during operations
   {isConnecting && <Spinner />}
   {isMoving && <LinearProgress />}
   ```

4. **Keyboard shortcuts**
   ```typescript
   useEffect(() => {
     const handleKeyPress = (e: KeyboardEvent) => {
       if (e.key === ' ') {
         e.preventDefault();
         emergencyStop();  // Spacebar = E-stop
       }

       if (e.ctrlKey && e.key === 'h') {
         e.preventDefault();
         homeAll();  // Ctrl+H = Home all
       }
     };

     window.addEventListener('keydown', handleKeyPress);
     return () => window.removeEventListener('keydown', handleKeyPress);
   }, []);
   ```

5. **Settings panel**
   - Save/load preferences (speeds, defaults)
   - Joint limit overrides
   - Safety settings

### 6.2 Documentation (Day 42-43)

**Create documentation files:**

1. **docs/USER_MANUAL.md**
   - Getting started guide
   - Connection instructions
   - Feature tutorials (manual control, paths, G-code)
   - Troubleshooting

2. **docs/CALIBRATION_GUIDE.md**
   - Homing procedure
   - DH parameter tuning
   - Surface calibration for drawing
   - Workspace mapping

3. **docs/SERIAL_PROTOCOL.md**
   - Complete command reference
   - Response formats
   - Example sequences
   - Error codes

4. **README.md**
   - Project overview
   - Quick start
   - Features list
   - Screenshots
   - License

### 6.3 Testing & Validation (Day 43-45)

**Comprehensive test suite:**

```markdown
## Final System Tests

### 1. Connection Tests
- [ ] Connect/disconnect 10 times
- [ ] No memory leaks
- [ ] Auto-reconnect after disconnect
- [ ] Multiple sessions (close tab, reopen)

### 2. Motion Tests
- [ ] 100 random movements (no crashes)
- [ ] All joints to full range
- [ ] Emergency stop at random times
- [ ] Soft limits respected

### 3. Homing Tests
- [ ] Home each joint 10 times
- [ ] Repeatability <0.5°
- [ ] Home all sequence works
- [ ] Recovery from failed homing

### 4. Kinematics Tests
- [ ] FK/IK accuracy (<5mm error)
- [ ] 50 random IK solutions
- [ ] Singularity detection
- [ ] Workspace boundary check

### 5. Path Tests
- [ ] 10-waypoint path execution
- [ ] Pause/resume mid-path
- [ ] Path save/load
- [ ] Loop execution (100 iterations)

### 6. G-code Tests
- [ ] Parse 10+ different G-code files
- [ ] Draw square (±2mm accuracy)
- [ ] Draw circle (±2mm accuracy)
- [ ] Complex shape (readable output)

### 7. Performance Tests
- [ ] 3D viewer 60 FPS
- [ ] UI responsive (<100ms)
- [ ] No lag during motion
- [ ] Memory stable (no leaks)

### 8. Stress Tests
- [ ] 8-hour continuous operation
- [ ] 1000+ movements
- [ ] Multiple E-stops
- [ ] Recovery from errors

### 9. Cross-Browser Tests
- [ ] Chrome (primary)
- [ ] Edge (should work)
- [ ] Firefox (may not work - no Web Serial)
- [ ] Safari (won't work - no Web Serial)

### 10. User Acceptance Tests
- [ ] New user can connect in <5 min
- [ ] Intuitive UI (minimal instructions needed)
- [ ] Helpful error messages
- [ ] No unexpected behavior
```

### 6.4 Performance Optimization (Day 45-46)

**Optimizations:**

1. **3D rendering**
   ```typescript
   // Use React.memo for expensive components
   const Viewer3D = React.memo(() => {
     // ... rendering code ...
   });

   // Throttle joint updates
   import { throttle } from 'lodash';

   const updateJoints = throttle((angles: JointAngles) => {
     // Update 3D model
   }, 50);  // Max 20Hz
   ```

2. **State updates**
   ```typescript
   // Batch state updates
   useRobotStore.setState({
     currentAngles: newAngles,
     robotState: RobotState.MOVING,
     endstopState: newEndstops
   });  // Single render instead of 3
   ```

3. **Serial communication**
   ```typescript
   // Buffer commands
   class CommandQueue {
     private queue: Command[] = [];

     async send(cmd: Command) {
       this.queue.push(cmd);
       if (this.queue.length > 10) {
         console.warn('Command queue backing up');
       }
       await this.processQueue();
     }
   }
   ```

### 6.5 Deployment (Day 46-47)

**Build for production:**

```bash
# Optimize build
npm run build

# Build stats
ls -lh build/

# Should be <5MB total
```

**Deployment options:**

**Option 1: Local deployment (recommended for now)**
```bash
# Serve locally
npm install -g serve
serve -s build -p 3000

# Access at http://localhost:3000
```

**Option 2: Static hosting (GitHub Pages, Netlify)**
```bash
# Deploy to GitHub Pages
npm install -g gh-pages

# Add to package.json:
"homepage": "https://yourusername.github.io/robot-arm-control",
"scripts": {
  "predeploy": "npm run build",
  "deploy": "gh-pages -d build"
}

npm run deploy
```

**Option 3: Electron desktop app**
```bash
# Install Electron
npm install electron electron-builder

# Create electron/main.js
const { app, BrowserWindow } = require('electron');

function createWindow() {
  const win = new BrowserWindow({
    width: 1200,
    height: 800,
    webPreferences: {
      nodeIntegration: true
    }
  });

  win.loadFile('build/index.html');
}

app.whenReady().then(createWindow);

# Build app
npm run electron-build
```

### Phase 6 Deliverables

✅ **Polish:**
- Refined UI/UX
- Error handling
- Loading states
- Keyboard shortcuts

✅ **Documentation:**
- User manual
- Calibration guide
- API documentation
- README

✅ **Testing:**
- Comprehensive test suite
- Performance benchmarks
- Cross-browser compatibility

✅ **Deployment:**
- Production build
- Deployment instructions
- Desktop app (optional)

**EXIT CRITERIA:**
- [ ] All tests passing
- [ ] Documentation complete
- [ ] Production build <5MB
- [ ] No critical bugs
- [ ] User manual tested with new user

---

## Risk Management

### Critical Risks & Mitigation

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| **URDF file not available** | Medium | HIGH | Create from scratch using CAD measurements (add 1-2 weeks) |
| **Web Serial API issues** | Low | HIGH | Provide Electron fallback, test on multiple systems |
| **IK convergence failures** | Medium | Medium | Implement robust numerical fallback, workspace limits |
| **Real-time motion timing** | Low | HIGH | All timing in firmware, web app sends high-level commands |
| **3D visualization performance** | Low | Medium | Use LOD meshes, optimize rendering, 30 FPS acceptable |
| **DH parameters incorrect** | High | HIGH | **Validation critical** - test with known positions |
| **Homing unreliable** | Medium | Medium | Add slow fine-approach, debouncing, retry logic |
| **Cross-browser compatibility** | Medium | Low | Chrome/Edge only, clear browser requirements |
| **Drawing accuracy poor** | Medium | Medium | Calibration wizard, manual offset adjustment |
| **Singularity handling** | Medium | Low | Warn user, prevent moves near singularities |

### Mitigation Strategies

**For missing URDF (highest risk):**
1. Check if URDF can be generated from CAD model
2. Use online URDF builder (link construction kit)
3. Manual creation using measurements
4. Simplified URDF (cylinders instead of detailed meshes)

**For IK failures:**
1. Always provide numerical IK fallback
2. Detect singularities (Jacobian determinant)
3. Limit workspace to verified reachable volume
4. Multiple solution ranking (prefer current configuration)

**For timing issues:**
1. All critical timing in Teensy firmware
2. Web app sends high-level goals, not real-time commands
3. Buffering in firmware (queue of 5-10 positions)
4. Web Serial has <10ms latency typically

---

## Quality Assurance

### Code Quality Standards

**TypeScript:**
- Strict mode enabled
- No `any` types (use `unknown` if needed)
- All functions documented with JSDoc
- Unit tests for kinematics (>80% coverage)

**C++:**
- Const-correctness
- RAII for resource management
- No dynamic allocation in ISRs
- Serial protocol fully documented

**React:**
- Functional components only
- Hooks for state management
- PropTypes or TypeScript interfaces
- Component documentation

### Testing Strategy

**Unit Tests:**
- FK/IK solvers (critical path)
- G-code parser
- Trajectory planner
- Coordinate transformations

**Integration Tests:**
- Firmware + Web app communication
- Full motion sequences
- Homing procedures
- Path execution

**Manual Tests:**
- Physical robot movement
- Drawing accuracy
- E-stop response
- User workflows

### Performance Benchmarks

| Metric | Target | Measurement |
|--------|--------|-------------|
| **Connection time** | <10s | Time from click to connected |
| **UI response** | <100ms | Click to visual feedback |
| **3D FPS** | >30 FPS | Constant during motion |
| **Position update** | <100ms | Firmware to UI display |
| **IK solve time** | <50ms | Single solution |
| **Path planning** | <1s per 10 wp | Offline computation |
| **Memory usage** | <500MB | Browser DevTools |
| **Build size** | <5MB | Optimized production build |

---

## Success Metrics

### Phase Completion Criteria

**Phase 1: Foundation**
- ✅ All 6 motors controllable
- ✅ Homing functional (J2-J5)
- ✅ Web app connects reliably
- ✅ Position feedback real-time

**Phase 2: Kinematics**
- ✅ FK error <5mm
- ✅ IK success >95%
- ✅ Cartesian control working
- ✅ Multiple solutions handled

**Phase 3: 3D Visualization**
- ✅ Robot model renders correctly
- ✅ Real-time joint updates
- ✅ 30+ FPS performance
- ✅ Camera controls intuitive

**Phase 4: Motion Control**
- ✅ Smooth trajectory execution
- ✅ Path planning working
- ✅ Waypoint teach mode
- ✅ Save/load paths

**Phase 5: Drawing**
- ✅ G-code parser working
- ✅ Drawing accuracy ±2mm
- ✅ Surface calibration
- ✅ Pen control reliable

**Phase 6: Production**
- ✅ All tests passing
- ✅ Documentation complete
- ✅ Production build working
- ✅ User acceptance achieved

### Final Acceptance Criteria

**Functional Requirements:**
- [ ] Can control all 6 joints manually with <1° error
- [ ] Can command Cartesian positions with <5mm error
- [ ] Can execute multi-waypoint paths smoothly
- [ ] Can parse and execute G-code files
- [ ] Can draw simple shapes with ±2mm accuracy
- [ ] Emergency stop responds in <100ms

**Performance Requirements:**
- [ ] Connection established in <10 seconds
- [ ] UI responds in <100ms
- [ ] 3D visualization runs at 30+ FPS
- [ ] Position updates every 100ms
- [ ] No crashes during 1-hour operation

**Quality Requirements:**
- [ ] Code documented (JSDoc, comments)
- [ ] User manual complete
- [ ] All critical features tested
- [ ] Error messages helpful
- [ ] No critical bugs

**Usability Requirements:**
- [ ] New user can connect in <5 minutes
- [ ] Intuitive controls (minimal training)
- [ ] Clear visual feedback
- [ ] Recoverable from errors
- [ ] Keyboard shortcuts available

---

## Appendix A: Creating URDF from Scratch

**If URDF file is not available, follow these steps:**

### A.1 Measure Robot Dimensions

**Required measurements:**
1. Base height (Z0 to J1 axis)
2. Upper arm length (J2 to J3 pivot)
3. Forearm length (J3 to J4 pivot)
4. Wrist lengths (J4-J5, J5-J6 distances)
5. Tool/end-effector length

**Tool: Caliper or ruler**

### A.2 Determine Joint Axes

**For each joint:**
- Rotation axis direction (X, Y, or Z in local frame)
- Joint type (revolute for all 6)
- Joint limits (from spec)

### A.3 Create Minimal URDF

```xml
<?xml version="1.0"?>
<robot name="6dof_arm">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint 1 -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.698" upper="0.524" effort="10" velocity="1"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Repeat for joints 2-6 -->

</robot>
```

### A.4 Validate URDF

```bash
# Install urdfpy (Python)
pip install urdfpy

# Check URDF
python -c "import urdfpy; robot = urdfpy.URDF.load('robot.urdf'); print(robot)"

# Visualize (optional)
pip install trimesh pyglet
python -c "import urdfpy; urdfpy.URDF.load('robot.urdf').show()"
```

### A.5 Create STL Meshes (Optional)

**Option 1: Use simple geometries**
- Cylinders for links
- Boxes for joints
- No STL files needed

**Option 2: Export from CAD**
- If CAD model available
- Export each part as STL
- Reference in URDF `<mesh filename="link1.stl"/>`

**Option 3: Skip for now**
- Use geometric primitives
- Add detailed meshes later

---

## Appendix B: Alternative Firmware Platforms

**If Teensy 4.1 is not available:**

### Option 1: Arduino Mega + RAMPS
- **Pros**: Cheap, common
- **Cons**: Slower (16 MHz), may struggle with 6 axes
- **Verdict**: Possible but not recommended

### Option 2: ESP32
- **Pros**: WiFi, dual-core, fast
- **Cons**: Web Serial over WiFi different implementation
- **Verdict**: Good alternative, needs code adaptation

### Option 3: Raspberry Pi Pico
- **Pros**: Cheap, fast (133 MHz), dual-core
- **Cons**: Different toolchain
- **Verdict**: Good alternative, needs minor changes

**Recommendation**: Stick with Teensy 4.1 if possible. If not available, ESP32 is best alternative.

---

## Appendix C: Bill of Materials

**Hardware:**
- [x] Teensy 4.1
- [x] 6× TMC2209 drivers
- [x] 2× CNC Shield V3
- [x] 6× Stepper motors (as per spec)
- [x] 4× Omron D2F-L endstops
- [x] 24V power supply (≥5A)
- [x] USB cable (Teensy to PC)
- [ ] Emergency stop button (NC contacts)
- [ ] Wiring (22-24 AWG)
- [ ] Connectors (Dupont, JST, etc.)

**Software:**
- [ ] Arduino IDE 2.x or PlatformIO
- [ ] Teensyduino
- [ ] Node.js 18+ LTS
- [ ] Chrome or Edge browser
- [ ] VS Code (recommended)

**Files (CRITICAL):**
- [ ] robot.urdf file
- [ ] STL mesh files (6+ files)

---

## Conclusion

This implementation plan provides a comprehensive, phase-by-phase approach to building a professional 6DOF robot arm control system. By following this plan:

- **Weeks 1-2**: Basic firmware and manual control
- **Weeks 3-4**: Kinematics (FK/IK)
- **Week 5**: 3D visualization
- **Weeks 6-7**: Path planning and trajectories
- **Weeks 7-8**: Drawing and G-code
- **Week 8-9**: Polish and production

**Key Success Factors:**
1. **Start with firmware** - get hardware working first
2. **Validate kinematics early** - DH parameters critical
3. **Test incrementally** - each phase has exit criteria
4. **Document as you go** - easier than retroactive docs
5. **User acceptance** - test with someone unfamiliar

**Critical Path Items:**
- URDF file availability (Week 3 blocker)
- DH parameter validation (Week 3-4)
- IK solver stability (Week 4)
- 3D visualization performance (Week 5)

**Good luck with implementation! 🤖**

---

**Document Version**: 1.0
**Last Updated**: February 9, 2026
**Author**: AI Assistant
**Status**: Ready for Implementation

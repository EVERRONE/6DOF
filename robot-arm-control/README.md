# 6DOF Robot Arm - Web Control Application

## Overview
A React + TypeScript web application for controlling a 6-axis robot arm via the Web Serial API.

## Features (Phase 1)
- ✅ Connect to robot via Web Serial API
- ✅ Manual control of all 6 joints with sliders
- ✅ Real-time position feedback
- ✅ Motor enable/disable
- ✅ Joint homing controls
- ✅ Emergency stop button
- ✅ Status monitoring (robot state, endstops)

## Browser Requirements
**Supported:**
- Chrome 89+
- Edge 89+

**Not Supported:**
- Firefox (no Web Serial API support)
- Safari (no Web Serial API support)

## Installation

### Prerequisites
- Node.js 18+ LTS
- Chrome or Edge browser
- Robot arm with Teensy firmware uploaded

### Setup
```bash
npm install
npm start
```

The application will open at `http://localhost:3000`

## Usage

### 1. Connect to Robot
1. Click **"Connect to Robot"** button
2. Select the Teensy COM port from the browser dialog
3. Wait for connection (status indicator turns green)

### 2. Enable Motors
- Check the **"Motors Enabled"** checkbox
- This sends `E 1` command to the firmware

### 3. Control Joints
- **Adjust sliders** to set target angles
- **Click "Move to Target"** to execute movement
- Current position is shown in gray
- Target position is shown in blue

### 4. Homing
- Click individual **"Home J2/J3/J4/J5"** buttons to home specific joints
- Click **"Home All"** to home all joints in sequence (J2→J3→J4→J5)
- Homing only works for joints with endstops (J2-J5)

### 5. Emergency Stop
- Large red **"E-STOP"** button (bottom right)
- Immediately halts all motion
- Sends `S` command to firmware

## Project Structure

```
src/
├── components/
│   ├── ConnectionPanel.tsx      # Connect/disconnect UI
│   ├── JointControlPanel.tsx    # Sliders and controls
│   ├── StatusBar.tsx            # Robot state and endstop indicators
│   └── EmergencyStop.tsx        # E-stop button
├── communication/
│   ├── SerialManager.ts         # Web Serial API wrapper
│   └── types.ts                 # Serial message types
├── store/
│   └── robotStore.ts            # Zustand state management
├── types/
│   └── robot.ts                 # Robot interfaces and enums
├── App.tsx                      # Main application layout
└── index.tsx                    # React entry point
```

## Troubleshooting

### "Web Serial API not supported"
- Use Chrome or Edge browser (version 89+)
- Web Serial requires HTTPS or localhost

### Connection Fails
- Verify Teensy is connected via USB
- Check firmware is uploaded and running
- Try different USB port
- Close other programs using the COM port (Arduino IDE, etc.)

### Position Updates Not Received
- Check Serial Monitor baud rate (115200)
- Verify firmware is sending position updates
- Check browser console for errors

### Sliders Don't Work
- Ensure "Motors Enabled" is checked
- Verify robot is connected (green status indicator)
- Check firmware responded to `E 1` command

## Technologies Used
- **React 18** - UI framework
- **TypeScript 5** - Type safety
- **Zustand** - State management
- **Tailwind CSS** - Styling
- **Web Serial API** - Hardware communication

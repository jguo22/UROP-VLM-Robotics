# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Unity-based simulation of two UR5 robotic arms controlled by Agentic AI for collaborative tasks like gearbox assembly. The system supports multiple control modes including manual control, inverse kinematics, automated pick-and-place, CSV trajectory playback, and synchronized multi-robot AI control.

**Key Technologies:**
- Unity 6000.2.2f1 (specific version required)
- C# scripts (78 total) for robot control and physics
- Python socket integration for external AI control
- Unity ArticulationBody physics system for realistic robot dynamics
- Custom UR5 inverse kinematics solver (ikUR5 library)

## Development Commands

### Unity Operations

**Open Project:**
```bash
# In Unity Hub, add project from disk and select ur5_simulation folder
# Verify editor version is 6000.2.2f1
```

**Run Simulation:**
- Open RobotArmScene (default, completed gearbox) or SampleScene (partial gearbox) in Assets/Scenes
- Press Play button in Unity Editor
- Use number keys 1-5 to switch control modes (single robot mode)
- Press 'P' to switch between robots in single robot mode

**Enable Agentic AI Mode:**
1. Select SceneSetup in Hierarchy
2. Enable "Agentic AI" checkbox in Inspector
3. Place trajectory files in `/AgenticAIPaths/` folder (create if needed):
   - `ur5_left.csv` - left robot trajectory
   - `ur5_right.csv` - right robot trajectory
4. Press F1 to load trajectories
5. Press Space to start synchronized playback

### Python Socket Integration

**Configure Socket Connection:**
```bash
# Edit socket.env in project root
HOST="127.0.0.1"
PORT=5000
```

**Run Python Client:**
```bash
cd python_socket_template
python agentic_socket.py
```

**Enable Socket in Unity:**
- Select SceneSetup in Hierarchy
- Enable "Python Socket" checkbox in Inspector
- Unity will start TCP server on launch

## Architecture Overview

### Dual-Mode Control System

**Single Robot Mode (Default):**
- User controls one robot at a time via `UnifiedRobotController`
- Switch robots with 'P' key
- 5 control modes accessible via number keys:
  1. **Manual** - Direct joint control with keyboard
  2. **IK** - End-effector position control
  3. **Pick and Place** - Automated sequences
  4. **Programmatic** - Script-based control
  5. **CSV Trajectory** - Playback recorded movements

**Agentic AI Mode (Multi-Robot):**
- Both robots operate simultaneously via `AgentRobotController`
- Synchronized trajectory playback from pre-computed CSV files
- Timestamp-based coordination ensures precise collaboration
- Managed by `SceneSetup` when `allRobotsActive` is enabled

### Key Components and Their Relationships

**Setup Layer (`Assets/Scripts/Setup/`):**
- `SceneSetup.cs` - Master orchestrator, manages robot switching and AI mode
- `RobotArmSetup.cs` - Configures individual robot physics (6 UR5 joints with ArticulationBody)
- `PythonSocketSetup.cs` - TCP server for Python AI integration (reads socket.env)

**Controller Layer (`Assets/Scripts/Controllers/`):**

For single robots:
- `UnifiedRobotController.cs` - Hub for single robot with 5 control modes
- `ManualController.cs` - Keyboard-based joint control
- `PickAndPlaceController.cs` - Waypoint-based automation with custom IK solver
- `CSVTrajectoryController.cs` - Single robot trajectory playback
- `SuctionController.cs` - Distance-based suction gripper (2.5cm threshold)

For multi-robot Agentic AI:
- `AgentRobotController.cs` - Coordinates multiple robots simultaneously
- `AgentTrajectoryController.cs` - Loads and synchronizes CSV trajectories for all robots
- `AgentTrajectory.cs` - Scans `/AgenticAIPaths/` folder and loads matching CSV files

**IK Solver Library (`Assets/Scripts/ikUR5/`):**
- Complete inverse kinematics implementation for UR5 6-DOF arm
- `Solver/Robot6ROffsetWirst.cs` - Main IK solver using Denavit-Hartenberg parameters
- `Kinematic/` - Forward kinematics, homogeneous transformations, frame management
- `Controller/` - Cartesian/joint target management and solution selection

### Data Flow: CSV Trajectory System

**Recording (Single Robot):**
```
User performs task → RobotArmSetup.cs exports CSV → Saved to /Exports/RobotJointCoordinates_[timestamp].csv
```

**AI Trajectory Generation (External):**
```
Recorded demos → External AI system → Optimized trajectories → /AgenticAIPaths/ur5_left.csv, ur5_right.csv
```

**Playback (Multi-Robot):**
```
F1 key press → AgentTrajectory scans folder → Loads CSVs → AgentTrajectoryController merges timestamps → Synchronized execution
```

**CSV Format:**
- Header: `Timestamp,base_PosX,base_PosY,base_PosZ,base_RotX,base_RotY,base_RotZ,base_RotW,[...joint data],suction_state,block_attached`
- Each row: timestamp, 6 joints × 7 values (position xyz + rotation xyzw), suction state, attachment state
- Timestamps must be in seconds for proper synchronization

### Critical File Locations

**Trajectory Files:**
- `/AgenticAIPaths/` - Agentic AI trajectories (ur5_left.csv, ur5_right.csv) - create this folder if needed
- `/Exports/` - Auto-exported recordings from Unity (RobotJointCoordinates_*.csv)
- `/Imports/` - Custom test trajectories with README and sample files

**Unity Assets:**
- `/Assets/Scenes/` - RobotArmScene (default), SampleScene, PlayScene
- `/Assets/Scripts/` - All C# code organized by layer
- `/Assets/URDF/` - Robot model definition (ur5.urdf)
- `/Assets/Prefabs/` - Reusable game objects

**Configuration:**
- `socket.env` - Python socket configuration (HOST, PORT)
- `/Assets/Scripts/Setup/ConstantsUR5.cs` - UR5 robot constants and parameters

**Logs:**
- `/Logs/` - Pick-and-place operation logs (when using PickAndPlaceController)

### Python-Unity Communication

**Current Implementation:**
- Unity runs TCP server (TcpListener) configured from socket.env
- Python client connects and sends/receives messages
- Echo protocol: Python sends → Unity echoes back
- Asynchronous message handling via BeginRead/ReceiveCallback

**Integration Points for AI:**
1. Real-time command sending from Python AI agents
2. State feedback (joint angles, positions) from Unity to Python
3. Vision-based control via OpenVLA integration point (UR5TCPServer.cs)
4. Reinforcement learning agent control via socket commands

### Control Mode Keyboard Shortcuts

**Mode Switching (Single Robot):**
- `1` - Manual mode
- `2` - IK mode
- `3` - Pick and Place mode
- `4` - Programmatic mode
- `5` - CSV Trajectory mode
- `P` - Switch between robots

**Manual Mode:**
- Arrow keys - Select joint (0-5)
- Up/Down or W/S - Rotate selected joint
- Space - Toggle suction
- R - Reset to home position

**IK Mode:**
- W/A/S/D - Move end-effector in X/Z plane
- Q/E - Move end-effector up/down (Y axis)
- Space - Toggle suction

**CSV Trajectory Mode:**
- F1 - Scan and load CSV files
- Space - Play/Pause
- S - Stop playback
- +/- - Adjust speed (0.25x to 4x)
- L - Toggle looping
- Left/Right arrows - Step through frames

**Agentic AI Mode:**
- F1 - Load all robot trajectories
- Space - Start/pause synchronized playback

### Important Unity Configuration

**Robot Setup (Required for each ur5_left/ur5_right):**
- URDF Robot [✓ Enabled]
- RobotArmSetup [✓ Enabled]
- SuctionController [✓ Enabled]
- UnifiedRobotController (single mode) OR AgentRobotController (AI mode)
- ManualController, PickAndPlaceController, CSVTrajectoryController [✗ Disabled initially]

**SceneSetup Configuration:**
- RobotArms[] - References to ur5_left and ur5_right
- Target Blocks[] - Array of pickable objects
- Agentic AI checkbox - Enable for multi-robot mode
- Python Socket checkbox - Enable for external AI control

**SuctionController Parameters:**
- Target Blocks[] - Must match blocks in scene
- Detection Distance - Default 0.025 (2.5cm)
- Attraction Force - Physics force applied when block in range
- Visual feedback: Green (attached) / Yellow (in range) / Red (too far)

### Multi-Robot Collaboration Workflow

The system is designed for collaborative tasks where both robots work together:

1. **Task Definition**: Create task requiring coordination (e.g., gearbox assembly)
2. **Manual Recording**: Record human demonstrations for each robot
3. **AI Processing**: External AI optimizes and generates synchronized trajectories
4. **Deployment**: Place ur5_left.csv and ur5_right.csv in /AgenticAIPaths/
5. **Execution**: Enable Agentic AI mode, press F1 to load, Space to execute
6. **Synchronization**: AgentTrajectoryController merges timestamps ensuring coordination

**Example Task - Gearbox Assembly:**
- ur5_left picks gear A from left side
- ur5_right picks gear B from right side
- Both robots move to center workspace
- Robots position gears for assembly
- Coordinated release and placement
- All movements synchronized via merged timeline

## Development Notes

### Working with Robot Control Scripts

When modifying controller scripts, understand the control hierarchy:
- `SceneSetup` determines single vs. multi-robot mode
- Single mode: `UnifiedRobotController` manages one robot with sub-controllers
- Multi mode: `AgentRobotController` manages all robots simultaneously
- All controllers ultimately set joint angles on ArticulationBody components configured by `RobotArmSetup`

### IK Solver Usage

The ikUR5 library provides complete forward/inverse kinematics:
- Use `Controller.cs` for high-level IK operations
- `Robot6ROffsetWirst.cs` handles analytical IK solution
- Multiple solutions possible - system selects closest to current configuration
- Joint limits and singularities handled automatically

### CSV Export and Recording

`RobotArmSetup.cs` automatically exports during playback:
- Press 'E' key to start/stop export
- Exports every frame: joint angles, positions, rotations, suction state, block attachment
- Files saved to /Exports/ with timestamp
- Use for creating training data or debugging trajectories

### Physics Configuration

ArticulationBody settings in `RobotArmSetup.cs`:
- Joint friction: 0.05
- Joint damping: 100
- Angular damping: 0.5
- Force limit: 1000
- Modify these if robot behavior seems unstable or too sluggish

### Adding New Control Modes

To add a new control mode to UnifiedRobotController:
1. Create new controller script (e.g., `MyNewController.cs`)
2. Add controller reference to `UnifiedRobotController.cs`
3. Add new enum value to `ControlMode`
4. Implement mode switching in `Update()` method
5. Add keyboard shortcut for mode activation

### External AI Integration

Current socket implementation is echo protocol - extend for real control:
1. Define command protocol (JSON or custom format)
2. Modify `PythonSocketSetup.cs` ReceiveCallback to parse commands
3. Call appropriate robot controller methods based on commands
4. Send state feedback back to Python client
5. Example commands: `{"robot": "ur5_left", "joint": 2, "angle": 45.0}`

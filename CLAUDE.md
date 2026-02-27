# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Agentic AI Digital Twin - Unity simulation of dual UR5 robotic arms controlled by OpenAI-powered AI agents via socket communication. Enables autonomous pick-and-place with real-time inverse kinematics.

## Requirements

- **Unity**: 6000.2.2f1 (exact version required)
- **Platform**: Windows
- **Python**: 3.x

## Commands

```bash
# Python setup
cd python_socket
pip install -r requirements.txt

# Set OpenAI API key (or use config.env)
export OPENAI_API_KEY="your-key"

# Run agent - sequential arm movement
python python_socket/agentic_socket.py /one

# Run agent - simultaneous dual-arm movement
python python_socket/agentic_socket.py /simul
```

## Architecture

### Communication Flow
```
Python Agent ←→ Socket (127.0.0.1:65432) ←→ Unity

1. Python sends get_scene_state → Unity returns robot/object positions
2. OpenAI generates JSON command list from scene state
3. For each command:
   - Python sends command → Unity responds with "run_ik" or "run_simul_ik"
   - Python IK solver computes joint angles → sends back to Unity
   - Unity executes movement
```

### Python Components (`python_socket/`)
- **agentic_socket.py**: Main loop - socket connection, command dispatch, IK solving
- **agent.py**: OpenAI wrapper, injects scene state into prompts
- **ik_solver.py**: UR5 IK using roboticstoolbox, Unity↔ROS coordinate transforms
- **agentic_setup.py**: AI prompts (TEST_PROMPT, SIMULTAENOUS_PROMPT), gear positions

### Unity Components (`ur5_simulation/Assets/Scripts/`)
- **Setup/SceneSetup.cs**: Master controller, checkbox-driven mode selection
- **Setup/AgenticSocketSetup.cs**: Sequential socket handler
- **Setup/SimulAgenticSocketSetup.cs**: Simultaneous dual-arm socket handler
- **Setup/PythonSocketSetup.cs**: JSON data structures for socket protocol
- **Controllers/AgentRobotController.cs**: Executes joint angles, suction control
- **Controllers/PoseAndImageRecorder.cs**: RLDS-compatible dataset recorder (joint state, delta EE pose, suction, episode flags, camera screenshots at adjustable FPS)

### JSON Protocol

Single-arm command:
```json
{"type": "execute_action", "action_type": "move_robot", "robot_name": "ur5_left", "parameters": {"target_position": [x, y, z]}}
```

Simultaneous command:
```json
{"ur5_left": {"action_type": "move_robot", "parameters": {"target_position": [x, y, z]}}, "ur5_right": {"action_type": "stationary"}}
```

Action types: `move_robot`, `activate_suction`, `deactivate_suction`, `home_robot`, `stationary`

### Coordinate Transform
- Unity: X-right, Y-up, Z-forward (left-handed)
- ROS: X-forward, Y-left, Z-up (right-handed)
- Transform in `ik_solver.py`: `[Z_unity, -X_unity, Y_unity]`

## Unity Scene Setup

On SceneSetup GameObject, enable ONE of:
- **allRobotsActive + agenticSocket**: Sequential control via Python
- **allRobotsActive + simulAgenticSocket**: Simultaneous dual-arm control
- **ikSocket** (without allRobotsActive): Direct IK mode

Scenes: `Assets/Scenes/RobotArmScene.unity` (complete gearbox), `SampleScene.unity` (partial)

## CSV Trajectory Files

Paths (relative to `ur5_simulation/`):
- **Exports/**: Auto-recorded trajectories from simulation
- **Exports/dataset/**: RLDS dataset episodes recorded by `PoseAndImageRecorder`
- **Imports/**: Custom/manual trajectory files

File filtering logic: `Assets/Scripts/Controllers/CSVTrajectoryExample.cs` → `PopulateCSVFiles()` (line ~66)

## Dataset Recording (RLDS)

`PoseAndImageRecorder` records episodes for robot learning. Attach to any GameObject and assign in Inspector:
- `endEffectorTransform`: end effector bone
- `recordingCamera`: camera for screenshots
- `suctionController`: robot's SuctionController
- `robotArmSetup`: robot's RobotArmSetup (for joint angles)

Each episode saved to `Exports/dataset/<sessionName>_<timestamp>/`:
- `poses.csv`: frame, timestamp, joint_0..5 (rad), delta_pos_xyz, delta_rot_xyzw, suction_on, is_first, is_last, is_terminal, reward, discount
- `images/000000.png` ...: camera screenshot per frame

Call `StartRecording()` / `StopRecording()` repeatedly — render resources persist across episodes.
`language_instruction` not recorded; add externally before training.

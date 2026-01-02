# UR5 IK Server Setup and Usage

This directory contains a Python-based inverse kinematics (IK) server for the UR5 robot that integrates with the Unity simulation via TCP sockets. The server uses the `roboticstoolbox-python` library to perform accurate IK calculations.

## Overview

The system consists of two components:
1. **Python IK Server** (`ur5_ik_server.py`) - Runs the IK computations using roboticstoolbox
2. **Unity C# Client** (`UR5IKSolver.cs`) - Sends IK requests from Unity and receives solutions

## Architecture

```
Unity (C#)                           Python Server
┌──────────────┐                     ┌──────────────┐
│ UR5IKSolver  │ ──TCP Socket──────> │ UR5IKServer  │
│              │   (Port 5010)       │              │
│ - SolveIK    │ <────────────────── │ - rtb.ik_LM  │
│ - SolveIKDelta│                    │ - UR5 Model  │
└──────────────┘                     └──────────────┘
```

### Coordinate System Conversion

The system automatically handles coordinate system differences:

- **Unity**: Left-handed, Y-up coordinate system
  - X: Right
  - Y: Up
  - Z: Forward

- **ROS/RTB**: Right-handed, Z-up coordinate system
  - X: Forward
  - Y: Left
  - Z: Up

Conversion (Unity → ROS): `[X_ros, Y_ros, Z_ros] = [Z_unity, -X_unity, Y_unity]`

## Installation

### 1. Install Python Dependencies

```bash
cd /Users/testing2/Documents/UROP-VLM-Robotics/PythonClient

# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install required packages
pip install roboticstoolbox-python spatialmath-python numpy
```

### 2. Verify UR5 URDF File

Ensure `ur5.urdf` exists in the PythonClient directory. This file is required for the robot model.

```bash
ls ur5.urdf
```

## Running the IK Server

### Start the Server

```bash
cd /Users/testing2/Documents/UROP-VLM-Robotics/PythonClient
source venv/bin/activate  # If using virtual environment
python ur5_ik_server.py
```

**Default settings:**
- Host: `127.0.0.1`
- Port: `5010`

### Custom Host/Port

```bash
python ur5_ik_server.py --host 0.0.0.0 --port 5015
```

### Expected Output

```
Loaded UR5 robot model from: /path/to/ur5.urdf
URRobot: UR5 (6 joints, 6 links)
UR5 IK Server listening on 127.0.0.1:5010
Waiting for Unity client connections...
```

## Unity Integration

### 1. Configure UR5IKSolver Component

In Unity, select the GameObject with the `UR5IKSolver` component and configure:

**Settings in Inspector:**
- **Server Host**: `127.0.0.1` (or IP of Python server)
- **Server Port**: `5010` (must match Python server port)
- **Auto Reconnect**: ✓ Enabled (recommended)
- **Connection Timeout**: `5000` ms
- **Debug Mode**: Enable for detailed logging

### 2. Using SolveIK Method

Solve IK for a specific target pose:

```csharp
// Get reference to IK solver
UR5IKSolver ikSolver = GetComponent<UR5IKSolver>();

// Define target pose
Vector3 targetPosition = new Vector3(0.5f, 0.3f, 0.2f);
Quaternion targetRotation = Quaternion.Euler(0, 90, 0);

// Get current joint angles
float[] currentAngles = new float[6] { 0, -90, 90, 0, 90, 0 };

// Solve IK
float[] solution = ikSolver.SolveIK(targetPosition, targetRotation, currentAngles);

if (solution != null)
{
    Debug.Log("IK solution found!");
    // Apply solution to robot joints
    for (int i = 0; i < 6; i++)
    {
        robotJoints[i].targetAngle = solution[i] * Mathf.Rad2Deg;
    }
}
else
{
    Debug.LogWarning("No IK solution found for target pose");
}
```

### 3. Using SolveIKDelta Method

Apply delta movements to current pose:

```csharp
// Get current end-effector pose
Vector3 currentPos = endEffector.position;
Quaternion currentRot = endEffector.rotation;

// Define delta action: [dx, dy, dz, droll, dpitch, dyaw, gripper]
float[] deltaAction = new float[7]
{
    0.1f,  // Move 10cm in X
    0.0f,  // No Y movement
    0.05f, // Move 5cm in Z
    0.0f,  // No roll change
    0.0f,  // No pitch change
    0.0f,  // No yaw change
    1.0f   // Gripper (not used in IK)
};

// Solve IK for delta
float[] solution = ikSolver.SolveIKDelta(currentPos, currentRot, deltaAction, currentAngles);

if (solution != null)
{
    // Apply solution
}
```

## Protocol Specification

### Binary Communication Protocol

All values are transmitted as little-endian doubles (8 bytes each).

#### Command 1: SolveIK

**Request (105 bytes):**
```
Byte 0:       Command type (1)
Bytes 1-24:   Target position (3 doubles: x, y, z)
Bytes 25-56:  Target rotation (4 doubles: quat x, y, z, w)
Bytes 57-104: Current joint angles (6 doubles: j0-j5)
```

**Response:**
```
Byte 0:       Success flag (1=success, 0=failure)
Bytes 1-48:   Joint angles solution (6 doubles, only if success=1)
```

#### Command 2: SolveIKDelta

**Request (161 bytes):**
```
Byte 0:        Command type (2)
Bytes 1-24:    Current position (3 doubles: x, y, z)
Bytes 25-56:   Current rotation (4 doubles: quat x, y, z, w)
Bytes 57-112:  Delta action (7 doubles: dx, dy, dz, dr, dp, dy, gripper)
Bytes 113-160: Current joint angles (6 doubles: j0-j5)
```

**Response:**
```
Byte 0:       Success flag (1=success, 0=failure)
Bytes 1-48:   Joint angles solution (6 doubles, only if success=1)
```

## Troubleshooting

### Connection Issues

**Problem**: Unity can't connect to Python server

**Solutions**:
1. Ensure Python server is running before starting Unity
2. Check firewall settings (allow port 5010)
3. Verify host/port match in both Unity and Python
4. Try `127.0.0.1` instead of `localhost`

### IK Solution Failures

**Problem**: Server returns no solution

**Possible causes**:
1. Target pose is out of robot's workspace
2. Target pose is in a singularity
3. Target orientation is not achievable
4. Joint limits exceeded

**Solutions**:
- Check target pose is within UR5 reach (~850mm)
- Provide better initial guess (current angles closer to solution)
- Relax orientation constraints if only position matters
- Validate target pose with forward kinematics first

### Performance Issues

**Problem**: IK solving is slow

**Solutions**:
- IK solving typically takes 10-50ms per request
- Don't call IK every frame - use interpolation between solutions
- Consider caching solutions for common poses
- Use delta IK for small movements (faster convergence)

### Coordinate System Issues

**Problem**: Robot moves to wrong positions

**Solutions**:
- Verify coordinate system conversion is working
- Use `ur5_fk_validator.py` to validate forward kinematics
- Check Unity and ROS coordinate frames match expected behavior
- Enable debug mode to see transmitted values

## Example Workflow

### 1. Start Python Server
```bash
cd PythonClient
source venv/bin/activate
python ur5_ik_server.py
```

### 2. Launch Unity Simulation
- Open Unity project
- Open scene with UR5 robots
- Ensure UR5IKSolver component is configured
- Press Play

### 3. Verify Connection
Check Unity Console for:
```
UR5IKSolver: Connecting to Python IK server at 127.0.0.1:5010...
UR5IKSolver: Connected to Python IK server successfully
```

### 4. Test IK Solving
Enable Debug Mode in Unity Inspector and move the robot. You should see:
```
UR5IKSolver.SolveIK: Sending request - pos=(0.5, 0.3, 0.2), rot=(0.0, 0.7, 0.0, 0.7)
UR5IKSolver.SolveIK: Solution found - joints=0.234, -1.234, 1.567, -0.876, 1.234, 0.456
```

## Testing and Validation

### Forward Kinematics Validation

Use `ur5_fk_validator.py` to verify the robot model is correct:

```bash
python ur5_fk_validator.py --host 127.0.0.1 --port 5005
```

This connects to Unity's FK server and validates calculated vs. reported positions.

### Unit Testing IK Server

Test the IK server directly in Python:

```python
from ur5_ik_server import UR5IKServer
import numpy as np

server = UR5IKServer()

# Test SolveIK
target_pos = np.array([0.5, 0.0, 0.3])
target_rot = np.array([0, 0, 0, 1])  # No rotation
current_angles = np.zeros(6)

solution = server.solve_ik(target_pos, target_rot, current_angles)
print(f"Solution: {solution}")
```

## Advanced Configuration

### Multiple Robot Support

To control multiple UR5 robots, run multiple IK servers on different ports:

```bash
# Terminal 1 - Left robot
python ur5_ik_server.py --port 5010

# Terminal 2 - Right robot
python ur5_ik_server.py --port 5011
```

Configure each Unity robot's UR5IKSolver component with the corresponding port.

### Custom IK Solvers

The Python server uses Levenberg-Marquardt (LM) method by default. You can modify `ur5_ik_server.py` to use different solvers:

```python
# In solve_ik method:
# solution = self.ur5.ik_LM(T_target, q0=current_angles)  # Default
solution = self.ur5.ik_NR(T_target, q0=current_angles)     # Newton-Raphson
solution = self.ur5.ikine_LMS(T_target, q0=current_angles) # LM Sugihara
```

## Files Reference

- `ur5_ik_server.py` - Python IK server implementation
- `ur5_fk_validator.py` - Forward kinematics validation tool
- `ur5.urdf` - UR5 robot URDF model
- `UR5IKSolver.cs` - Unity C# IK client (in Unity project)
- `IK_SERVER_README.md` - This documentation

## Support and Development

For issues or questions:
1. Check Unity Console for error messages
2. Check Python server terminal for server-side errors
3. Enable debug mode for detailed logging
4. Validate coordinate systems with FK validator

## License

This implementation uses:
- roboticstoolbox-python: https://github.com/petercorke/robotics-toolbox-python
- spatialmath-python: https://github.com/petercorke/spatialmath-python

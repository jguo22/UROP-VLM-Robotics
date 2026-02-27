# UR5 Inverse Kinematics Server

Python-based inverse kinematics (IK) and forward kinematics (FK) server for the UR5 robot arm, integrated with Unity simulation via TCP sockets. Uses `roboticstoolbox-python` for accurate kinematics calculations.

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Protocol Specification](#protocol-specification)
- [Coordinate Systems](#coordinate-systems)
- [Quaternion Representation](#quaternion-representation)
- [File Structure](#file-structure)
- [Usage Examples](#usage-examples)
- [Validation and Testing](#validation-and-testing)
- [Troubleshooting](#troubleshooting)
- [Advanced Configuration](#advanced-configuration)

## Overview

The system provides two main services:

1. **IK Server** (`main.py`) - Solves inverse kinematics for target poses
2. **FK/IK Validator** (`validator.py`) - Validates forward and inverse kinematics accuracy

Both services handle coordinate system conversion between Unity (left-handed, Y-up) and ROS (right-handed, Z-up) automatically.

## Architecture

```
Unity (C#)                           Python Server
┌──────────────┐                     ┌──────────────────┐
│ UR5IKSolver  │ ──TCP (5010)──────> │ UR5IKServer      │
│              │                     │ (main.py)        │
│ - SolveIK    │ <────────────────── │ - rtb.ik_LM      │
└──────────────┘                     │ - Coord Transform│
                                     └──────────────────┘

┌──────────────┐                     ┌──────────────────┐
│ Unity FK     │ ──TCP (5005)──────> │ UR5FKValidator   │
│ Server       │                     │ (validator.py)   │
│              │ <────────────────── │ - FK/IK Testing  │
└──────────────┘                     └──────────────────┘
```

### Key Features

- **Automatic Coordinate Conversion**: Unity ↔ ROS coordinate systems
- **Quaternion Standardization**: Internal w,x,y,z format (except Unity inputs)
- **High-Precision IK**: Levenberg-Marquardt solver with 1e-10 tolerance
- **Real-time Validation**: FK and IK round-trip testing
- **Binary Protocol**: Efficient 104-byte message format

## Installation

### 1. Install Python Dependencies

```bash
cd python/ik_server

# Create virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install required packages
pip install roboticstoolbox-python spatialmath-python numpy
```

### 2. Verify URDF File

Ensure `ur5.urdf` exists in the `ik_server` directory:

```bash
ls ur5.urdf
```

## Quick Start

### Running the IK Server

```bash
cd python/ik_server
python main.py
```

**Default settings:**
- Host: `127.0.0.1`
- Port: `5010`

**Custom configuration:**
```bash
python main.py --host 0.0.0.0 --port 5015
```

**Expected output:**
```
Loaded UR5 robot model from: /path/to/ur5.urdf
URRobot: UR5 (6 joints, 6 links)
UR5 IK Server listening on 127.0.0.1:5010
Waiting for Unity client connections...
```

### Running the FK/IK Validator

```bash
python validator.py
```

**Optional arguments:**
```bash
python validator.py --host 127.0.0.1 --port 5005 \
                    --tolerance 0.01 \
                    --rotation-tolerance 0.05 \
                    --joint-tolerance 0.01
```

**What it validates:**
1. **Forward Kinematics**: Joint angles → End effector pose
2. **Rotation Accuracy**: Quaternion orientation matching
3. **Inverse Kinematics**: Pose → Joint angles (round-trip test)

## Protocol Specification

### IK Server Protocol (Port 5010)

All values transmitted as **little-endian doubles** (8 bytes each).

#### Request Format (104 bytes)

```
Bytes 0-23:   Target position (3 doubles: x, y, z) in Unity coordinates
Bytes 24-55:  Target rotation (4 doubles: x, y, z, w) quaternion in Unity
Bytes 56-103: Current joint angles (6 doubles: radians)
```

#### Response Format

**Success (49 bytes):**
```
Byte 0:      Success flag (0x01)
Bytes 1-48:  Joint angles solution (6 doubles: radians)
```

**Failure (1 byte):**
```
Byte 0:      Failure flag (0x00)
```

### FK Validator Protocol (Port 5005)

Unity sends robot state for validation (104 bytes):

```
Bytes 0-23:   End effector position (3 doubles: x, y, z) in Unity
Bytes 24-55:  End effector rotation (4 doubles: x, y, z, w) in Unity
Bytes 56-103: Current joint angles (6 doubles: radians)
```

## Coordinate Systems

### Unity (Left-handed)

- **X-axis**: Right
- **Y-axis**: Up
- **Z-axis**: Forward

### ROS/Robotics Toolbox (Right-handed)

- **X-axis**: Forward
- **Y-axis**: Left
- **Z-axis**: Up

### Transformation Formulas

**Position: Unity → ROS**
```python
[x_ros, y_ros, z_ros] = [z_unity, -x_unity, y_unity]
```

**Position: ROS → Unity**
```python
[x_unity, y_unity, z_unity] = [-y_ros, z_ros, x_ros]
```

**Quaternion: Unity [x,y,z,w] → ROS [w,x,y,z]**
```python
[w, x, y, z]_ros = [w, -z, x, -y]  # where [x,y,z,w] from Unity
```

**Quaternion: ROS [w,x,y,z] → Unity [x,y,z,w]**
```python
[x, y, z, w]_unity = [y, -z, -x, w]  # where [w,x,y,z] from ROS
```

All transformations are handled automatically by `coordinate_transforms.py`.

## Quaternion Representation

**Important:** The codebase uses **[w, x, y, z]** format for all internal quaternion operations, except when receiving input from Unity.

### Format Convention

- **Unity Input/Output**: `[x, y, z, w]` (Unity standard)
- **Internal Processing**: `[w, x, y, z]` (mathematical standard)
- **ROS Coordinates**: `[w, x, y, z]` (ROS standard)

### Key Functions

```python
# Convert quaternion array to UnitQuaternion object
def arrayToQuaternion(arr: np.ndarray) -> UnitQuaternion:
    """Expects arr in [w, x, y, z] format"""
    return UnitQuaternion(arr[0], arr[1:4])

# Convert Unity quaternion to ROS quaternion
def unity_to_ros_quaternion(q_unity: np.ndarray) -> np.ndarray:
    """
    Input:  [x, y, z, w] (Unity format)
    Output: [w, x, y, z] (ROS format)
    """
    x, y, z, w = q_unity
    return np.array([w, -z, x, -y])

# Calculate angle between quaternions
def quaternion_angle_between(q1, q2) -> float:
    """
    Both inputs should be [w, x, y, z] or UnitQuaternion
    Returns: Angular difference in radians
    """
```

## File Structure

```
ik_server/
├── README.md                    # This file (combined documentation)
├── main.py                      # IK server implementation
├── validator.py                 # FK/IK validation tool
├── coordinate_transforms.py     # Unity ↔ ROS transformations
├── constants.py                 # Protocol constants and configurations
├── ur5.urdf                     # UR5 robot URDF model
└── __init__.py                  # Package initialization
```

### File Descriptions

**`main.py`** - UR5IKServer class
- TCP server on port 5010
- Receives IK requests from Unity
- Uses Levenberg-Marquardt IK solver
- Handles coordinate system conversion
- Returns joint angle solutions

**`validator.py`** - UR5FKValidator class
- TCP client connecting to Unity FK server (port 5005)
- Validates forward kinematics accuracy
- Tests inverse kinematics round-trip
- Measures position, rotation, and joint angle errors
- Displays real-time validation results

**`coordinate_transforms.py`** - Coordinate system utilities
- `unity_to_ros_position()` / `ros_to_unity_position()`
- `unity_to_ros_quaternion()` / `ros_to_unity_quaternion()`
- `arrayToQuaternion()` - Convert array to UnitQuaternion
- `quaternion_angle_between()` - Angular distance calculation
- `validate_position_transform()` - Round-trip position test
- `validate_quaternion_transform()` - Round-trip quaternion test

**`constants.py`** - Configuration constants
- Network settings (hosts, ports)
- Protocol message types and sizes
- Robot configuration (DOF, joint count)
- Tolerance values for validation
- File paths for URDF and profiles

## Usage Examples

### Python: Direct IK Solving

```python
from main import UR5IKServer
import numpy as np

# Create server instance
server = UR5IKServer()

# Define target (in Unity coordinates)
target_pos = np.array([0.5, 0.3, 0.2])  # Unity: x, y, z
target_rot = np.array([0, 0, 0, 1])     # Unity: x, y, z, w (identity)
current_angles = np.zeros(6)

# Solve IK (automatic coordinate conversion)
solution = server.solve_ik(target_pos, target_rot, current_angles)

if solution is not None:
    print(f"Solution found: {solution}")
else:
    print("No IK solution exists for target pose")
```

### Unity C#: Using IK Solver

```csharp
// Get reference to IK solver
UR5IKSolver ikSolver = GetComponent<UR5IKSolver>();

// Define target pose in Unity world coordinates
Vector3 targetPosition = new Vector3(0.5f, 0.3f, 0.2f);
Quaternion targetRotation = Quaternion.Euler(0, 90, 0);

// Get current joint angles
float[] currentAngles = GetJointAngles();

// Solve IK
float[] solution = ikSolver.SolveIK(targetPosition, targetRotation, currentAngles);

if (solution != null)
{
    // Apply solution to robot joints
    SetJointAngles(solution);
}
```

### Python: Testing Coordinate Transforms

```python
from coordinate_transforms import (
    unity_to_ros_position,
    unity_to_ros_quaternion,
    validate_position_transform,
    validate_quaternion_transform
)
import numpy as np

# Test position transformation (round-trip)
unity_pos = np.array([1.0, 2.0, 3.0])
assert validate_position_transform(unity_pos), "Position transform failed!"

# Test quaternion transformation (round-trip)
unity_quat = np.array([0.0, 0.707, 0.0, 0.707])  # Unity: x,y,z,w
assert validate_quaternion_transform(unity_quat), "Quaternion transform failed!"

print("All transformations validated successfully!")
```

## Validation and Testing

### Running Validation

1. **Start Unity FK Server**: Play Unity scene with FK validation enabled
2. **Run Python Validator**: `python validator.py`
3. **Observe Results**: Real-time validation output in terminal

### Validation Output Example

```
Iteration 1:
Original Joint Angles (rad): ['0.0000', '0.0000', '0.0000', '0.0000', '0.0000', '0.0000']

Forward Kinematics - Position Validation:
  Unity EE:        [0.500000, 0.300000, 0.200000]
  Transformed EE:  [0.200000, -0.500000, 0.300000]
  Calculated EE:   [0.200001, -0.500001, 0.300000]
  Error vector:    [0.000001, -0.000001, 0.000000]
  Error magnitude: 0.000001m
  Status: ✓ VALID

Forward Kinematics - Rotation Validation:
  Unity Quat [x,y,z,w]:       [0.000000, 0.707107, 0.000000, 0.707107]
  Transformed Quat [w,x,y,z]: [0.707107, 0.000000, 0.000000, -0.707107]
  Calculated Quat [w,x,y,z]:  [0.707107, 0.000000, 0.000000, -0.707107]
  Angular error: 0.000000 rad (0.00°)
  Status: ✓ VALID

Inverse Kinematics - Round-trip Validation:
  IK Joint Angles (rad): ['0.0000', '0.0001', '-0.0001', '0.0000', '0.0000', '0.0000']
  Joint Errors (rad):    ['0.000000', '0.000100', '-0.000100', '0.000000', '0.000000', '0.000000']
  Max joint error: 0.000100 rad (0.01°)
  IK iterations: 5
  Status: ✓ VALID

Overall: ✓ ALL VALID
```

### Tolerance Settings

Default validation tolerances:

- **Position**: 0.01 m (10 mm)
- **Rotation**: 0.05 rad (~2.87°)
- **Joint Angles**: 0.01 rad (~0.57°)
- **IK Solver**: 1e-10 (very strict convergence)

Adjust with command-line arguments:
```bash
python validator.py --tolerance 0.001 --rotation-tolerance 0.01 --joint-tolerance 0.005
```

## Troubleshooting

### Connection Issues

**Problem**: Unity can't connect to Python server

**Solutions**:
1. Ensure Python server is running **before** starting Unity
2. Check firewall settings (allow ports 5010 and 5005)
3. Verify host/port match in both Unity and Python
4. Try `127.0.0.1` instead of `localhost`
5. Check if port is already in use: `lsof -i :5010` (macOS/Linux)

### IK Solution Failures

**Problem**: Server returns no solution (failure byte 0x00)

**Possible Causes**:
1. Target pose is outside robot's workspace (~850mm reach)
2. Target orientation is physically impossible
3. Configuration is in/near a singularity
4. Joint limits would be exceeded

**Solutions**:
- Verify target is within UR5 reach envelope
- Use FK validator to test if joint angles produce expected pose
- Provide better initial guess (current angles closer to solution)
- Check for coordinate system errors (wrong Unity/ROS conversion)
- Enable debug logging to see exact values being sent

### Coordinate System Issues

**Problem**: Robot moves to unexpected positions

**Solutions**:
1. Verify Unity coordinate frame matches expectations (Y-up, left-handed)
2. Run `validate_position_transform()` and `validate_quaternion_transform()`
3. Use FK validator to compare Unity vs Python FK results
4. Check quaternion format (Unity uses x,y,z,w; ROS uses w,x,y,z)
5. Enable debug logging in both Unity and Python

### Validation Failures

**Problem**: FK/IK validation shows large errors

**Possible Causes**:
1. URDF model mismatch between Unity and Python
2. Different joint angle conventions (degrees vs radians)
3. Incorrect coordinate transformations
4. Numerical precision issues

**Solutions**:
- Verify URDF parameters match Unity articulation body setup
- Ensure joint angles are in radians (not degrees)
- Check transformation functions in `coordinate_transforms.py`
- Tighten IK solver tolerance if needed

### Performance Issues

**Problem**: IK solving is slow (>100ms per request)

**Solutions**:
- Typical IK solve time: 10-50ms per request
- Provide better initial guess (closer to expected solution)
- Don't call IK every frame - interpolate between solutions
- Use multiple IK servers for parallel solving
- Consider caching common poses

## Advanced Configuration

### Multiple Robot Support

Run multiple IK servers on different ports for multi-robot control:

```bash
# Terminal 1 - Left robot
python main.py --port 5010

# Terminal 2 - Right robot
python main.py --port 5011
```

Configure each Unity robot's `UR5IKSolver` component with the corresponding port.

### Custom IK Solvers

Modify the solver in `main.py`:

```python
# Default: Levenberg-Marquardt (best accuracy)
result = self.ur5.ik_LM(T_target, q0=current_angles, tol=IK_SOLVER_TOLERANCE)

# Alternative: Newton-Raphson (faster)
result = self.ur5.ik_NR(T_target, q0=current_angles)

# Alternative: LM Sugihara (good for singularities)
result = self.ur5.ikine_LMS(T_target, q0=current_angles)
```

### Adjusting Solver Tolerance

Edit `constants.py`:

```python
IK_SOLVER_TOLERANCE: Final[float] = 1e-10  # Very strict (default)
IK_SOLVER_TOLERANCE: Final[float] = 1e-6   # Relaxed (faster)
```

Stricter tolerance = more accurate but slower convergence.

### Network Configuration

For remote server deployment, edit `constants.py`:

```python
DEFAULT_HOST: Final[str] = "0.0.0.0"  # Listen on all interfaces
IK_SERVER_PORT: Final[int] = 5010      # Custom port
```

## Dependencies

- **Python**: 3.8+
- **roboticstoolbox-python**: UR5 robot model and IK solvers
- **spatialmath-python**: SE3 transforms, quaternion operations
- **numpy**: Numerical computations

Install all dependencies:
```bash
pip install roboticstoolbox-python spatialmath-python numpy
```

## References

- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
- [Spatialmath for Python](https://github.com/petercorke/spatialmath-python)
- [UR5 Robot Specifications](https://www.universal-robots.com/products/ur5-robot/)
- [Unity ArticulationBody Documentation](https://docs.unity3d.com/Manual/class-ArticulationBody.html)

## Support

For issues or questions:
1. Check Unity Console for error messages
2. Check Python server terminal for server-side errors
3. Enable debug logging for detailed diagnostics
4. Run FK validator to isolate coordinate system issues
5. Verify URDF model matches Unity robot configuration

---

**Note**: This system uses accurate IK solving with automatic coordinate conversion. Always validate your setup using the FK/IK validator before production use.

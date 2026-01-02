# UR5 Inverse Kinematics Server

This folder contains the IK (Inverse Kinematics) server implementation for the UR5 robot arm.

## Contents

- **ur5_ik_server.py**: TCP server that provides IK solving for Unity clients
- **ur5_fk_validator.py**: Forward and inverse kinematics validation tool
- **ur5.urdf**: URDF model of the UR5 robot
- **constants.py**: Shared constants for protocols and configurations
- **coordinate_transforms.py**: Unity ↔ ROS coordinate system transformations
- **IK_SERVER_README.md**: Detailed documentation for the IK server

## Quick Start

### Running the IK Server

```bash
cd ik_server
python ur5_ik_server.py
```

Optional arguments:
- `--host`: Server host address (default: 127.0.0.1)
- `--port`: Server port (default: 5010)

### Running the FK/IK Validator

```bash
python ur5_fk_validator.py
```

Optional arguments:
- `--host`: Unity server host (default: localhost)
- `--port`: Unity server port (default: 5005)
- `--tolerance`: Position error tolerance in meters (default: 0.01)
- `--rotation-tolerance`: Rotation error tolerance in radians (default: 0.05)
- `--joint-tolerance`: Joint angle error tolerance in radians (default: 0.01)

## Protocol

The IK server uses a binary TCP protocol:

**Request (104 bytes):**
- 3 doubles (24 bytes): Target position (x, y, z) in Unity coordinates
- 4 doubles (32 bytes): Target rotation quaternion (x, y, z, w) in Unity
- 6 doubles (48 bytes): Current joint angles in radians

**Response:**
- 1 byte: Success flag (1=success, 0=failure)
- 48 bytes: 6 joint angles as doubles (if success)

## Coordinate Systems

- **Unity**: Left-handed (X-right, Y-up, Z-forward)
- **ROS/RTB**: Right-handed (X-forward, Y-left, Z-up)

Transformations are handled automatically by `coordinate_transforms.py`.

## Dependencies

- roboticstoolbox-python
- spatialmath-python
- numpy

See IK_SERVER_README.md for detailed setup instructions.

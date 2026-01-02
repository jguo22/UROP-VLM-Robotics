"""
Constants for Unity-ROS Communication

This module defines all protocol constants, default values, and magic numbers
used across the Unity-ROS bridge communication system.
"""

from typing import Final

# ============================================================================
# Network Configuration
# ============================================================================

DEFAULT_HOST: Final[str] = "127.0.0.1"
"""Default host address for TCP connections"""

# Port assignments
UNITY_VLA_PORT: Final[int] = 5000
"""Port for Unity-VLA image/action communication"""

FK_VALIDATOR_PORT: Final[int] = 5005
"""Port for Forward Kinematics validation"""

IK_SERVER_PORT: Final[int] = 5010
"""Port for Inverse Kinematics server"""

# Connection settings
DEFAULT_MAX_RETRIES: Final[int] = 5
"""Default maximum number of connection retry attempts"""

DEFAULT_RETRY_DELAY: Final[float] = 2.0
"""Default delay between connection retries (seconds)"""

DEFAULT_SOCKET_TIMEOUT: Final[float] = 600.0
"""Default socket timeout (seconds) - 10 minutes"""

# ============================================================================
# Protocol Message Types
# ============================================================================

MSG_TYPE_IMAGE_REQUEST: Final[int] = 0x00
"""Message type byte for requesting an image from Unity"""

MSG_TYPE_ACTION: Final[int] = 0x01
"""Message type byte for sending an action to Unity"""

# ============================================================================
# Protocol Acknowledgments
# ============================================================================

ACK_SUCCESS: Final[bytes] = b"\x01"
"""Success acknowledgment byte"""

ACK_FAILURE: Final[bytes] = b"\x00"
"""Failure acknowledgment byte"""

# ============================================================================
# Data Sizes (in bytes)
# ============================================================================

# Basic data types
BYTES_PER_DOUBLE: Final[int] = 8
"""Size of a double (float64) in bytes"""

BYTES_PER_FLOAT: Final[int] = 4
"""Size of a float (float32) in bytes"""

BYTES_PER_INT: Final[int] = 4
"""Size of an int32 in bytes"""

# IK/FK Protocol
POSITION_DOUBLES: Final[int] = 3
"""Number of doubles for 3D position (x, y, z)"""

QUATERNION_DOUBLES: Final[int] = 4
"""Number of doubles for quaternion (x, y, z, w)"""

JOINT_ANGLES_COUNT: Final[int] = 6
"""Number of joint angles for UR5 robot"""

IK_REQUEST_DOUBLES: Final[int] = POSITION_DOUBLES + QUATERNION_DOUBLES + JOINT_ANGLES_COUNT
"""Total doubles in IK request: pos(3) + quat(4) + joints(6) = 13"""

IK_REQUEST_BYTES: Final[int] = IK_REQUEST_DOUBLES * BYTES_PER_DOUBLE
"""Total bytes in IK request: 13 * 8 = 104 bytes"""

IK_RESPONSE_HEADER_BYTES: Final[int] = 1
"""Size of IK response header (success flag)"""

IK_RESPONSE_JOINTS_BYTES: Final[int] = JOINT_ANGLES_COUNT * BYTES_PER_DOUBLE
"""Size of joint angles in IK response: 6 * 8 = 48 bytes"""

IK_RESPONSE_TOTAL_BYTES: Final[int] = IK_RESPONSE_HEADER_BYTES + IK_RESPONSE_JOINTS_BYTES
"""Total IK response size: 1 + 48 = 49 bytes"""

# FK Protocol
FK_DATA_DOUBLES: Final[int] = POSITION_DOUBLES + QUATERNION_DOUBLES + JOINT_ANGLES_COUNT
"""Total doubles in FK data packet: pos(3) + quat(4) + joints(6) = 13"""

FK_DATA_BYTES: Final[int] = FK_DATA_DOUBLES * BYTES_PER_DOUBLE
"""Total bytes in FK data packet: 13 * 8 = 104 bytes"""

# ============================================================================
# Robot Configuration
# ============================================================================

UR5_DOF: Final[int] = 6
"""Degrees of freedom for UR5 robot (number of joints)"""

ACTION_VECTOR_SIZE: Final[int] = 7
"""Size of action vector: 6 DOF end-effector delta + 1 gripper state"""

# ============================================================================
# Tolerance Values
# ============================================================================

DEFAULT_POSITION_TOLERANCE: Final[float] = 0.01
"""Default position error tolerance in meters"""

DEFAULT_ROTATION_TOLERANCE: Final[float] = 0.05
"""Default rotation error tolerance in radians (~2.87 degrees)"""

DEFAULT_JOINT_TOLERANCE: Final[float] = 0.01
"""Default joint angle error tolerance in radians (~0.57 degrees)"""

IK_SOLVER_TOLERANCE: Final[float] = 1e-6
"""Tolerance for IK solver convergence"""

# ============================================================================
# File Paths
# ============================================================================

UR5_URDF_FILENAME: Final[str] = "ur5.urdf"
"""Filename for UR5 URDF model"""

PROFILE_DIR: Final[str] = "profiles"
"""Directory for saving profiling data"""

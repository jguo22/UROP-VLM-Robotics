"""
Coordinate System Transformations for Unity-ROS Communication

This module provides transformations between Unity's left-handed coordinate system
and ROS/Robotics Toolbox's right-handed coordinate system.

Coordinate Systems:
    Unity (Left-handed):
        - X-axis: Right
        - Y-axis: Up
        - Z-axis: Forward

    ROS/RTB (Right-handed):
        - X-axis: Forward
        - Y-axis: Left
        - Z-axis: Up

Transformation Mapping:
    ROS/RTB ← Unity:
        - X_ros = Z_unity  (forward)
        - Y_ros = -X_unity (left)
        - Z_ros = Y_unity  (up)

    Unity ← ROS/RTB:
        - X_unity = -Y_ros (right)
        - Y_unity = Z_ros  (up)
        - Z_unity = X_ros  (forward)
"""

import numpy as np
from spatialmath import UnitQuaternion
from typing import Union


def unity_to_ros_position(unity_pos: np.ndarray) -> np.ndarray:
    """
    Convert position from Unity to ROS coordinate system.

    Args:
        unity_pos: Position in Unity coordinates [x, y, z]

    Returns:
        Position in ROS coordinates [x, y, z]

    Example:
        >>> unity_pos = np.array([1.0, 2.0, 3.0])  # right=1, up=2, forward=3
        >>> ros_pos = unity_to_ros_position(unity_pos)
        >>> # ros_pos = [3.0, -1.0, 2.0]  # forward=3, left=-1, up=2
    """
    x_unity, y_unity, z_unity = unity_pos
    return np.array([
        z_unity,   # X_ros = Z_unity (forward)
        -x_unity,  # Y_ros = -X_unity (left)
        y_unity    # Z_ros = Y_unity (up)
    ])


def ros_to_unity_position(ros_pos: np.ndarray) -> np.ndarray:
    """
    Convert position from ROS to Unity coordinate system.

    Args:
        ros_pos: Position in ROS coordinates [x, y, z]

    Returns:
        Position in Unity coordinates [x, y, z]

    Example:
        >>> ros_pos = np.array([3.0, -1.0, 2.0])  # forward=3, left=-1, up=2
        >>> unity_pos = ros_to_unity_position(ros_pos)
        >>> # unity_pos = [1.0, 2.0, 3.0]  # right=1, up=2, forward=3
    """
    x_ros, y_ros, z_ros = ros_pos
    return np.array([
        -y_ros,  # X_unity = -Y_ros (right)
        z_ros,   # Y_unity = Z_ros (up)
        x_ros    # Z_unity = X_ros (forward)
    ])


def unity_to_ros_quaternion(unity_quat: np.ndarray) -> UnitQuaternion:
    """
    Convert quaternion from Unity to ROS coordinate system.

    Unity quaternions are in [x, y, z, w] format.
    This function accounts for the coordinate system handedness change.

    Args:
        unity_quat: Quaternion in Unity format [x, y, z, w]

    Returns:
        UnitQuaternion object for use with ROS/RTB

    Note:
        The transformation applies the same axis remapping as positions:
        Unity (x, y, z) → ROS (z, -x, y)
        This affects how quaternion components are remapped.
    """
    x, y, z, w = unity_quat

    # Apply coordinate transformation to quaternion components
    # ROS quaternion in [w, x, y, z] format (spatialmath convention)
    ros_quat = UnitQuaternion(
        [w, -z, x, -y],  # [w, x_ros, y_ros, z_ros]
        norm=True
    )

    return ros_quat


def unity_to_ros_quaternion_array(unity_quat: np.ndarray) -> np.ndarray:
    """
    Convert quaternion from Unity to ROS coordinate system, returning array.

    Args:
        unity_quat: Quaternion in Unity format [x, y, z, w]

    Returns:
        Quaternion in ROS format as numpy array [x, y, z, w]

    Note:
        This version returns a numpy array instead of UnitQuaternion,
        useful for validation and comparison operations.
    """
    x, y, z, w = unity_quat

    # Apply coordinate transformation
    # Return in [x, y, z, w] array format
    ros_quat = np.array([
        -z,  # x_ros
        x,   # y_ros
        -y,  # z_ros
        w    # w (unchanged)
    ])

    return ros_quat


def ros_to_unity_quaternion(ros_quat: Union[UnitQuaternion, np.ndarray]) -> np.ndarray:
    """
    Convert quaternion from ROS to Unity coordinate system.

    Args:
        ros_quat: Quaternion as UnitQuaternion or numpy array [x, y, z, w]

    Returns:
        Quaternion in Unity format [x, y, z, w]
    """
    # Extract components from UnitQuaternion if needed
    if isinstance(ros_quat, UnitQuaternion):
        # UnitQuaternion: v returns [x, y, z], s returns w
        x_ros, y_ros, z_ros = ros_quat.v
        w = ros_quat.s
    else:
        x_ros, y_ros, z_ros, w = ros_quat

    # Inverse transformation: ROS → Unity
    unity_quat = np.array([
        y_ros,   # x_unity = y_ros
        -z_ros,  # y_unity = -z_ros
        -x_ros,  # z_unity = -x_ros
        w        # w (unchanged)
    ])

    return unity_quat


def validate_position_transform(unity_pos: np.ndarray, tolerance: float = 1e-10) -> bool:
    """
    Validate that position transformation is invertible (round-trip test).

    Args:
        unity_pos: Test position in Unity coordinates
        tolerance: Acceptable numerical error

    Returns:
        True if round-trip transformation preserves the position
    """
    ros_pos = unity_to_ros_position(unity_pos)
    recovered_unity = ros_to_unity_position(ros_pos)
    error = np.linalg.norm(unity_pos - recovered_unity)
    return error < tolerance


def validate_quaternion_transform(unity_quat: np.ndarray, tolerance: float = 1e-10) -> bool:
    """
    Validate that quaternion transformation is invertible (round-trip test).

    Args:
        unity_quat: Test quaternion in Unity format [x, y, z, w]
        tolerance: Acceptable numerical error

    Returns:
        True if round-trip transformation preserves the quaternion
    """
    ros_quat = unity_to_ros_quaternion(unity_quat)
    recovered_unity = ros_to_unity_quaternion(ros_quat)

    # Normalize both quaternions for comparison
    unity_quat_norm = unity_quat / np.linalg.norm(unity_quat)
    recovered_unity_norm = recovered_unity / np.linalg.norm(recovered_unity)

    # Quaternions q and -q represent the same rotation
    error1 = np.linalg.norm(unity_quat_norm - recovered_unity_norm)
    error2 = np.linalg.norm(unity_quat_norm + recovered_unity_norm)
    error = min(error1, error2)

    return error < tolerance


# ============================================================================
# Legacy support - for backward compatibility with ur5_fk_validator.py
# ============================================================================

def transform_coordinates(
    unity_pos: np.ndarray,
    mode: str = 'unity_to_ros'
) -> np.ndarray:
    """
    Transform coordinates with mode selection (for backward compatibility).

    Args:
        unity_pos: Position vector
        mode: Transformation mode ('none' or 'unity_to_ros')

    Returns:
        Transformed position
    """
    if mode == 'none':
        return unity_pos
    elif mode == 'unity_to_ros':
        return unity_to_ros_position(unity_pos)
    else:
        raise ValueError(f"Unknown transformation mode: {mode}")


def transform_quaternion(
    unity_quat: np.ndarray,
    mode: str = 'unity_to_ros'
) -> np.ndarray:
    """
    Transform quaternion with mode selection (for backward compatibility).

    Args:
        unity_quat: Quaternion in [x, y, z, w] format
        mode: Transformation mode ('none' or 'unity_to_ros')

    Returns:
        Transformed quaternion in [x, y, z, w] format
    """
    if mode == 'none':
        return unity_quat
    elif mode == 'unity_to_ros':
        return unity_to_ros_quaternion_array(unity_quat)
    else:
        raise ValueError(f"Unknown transformation mode: {mode}")

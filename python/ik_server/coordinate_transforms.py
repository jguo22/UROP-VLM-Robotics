"""
Coordinate System Transformations for Unity-ROS Communication

This module provides transformations between Unity's left-handed coordinate system
and ROS/Robotics Toolbox's right-handed coordinate system.

Coordinate Systems:
    Unity (Left-handed):
        - X-axis: Right
        - Y-axis: Up
        - Z-axis: Forward

    ROS (Right-handed):
        - X-axis: Forward
        - Y-axis: Left
        - Z-axis: Up

Unity Quaternions are in [x, y, z, w] format.
ROS quaternions are in [w, x, y, z] format.
"""

import numpy as np


def unity_to_ros_position(unity_pos: np.ndarray) -> np.ndarray:
    """
    Convert position from Unity to ROS coordinate system.

    Args:
        unity_pos: Position in Unity coordinates [x, y, z]

    Returns:
        Position in ROS coordinates [x, y, z]
    """
    x_unity, y_unity, z_unity = unity_pos
    return np.array([
        z_unity,   # ROS X is Unity's forward (Z)
        -x_unity,  # ROS Y is Unity's left (-X)
        y_unity    # ROS Z is Unity's up (Y)
    ])


def ros_to_unity_position(ros_pos: np.ndarray) -> np.ndarray:
    """
    Convert position from ROS to Unity coordinate system.

    Args:
        ros_pos: Position in ROS coordinates [x, y, z]

    Returns:
        Position in Unity coordinates [x, y, z]
    """
    x_ros, y_ros, z_ros = ros_pos
    return np.array([
        -y_ros,  # Unity X is ROS's right (-Y)
        z_ros,   # Unity Y is ROS's up (Z)
        x_ros    # Unity Z is ROS's forward (X)
    ])


def unity_to_ros_quaternion(q_unity: np.ndarray) -> np.ndarray:
    """
    Convert quaternion from Unity to ROS coordinate system.

    Args:
        q_unity: Quaternion in Unity coordinates [w, x, y, z]

    Returns:
        Quaternion in ROS coordinates [w, x, y, z]
    """
    x, y, z, w = q_unity
    return np.array([w, -z, x, -y])


def ros_to_unity_quaternion(q_ros: np.ndarray) -> np.ndarray:
    """
    Convert quaternion from ROS to Unity coordinate system.

    Args:
        q_ros: Quaternion in ROS coordinates [w, x, y, z]

    Returns:
        Quaternion in Unity coordinates [w, x, y, z]
    """
    w, x, y, z = q_ros
    return np.array([y, -z, -x, w])


def validate_position_transform(
    unity_pos: np.ndarray,
    tolerance: float = 1e-10
) -> bool:
    """
    Validate that position transformation is invertible (round-trip test).

    Args:
        unity_pos: Test position in Unity coordinates [x, y, z]
        tolerance: Acceptable numerical error

    Returns:
        True if round-trip transformation preserves the position
    """
    ros_pos = unity_to_ros_position(unity_pos)
    recovered_unity = ros_to_unity_position(ros_pos)
    error = np.linalg.norm(unity_pos - recovered_unity)
    return bool(error <= tolerance)


def quaternion_angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Calculate the angle between two quaternions in radians."""
    dot = np.abs(np.dot(q1, q2))  # Absolute value to handle double cover
    dot = np.clip(dot, -1.0, 1.0)  # Ensure numerical stability
    return 2 * np.arccos(dot)


def validate_quaternion_transform(
    unity_quat: np.ndarray,
    tolerance: float = 1e-10
) -> bool:
    """
    Validate that quaternion transformation is invertible (round-trip test).

    Args:
        unity_quat: Test quaternion in Unity format [w, x, y, z]
        tolerance: Acceptable angular error in radians

    Returns:
        True if round-trip transformation preserves the quaternion
    """
    ros_quat = unity_to_ros_quaternion(unity_quat)
    recovered_unity = ros_to_unity_quaternion(ros_quat)

    # Calculate angular error between original and recovered quaternion
    error = quaternion_angle_between(unity_quat, recovered_unity)
    return error <= tolerance

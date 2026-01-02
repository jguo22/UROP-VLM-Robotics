"""
UR5 Forward and Inverse Kinematics Validator

This script validates both FK and IK for the UR5 robot:
- FK Validation: Connects to Unity TCP server and validates forward kinematics (position and rotation)
- IK Validation: Tests inverse kinematics round-trip accuracy (joint angles → FK → pose → IK → joint angles)
- Coordinate Transformation: Handles Unity to ROS coordinate system conversion

Binary protocol is much faster than JSON (no parsing overhead, smaller data size).
Protocol: 13 doubles (104 bytes) - [ee_x, ee_y, ee_z, quat_x, quat_y, quat_z, quat_w, j1, j2, j3, j4, j5, j6]
"""

import socket
import struct
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
import os

from constants import (
    DEFAULT_HOST,
    FK_VALIDATOR_PORT,
    FK_DATA_BYTES,
    DEFAULT_POSITION_TOLERANCE,
    DEFAULT_ROTATION_TOLERANCE,
    DEFAULT_JOINT_TOLERANCE,
    UR5_URDF_FILENAME,
    JOINT_ANGLES_COUNT,
)
from coordinate_transforms import transform_coordinates, transform_quaternion


class UR5FKValidator:
    """UR5 Forward Kinematics Validator using roboticstoolbox"""

    def __init__(
            self,
            host: str = DEFAULT_HOST,
            port: int = FK_VALIDATOR_PORT,
            tolerance: float = DEFAULT_POSITION_TOLERANCE,
            rotation_tolerance: float = DEFAULT_ROTATION_TOLERANCE,
            joint_tolerance: float = DEFAULT_JOINT_TOLERANCE,
            coordinate_mode: str = 'unity_to_ros'
    ):
        """
        Initialize the validator.

        Args:
                                        host (str): TCP server host
                                        port (int): TCP server port
                                        tolerance (float): Maximum allowable position error in meters
                                        rotation_tolerance (float): Maximum allowable rotation error in radians
                                        joint_tolerance (float): Maximum allowable joint angle error in radians
                                        coordinate_mode (str): Coordinate transformation mode:
                                                                        - 'none': No transformation
                                                                        - 'unity_to_ros': Unity (Y-up) to ROS (Z-up)
        """
        self.host = host
        self.port = port
        self.tolerance = tolerance
        self.rotation_tolerance = rotation_tolerance
        self.joint_tolerance = joint_tolerance
        self.coordinate_mode = coordinate_mode
        self.socket = None

        # Load UR5 model from URDF file
        urdf_path = os.path.join(os.path.dirname(__file__), UR5_URDF_FILENAME)
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        self.ur5 = rtb.Robot.URDF(urdf_path)
        print(f"Loaded UR5 robot from URDF: {urdf_path}")
        print(self.ur5)

        print(rtb.models.UR5())


    def connect(self):
        """Establish TCP connection to the server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Close TCP connection"""
        if self.socket:
            self.socket.close()
            print("Disconnected from server")

    def receive_data(self):
        """
        Receive data from TCP server in binary format.

        Binary format (104 bytes total):
                                        13 doubles: [ee_x, ee_y, ee_z, quat_x, quat_y, quat_z, quat_w, j1, j2, j3, j4, j5, j6]
                                        All values are float64 (double precision, little-endian)

        Returns:
                                        tuple: (end_effector_position, end_effector_rotation, joint_angles) as numpy arrays, or (None, None, None) if error
        """
        try:
            # Binary protocol: 13 doubles
            # Format: ee_x, ee_y, ee_z, quat_x, quat_y, quat_z, quat_w, j1, j2,
            # j3, j4, j5, j6
            data = self.socket.recv(FK_DATA_BYTES)
            if len(data) != FK_DATA_BYTES:
                print(f"Expected {FK_DATA_BYTES} bytes, got {len(data)}")
                return None, None, None

            # Unpack 13 doubles (little-endian)
            values = struct.unpack('<13d', data)

            end_effector_pos = np.array(values[:3])
            end_effector_quat = np.array(values[3:7])  # [x, y, z, w]
            joint_angles = np.array(values[7:])

            return end_effector_pos, end_effector_quat, joint_angles

        except struct.error as e:
            print(f"Binary unpacking error: {e}")
            return None, None, None
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None, None, None

    def calculate_forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics for given joint angles.

        Args:
                                        joint_angles (np.ndarray): Array of 6 joint angles in radians

        Returns:
                                        tuple: (position, quaternion) where position is [x, y, z] and quaternion is [x, y, z, w]
        """
        # Calculate forward kinematics using built-in UR5 model
        T = self.ur5.fkine(joint_angles)

        # Extract position (translation vector)
        position = T.t

        # Extract rotation as quaternion [x, y, z, w]
        # Convert rotation matrix to unit quaternion
        unit_quat = UnitQuaternion(T.R)

        # Debug: Check what we're getting
        # .vec returns [w, x, y, z], .v returns [x, y, z], .s returns w
        # We want to output [x, y, z, w]
        quat_v = unit_quat.v  # Vector part [x, y, z]
        quat_s = unit_quat.s  # Scalar part w

        # Construct [x, y, z, w]
        quaternion = np.concatenate([quat_v, [quat_s]])

        return position, quaternion

    def calculate_inverse_kinematics(
            self,
            target_position,
            target_rotation=None,
            q0=None
    ):
        """
        Calculate inverse kinematics for target pose.

        Args:
                        target_position (np.ndarray): Target position [x, y, z]
                        target_rotation (np.ndarray or UnitQuaternion, optional): Target rotation
                        q0 (np.ndarray, optional): Initial guess for joint angles

        Returns:
                        tuple: (success: bool, solution: np.ndarray or None, iterations: int)
        """
        try:
            # Create SE3 transform
            if target_rotation is None:
                # Default orientation - pointing down
                T_target = SE3.Trans(target_position)
            elif isinstance(target_rotation, UnitQuaternion):
                T_target = SE3.Rt(target_rotation.R, target_position)
            else:
                # Assume it's a rotation matrix
                T_target = SE3.Rt(target_rotation, target_position)

            # Solve IK using Levenberg-Marquardt
            result = self.ur5.ik_LM(T_target, q0=q0)

            print(result)
            # check if success or failure
            if len(result) >= 2:
                q_result = result[0]
                success = result[1] == 1 if isinstance(
                    result[1], int) else bool(result[1])
                iterations = result[3] if len(result) > 3 else 0
                return success, q_result if success else None, iterations
            else:
                return False, None, 0

        except Exception as e:
            print(f"IK calculation error: {e}")
            import traceback
            traceback.print_exc()
            return False, None, 0

    def validate(self, reported_position, calculated_position):
        """
        Validate that reported and calculated positions match within tolerance.

        Args:
                                        reported_position (np.ndarray): Reported end effector position
                                        calculated_position (np.ndarray): Calculated end effector position

        Returns:
                                        tuple: (bool: is_valid, float: error_magnitude, np.ndarray: error_vector)
        """
        # Calculate error
        error = calculated_position - reported_position
        error_magnitude = np.linalg.norm(error)

        # Check if within tolerance
        is_valid = error_magnitude <= self.tolerance

        return is_valid, error_magnitude, error

    def validate_rotation(self, reported_quat, calculated_quat):
        """
        Validate that reported and calculated rotations match within tolerance.

        Uses quaternion distance metric to compute angular difference.

        Args:
                                        reported_quat (np.ndarray): Reported quaternion [x, y, z, w]
                                        calculated_quat (np.ndarray): Calculated quaternion [x, y, z, w]

        Returns:
                                        tuple: (bool: is_valid, float: angular_error_rad)
        """
        # Normalize quaternions
        q1 = reported_quat / np.linalg.norm(reported_quat)
        q2 = calculated_quat / np.linalg.norm(calculated_quat)

        # Calculate quaternion dot product (cosine of half the angle between
        # them)
        dot_product = np.abs(np.dot(q1, q2))

        # Clamp to [-1, 1] to avoid numerical errors in arccos
        dot_product = np.clip(dot_product, -1.0, 1.0)

        # Angular error = 2 * arccos(|q1 · q2|)
        angular_error = 2.0 * np.arccos(dot_product)

        # Check if within tolerance
        is_valid = angular_error <= self.rotation_tolerance

        return is_valid, angular_error

    def validate_joint_angles(self, original_angles, calculated_angles):
        """
        Validate that original and calculated joint angles match within tolerance.

        Args:
                                        original_angles (np.ndarray): Original joint angles in radians
                                        calculated_angles (np.ndarray): Calculated joint angles from IK in radians

        Returns:
                                        tuple: (bool: is_valid, float: max_error_rad, np.ndarray: error_vector)
        """
        # Calculate error for each joint
        error = calculated_angles - original_angles

        # Normalize angles to [-pi, pi] for proper error calculation
        error = np.arctan2(np.sin(error), np.cos(error))

        # Find maximum error
        max_error = np.max(np.abs(error))

        # Check if all joints within tolerance
        is_valid = max_error <= self.joint_tolerance

        return is_valid, max_error, error

    def run_continuous_validation(self):
        """
        Continuously receive data and validate forward and inverse kinematics.

        Tests:
        1. Forward Kinematics: Joint angles → Position & Rotation
        2. Inverse Kinematics: Position & Rotation → Joint angles (round-trip test)
        """
        if not self.connect():
            return

        print(
            f"\nValidating forward and inverse kinematics (BINARY protocol)")
        print(f"Position tolerance: {self.tolerance}m")
        print(
            f"Rotation tolerance: {self.rotation_tolerance} rad ({np.degrees(self.rotation_tolerance):.2f}°)")
        print(
            f"Joint angle tolerance: {self.joint_tolerance} rad ({np.degrees(self.joint_tolerance):.2f}°)")
        print(f"Coordinate mode: {self.coordinate_mode}")
        print("-" * 80)

        try:
            iteration = 0
            while True:
                reported_ee_pos, reported_ee_quat, joint_angles = self.receive_data()
                if reported_ee_pos is None or reported_ee_quat is None or joint_angles is None:
                    print("No data received. Ending.")
                    break

                iteration += 1

                if len(joint_angles) != 6:
                    print(f"Expected 6 joint angles, got {len(joint_angles)}")
                    continue

                # Calculate forward kinematics
                calculated_ee_pos, calculated_ee_quat = self.calculate_forward_kinematics(
                    joint_angles)

                # Transform position and rotation coordinates
                transformed_ee_pos = transform_coordinates(
                    reported_ee_pos, self.coordinate_mode)
                transformed_ee_quat = transform_quaternion(
                    reported_ee_quat, self.coordinate_mode)

                # Validate position
                pos_valid, pos_error_mag, pos_error_vector = self.validate(
                    transformed_ee_pos, calculated_ee_pos)

                # Validate rotation
                rot_valid, rot_error = self.validate_rotation(
                    transformed_ee_quat, calculated_ee_quat)

                # Test inverse kinematics (round-trip test)
                # Convert calculated quaternion to UnitQuaternion for IK
                calc_unit_quat = UnitQuaternion(
                    calculated_ee_quat[3], calculated_ee_quat[:3]
                )
                ik_success, ik_joint_angles, ik_iterations = self.calculate_inverse_kinematics(
                    calculated_ee_pos,
                    calc_unit_quat,
                    q0=joint_angles  # Use current joint angles as initial guess
                )
                print(f"IK SUCCESS: {ik_success}")
                print(ik_joint_angles)
                print(ik_iterations)

                # Validate IK solution
                if ik_success and ik_joint_angles is not None:
                    ik_valid, ik_max_error, ik_error_vector = self.validate_joint_angles(
                        joint_angles, ik_joint_angles)
                else:
                    ik_valid = False
                    ik_max_error = np.inf
                    ik_error_vector = np.zeros(6)

                # Overall validity
                overall_valid = pos_valid and rot_valid and ik_valid

                # Print results
                print(f"\nIteration {iteration}:")
                print(
                    f"Original Joint Angles (rad): {[f'{j:.4f}' for j in joint_angles]}")

                print(f"\nForward Kinematics - Position Validation:")
                print(
                    f"	Unity EE:		[{reported_ee_pos[0]:.6f}, {reported_ee_pos[1]:.6f}, {reported_ee_pos[2]:.6f}]")
                print(
                    f"	Transformed EE: [{transformed_ee_pos[0]:.6f}, {transformed_ee_pos[1]:.6f}, {transformed_ee_pos[2]:.6f}]")
                print(
                    f"	Calculated EE:	[{calculated_ee_pos[0]:.6f}, {calculated_ee_pos[1]:.6f}, {calculated_ee_pos[2]:.6f}]")
                print(
                    f"	Error vector:	[{pos_error_vector[0]:.6f}, {pos_error_vector[1]:.6f}, {pos_error_vector[2]:.6f}]")
                print(f"  Error magnitude: {pos_error_mag:.6f}m")
                print(
                    f"	Status: {'✓ VALID' if pos_valid else '✗ INVALID (exceeds tolerance)'}")

                print(f"\nForward Kinematics - Rotation Validation:")
                print(
                    f"	Unity Quat:		  [{reported_ee_quat[0]:.6f}, {reported_ee_quat[1]:.6f}, {reported_ee_quat[2]:.6f}, {reported_ee_quat[3]:.6f}]")
                print(
                    f"	Transformed Quat: [{transformed_ee_quat[0]:.6f}, {transformed_ee_quat[1]:.6f}, {transformed_ee_quat[2]:.6f}, {transformed_ee_quat[3]:.6f}]")
                print(
                    f"	Calculated Quat:  [{calculated_ee_quat[0]:.6f}, {calculated_ee_quat[1]:.6f}, {calculated_ee_quat[2]:.6f}, {calculated_ee_quat[3]:.6f}]")
                print(
                    f"	Angular error: {rot_error:.6f} rad ({np.degrees(rot_error):.2f}°)")
                print(
                    f"	Status: {'✓ VALID' if rot_valid else '✗ INVALID (exceeds tolerance)'}")

                print(f"\nInverse Kinematics - Round-trip Validation:")
                if ik_success:
                    print(
                        f"	IK Joint Angles (rad): {[f'{j:.4f}' for j in ik_joint_angles]}")
                    print(
                        f"	Joint Errors (rad):    {[f'{e:.6f}' for e in ik_error_vector]}")
                    print(
                        f"	Max joint error: {ik_max_error:.6f} rad ({np.degrees(ik_max_error):.2f}°)")
                    print(f"  IK iterations: {ik_iterations}")
                    print(
                        f"	Status: {'✓ VALID' if ik_valid else '✗ INVALID (exceeds tolerance)'}")
                else:
                    print(f"  Status: ✗ IK FAILED TO CONVERGE")

                print(
                    f"\nOverall: {'✓ ALL VALID' if overall_valid else '✗ VALIDATION FAILED'}")
                print("-" * 80)

        except KeyboardInterrupt:
            print("\n\nValidation stopped by user")
        finally:
            self.disconnect()

    def run_single_validation(self, end_effector, joint_angles):
        """
        Run a single validation without TCP connection (for testing).

        Args:
                                        end_effector (list or np.ndarray): End effector position [x, y, z]
                                        joint_angles (list or np.ndarray): 6 joint angles in radians
        """
        end_effector = np.asarray(end_effector)
        joint_angles = np.asarray(joint_angles)

        calculated_ee_pos, calculated_ee_quat = self.calculate_forward_kinematics(
            joint_angles)
        is_valid, error_mag, error_vector = self.validate(
            end_effector, calculated_ee_pos)

        print(f"Reported EE:	{end_effector}")
        print(f"Calculated EE:	{calculated_ee_pos}")
        print(f"Error vector:	{error_vector}")
        print(f"Error magnitude: {error_mag:.6f}m")
        print(f"Status: {'✓ VALID' if is_valid else '✗ INVALID'}")

        return is_valid


def main():
    """Main entry point"""
    # Parse command line arguments
    import argparse

    parser = argparse.ArgumentParser(
        description='UR5 Forward and Inverse Kinematics Validator',
    )

    # FK options
    parser.add_argument('--host', type=str, default=DEFAULT_HOST,
                        help=f'Unity FK server host (default: {DEFAULT_HOST})')
    parser.add_argument('--port', type=int, default=FK_VALIDATOR_PORT,
                        help=f'Unity FK server port (default: {FK_VALIDATOR_PORT})')
    parser.add_argument(
        '--tolerance',
        type=float,
        default=DEFAULT_POSITION_TOLERANCE,
        help=f'Position error tolerance in meters (default: {DEFAULT_POSITION_TOLERANCE})')
    parser.add_argument(
        '--rotation-tolerance',
        type=float,
        default=DEFAULT_ROTATION_TOLERANCE,
        help=f'Rotation error tolerance in radians (default: {DEFAULT_ROTATION_TOLERANCE}, ~2.87 degrees)')
    parser.add_argument(
        '--joint-tolerance',
        type=float,
        default=DEFAULT_JOINT_TOLERANCE,
        help=f'Joint angle error tolerance in radians (default: {DEFAULT_JOINT_TOLERANCE}, ~0.57 degrees)')

    args = parser.parse_args()

    # Create validator
    validator = UR5FKValidator(
        host=args.host,
        port=args.port,
        tolerance=args.tolerance,
        rotation_tolerance=args.rotation_tolerance,
        joint_tolerance=args.joint_tolerance
    )

    validator.run_continuous_validation()


if __name__ == '__main__':
    main()

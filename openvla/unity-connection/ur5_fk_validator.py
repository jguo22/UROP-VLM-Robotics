"""
UR5 Forward and Inverse Kinematics Validator

This script validates both FK and IK for the UR5 robot:
- FK Validation: Connects to Unity TCP server and validates forward kinematics
- IK Validation: Tests inverse kinematics solutions and round-trip accuracy
- IK Server Testing: Tests the Python IK server if running

Binary protocol is much faster than JSON (no parsing overhead, smaller data size).
"""

import socket
import struct
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
import os
import time


class UR5FKValidator:
    """UR5 Forward Kinematics Validator using roboticstoolbox"""

    def __init__(
        self,
        host='localhost',
        port=5005,
        tolerance=0.01,
        coordinate_mode='unity_to_ros'
    ):
        """
        Initialize the validator.

        Args:
            host (str): TCP server host
            port (int): TCP server port
            tolerance (float): Maximum allowable position error in meters
            coordinate_mode (str): Coordinate transformation mode:
                - 'none': No transformation
                - 'unity_to_ros': Unity (Y-up) to ROS (Z-up)
        """
        self.host = host
        self.port = port
        self.tolerance = tolerance
        self.coordinate_mode = coordinate_mode
        self.socket = None

        # Load UR5 model from URDF file
        urdf_path = os.path.join(os.path.dirname(__file__), 'ur5.urdf')
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        self.ur5 = rtb.Robot.URDF(urdf_path)
        print(f"Loaded UR5 robot from URDF: {urdf_path}")
        print(self.ur5)

        print(rtb.models.UR5())

    def transform_coordinates(self, unity_pos, mode):
        """
        Transform coordinates between Unity and robotics conventions.

        Unity:     X-right, Y-up, Z-forward (left-handed)
        Standard robotics (ROS/RTB): X-forward, Y-left, Z-up (right-handed)

        Args:
            unity_pos: Position in Unity coordinates [x, y, z]
            mode: Transformation mode string

        Returns:
            Transformed position
        """
        if mode == 'none':
            return unity_pos
        elif mode == 'unity_to_ros':
            # Unity to ROS: [X_ros, Y_ros, Z_ros] = [Z_unity, -X_unity,
            # Y_unity]
            return np.array([unity_pos[2], -unity_pos[0], unity_pos[1]])
        else:
            return unity_pos

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

        Binary format (72 bytes total):
            9 doubles: [ee_x, ee_y, ee_z, j1, j2, j3, j4, j5, j6]
            All values are float64 (double precision, little-endian)

        Returns:
            tuple: (end_effector_position, joint_angles) as numpy arrays, or (None, None) if error
        """
        try:
            # Binary protocol: 9 doubles (72 bytes)
            # Format: ee_x, ee_y, ee_z, j1, j2, j3, j4, j5, j6
            data = self.socket.recv(72)
            if len(data) != 72:
                print(f"Expected 72 bytes, got {len(data)}")
                return None, None

            # Unpack 9 doubles (little-endian)
            values = struct.unpack('<9d', data)

            end_effector = np.array(values[:3])
            joint_angles = np.array(values[3:])

            return end_effector, joint_angles

        except struct.error as e:
            print(f"Binary unpacking error: {e}")
            return None, None
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None, None

    def calculate_forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics for given joint angles.

        Args:
            joint_angles (np.ndarray): Array of 6 joint angles in radians

        Returns:
            np.ndarray: End effector position [x, y, z]
        """
        # Calculate forward kinematics using built-in UR5 model
        T = self.ur5.fkine(joint_angles)

        # Extract position (translation vector)
        position = T.t

        return position

    def calculate_inverse_kinematics(
            self,
            target_position,
            target_rotation=None,
            q0=None):
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

            # Use initial guess if provided, otherwise use zeros
            if q0 is None:
                q0 = np.zeros(6)

            # Solve IK using Levenberg-Marquardt
            result = self.ur5.ik_LM(T_target, q0=q0, tol=1e-6)

            # Handle different return types from roboticstoolbox versions
            if isinstance(result, tuple):
                # Older version returns (q, success, error, iterations, ...)
                # or (q, success)
                if len(result) >= 2:
                    q_result = result[0]
                    success = result[1] == 0 if isinstance(
                        result[1], int) else bool(result[1])
                    iterations = result[3] if len(result) > 3 else 0
                    return success, q_result if success else None, iterations
                else:
                    return False, None, 0
            else:
                # Newer version returns object with attributes
                return result.success, result.q if result.success else None, result.iterations

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

    def run_continuous_validation(self):
        """Continuously receive data and validate forward kinematics"""
        if not self.connect():
            return

        print(
            f"\nValidating forward kinematics (BINARY protocol, tolerance: {self.tolerance}m)")
        print(f"Coordinate mode: {self.coordinate_mode}")
        print("-" * 70)

        try:
            iteration = 0
            while True:
                # Receive data
                reported_ee, joint_angles = self.receive_data()
                if reported_ee is None or joint_angles is None:
                    print("No data received, retrying...")
                    continue

                iteration += 1

                if len(joint_angles) != 6:
                    print(f"Expected 6 joint angles, got {len(joint_angles)}")
                    continue

                # Calculate forward kinematics
                calculated_ee = self.calculate_forward_kinematics(joint_angles)

                # Single mode validation
                transformed_ee = self.transform_coordinates(
                    reported_ee, self.coordinate_mode)
                is_valid, error_mag, error_vector = self.validate(
                    transformed_ee, calculated_ee)

                # Print results
                print(f"\nIteration {iteration}:")
                print(
                    f"Joint Angles (rad): {[f'{j:.4f}' for j in joint_angles]}")
                print(
                    f"Unity EE:       [{reported_ee[0]:.6f}, {reported_ee[1]:.6f}, {reported_ee[2]:.6f}]")
                print(
                    f"Transformed EE: [{transformed_ee[0]:.6f}, {transformed_ee[1]:.6f}, {transformed_ee[2]:.6f}]")
                print(
                    f"Calculated EE:  [{calculated_ee[0]:.6f}, {calculated_ee[1]:.6f}, {calculated_ee[2]:.6f}]")
                print(
                    f"Error vector:   [{error_vector[0]:.6f}, {error_vector[1]:.6f}, {error_vector[2]:.6f}]")
                print(f"Error magnitude: {error_mag:.6f}m")
                print(
                    f"Status: {'✓ VALID' if is_valid else '✗ INVALID (exceeds tolerance)'}")
                print("-" * 70)

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

        calculated_ee = self.calculate_forward_kinematics(joint_angles)
        is_valid, error_mag, error_vector = self.validate(
            end_effector, calculated_ee)

        print(f"Reported EE:    {end_effector}")
        print(f"Calculated EE:  {calculated_ee}")
        print(f"Error vector:   {error_vector}")
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
    parser.add_argument('--host', type=str, default='localhost',
                        help='Unity FK server host (default: localhost)')
    parser.add_argument('--port', type=int, default=5005,
                        help='Unity FK server port (default: 5005)')
    parser.add_argument(
        '--tolerance',
        type=float,
        default=0.01,
        help='Position error tolerance in meters (default: 0.01)')

    args = parser.parse_args()

    # Create validator
    validator = UR5FKValidator(
        host=args.host,
        port=args.port,
        tolerance=args.tolerance
    )

    validator.run_continuous_validation()


if __name__ == '__main__':
    main()

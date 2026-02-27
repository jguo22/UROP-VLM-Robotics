import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
from unityenv import PYTHON_DIR


class UR5IKSolver:
    """UR5 Inverse Kinematics Solver using roboticstoolbox"""

    def __init__(self, coordinate_mode='unity_to_ros'):
        """
        Initialize solver.

        Args:
                                        coordinate_mode (str): Coordinate transformation mode:
                                                                        - 'none': No transformation
                                                                        - 'unity_to_ros': Unity (Y-up) to ROS (Z-up)
        """

        self.coordinate_mode = coordinate_mode

        # Load UR5 model from URDF file
        urdf_path = PYTHON_DIR / 'ur5.urdf'
        if not urdf_path.exists():
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        self.ur5 = rtb.Robot.URDF(urdf_path)
        # print(f"Loaded UR5 robot from URDF: {urdf_path}")
        # print(self.ur5)

        # print(rtb.models.UR5())

    def transform_coordinates(self, unity_pos):
        """
        Transform coordinates between Unity and robotics conventions.

        Unity:	   X-right, Y-up, Z-forward (left-handed)
        Standard robotics (ROS/RTB): X-forward, Y-left, Z-up (right-handed)

        Args:
                                        unity_pos: Position in Unity coordinates [x, y, z]
                                        mode: Transformation mode string

        Returns:
                                        Transformed position
        """
        if self.coordinate_mode == 'none':
            return unity_pos
        elif self.coordinate_mode == 'unity_to_ros':
            # Unity to ROS: [X_ros, Y_ros, Z_ros] = [Z_unity, -X_unity,
            # Y_unity]
            return np.array([unity_pos[2], -unity_pos[0], unity_pos[1]])
        else:
            return unity_pos

    def transform_quaternion(self, unity_quat):
        """
        Transform quaternion between Unity and robotics conventions.

        Unity:	   X-right, Y-up, Z-forward (left-handed)
        Standard robotics (ROS/RTB): X-forward, Y-left, Z-up (right-handed)

        Args:
                                        unity_quat: Quaternion in Unity coordinates [x, y, z, w]
                                        mode: Transformation mode string

        Returns:
                                        Transformed quaternion [x, y, z, w]
        """
        if self.coordinate_mode == 'none':
            return unity_quat
        elif self.coordinate_mode == 'unity_to_ros':
            # Unity to ROS coordinate transformation for quaternions
            # Unity: X-right, Y-up, Z-forward
            # ROS:	 X-forward, Y-left, Z-up
            # Mapping: ROS_X = Unity_Z, ROS_Y = -Unity_X, ROS_Z = Unity_Y

            # Convert Unity quaternion to ROS quaternion
            # The transformation is: q_ros = q_transform * q_unity
            # Where q_transform represents the coordinate system change

            x, y, z, w = unity_quat

            # Apply coordinate transformation to quaternion
            # ROS quaternion components based on Unity to ROS mapping
            ros_quat = np.array([
                -z,
                x,
                -y,
                w
            ])

            return ros_quat
        else:
            return unity_quat

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
                # Default orientation - pointing up (not down)
                # T_target = SE3.Trans(target_position)
                T_target = SE3.Trans(target_position) @ SE3.Rx(np.pi)
            elif isinstance(target_rotation, UnitQuaternion):
                T_target = SE3.Rt(target_rotation.R, target_position)
            else:
                # Assume it's a rotation matrix
                T_target = SE3.Rt(target_rotation, target_position)

            # Solve IK using Levenberg-Marquardt
            result = self.ur5.ik_LM(T_target, q0=q0)

            # print(result)
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

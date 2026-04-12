import math
import os
import roboticstoolbox as rtb
from spatialmath import SE3

from constants import (
    UR5_URDF_FILENAME,
    IK_SOLVER_TOLERANCE,
)
from coordinate_transforms import unity_to_ros_position, unity_to_ros_quaternion, arrayToQuaternion


class NumericSolver():
    def __init__(self):
        # Load UR5 model from URDF file
        urdf_path = os.path.join(os.path.dirname(__file__), UR5_URDF_FILENAME)
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        self.ur5 = rtb.Robot.URDF(urdf_path)
        print(f"Loaded UR5 robot model from: {urdf_path}")
        print(self.ur5)
        print(rtb.models.UR5())

    def solve_ik(self, target_position, target_rotation, current_angles):
        """
        Solve inverse kinematics for target pose.

        Args:
            target_position: np.array([x, y, z]) in Unity coordinates
            target_rotation: np.array([x, y, z, w]) quaternion in Unity
            current_angles: np.array of 6 joint angles in radians

        Returns:
            np.array of 6 joint angles in radians, or None if no solution
        """
        try:
            # Convert Unity coordinates to ROS
            ros_position = unity_to_ros_position(target_position)
            ros_quat = unity_to_ros_quaternion(
                target_rotation)  # Returns [w, x, y, z]
            ros_rotation = arrayToQuaternion(ros_quat)

            # Create SE3 transform from position and quaternion
            T_target = SE3.Rt(ros_rotation.R, ros_position)

            # use current angles as seed
            # but with high value for 2nd joint to encourage elbow up
            current_angles[2] = max(current_angles[2], math.pi / 2)
            result = self.ur5.ik_LM(
                T_target,
                q0=current_angles,
                tol=IK_SOLVER_TOLERANCE)

            # result is a tuple: (q, success, iterations, searches, residual)
            if bool(result[1]):
                solution = result[0]
                print(solution[2])

                return solution
            else:
                print(f"IK solution failed (residual: {result[4]})")
                return None

        except Exception as e:
            print(f"Error in solve_ik: {e}")
            import traceback
            traceback.print_exc()
            return None

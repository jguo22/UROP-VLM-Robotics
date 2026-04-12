import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from ur_analytic_ik import ur5 as ur5_ik

from constants import UR5_URDF_FILENAME
from coordinate_transforms import unity_to_ros_position, unity_to_ros_quaternion, arrayToQuaternion


class AnalyticSolver():
    def __init__(self):
        # DH model used only to compute the base frame offset against the URDF
        self.ur5_dh = rtb.models.DH.UR5()

        urdf_path = os.path.join(os.path.dirname(__file__), UR5_URDF_FILENAME)
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        ur5_urdf = rtb.Robot.URDF(urdf_path)

        # Compute base offset so DH frame matches URDF world frame
        q_ref = [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0]
        T_dh = self.ur5_dh.fkine(q_ref)
        T_urdf = ur5_urdf.fkine(q_ref)
        self.ur5_dh.base = T_urdf * T_dh.inv()

        # verify correctness
        q_test = [
            [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0],
            [0, 0, 0, 0, 0, 0],
            [np.pi / 4, -np.pi / 3, np.pi / 3, -np.pi / 4, np.pi / 2, 0],
        ]
        for q in q_test:
            t1 = ur5_urdf.fkine(q)
            t2 = self.ur5_dh.fkine(q)
            assert (t1 == t2)

        # self.base_inv is used to transform targets
        # from world frame to DH frame
        self.base_inv = self.ur5_dh.base.inv()

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
            ros_quat = unity_to_ros_quaternion(target_rotation)  # [w, x, y, z]
            ros_rotation = arrayToQuaternion(ros_quat)

            # Target in world frame
            T_world = SE3.Rt(ros_rotation.R, ros_position)

            # Transform into DH base frame for ur_analytic_ik
            T_robot = self.base_inv * T_world

            # Get all analytical solutions
            solutions = ur5_ik.inverse_kinematics(T_robot.A)

            if len(solutions) == 0:
                print("IK: no solutions found")
                return None

            # Filter for elbow-up (q[2] > 0)
            elbow_up = [q for q in solutions if q[2] > 0]
            candidates = elbow_up if elbow_up else solutions
            print("CANDIDATES")
            print(candidates)

            # Among candidates, pick closest to current angles
            best = min(
                candidates,
                key=lambda q: np.linalg.norm(
                    q - current_angles))
            return np.array(best)

        except Exception as e:
            print(f"Error in solve_ik: {e}")
            import traceback
            traceback.print_exc()
            return None

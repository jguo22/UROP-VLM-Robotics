import os
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from constants import UR5_URDF_FILENAME
from coordinate_transforms import unity_to_ros_position, unity_to_ros_quaternion, arrayToQuaternion

import inspect
import roboticstoolbox as rtb
ur5 = rtb.models.DH.UR5()
print(inspect.signature(ur5.ikine_6s))
print([m for m in dir(type(ur5)) if not m.startswith('__')])


class AnalyticSolver():
    def __init__(self):
        # Use DH model for ikine_6s (analytical, elbow-up config)
        self.ur5 = rtb.models.DH.UR5()

        # Load URDF robot to compute base offset against DH model
        urdf_path = os.path.join(os.path.dirname(__file__), UR5_URDF_FILENAME)
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        ur5_urdf = rtb.Robot.URDF(urdf_path)

        print(ur5_urdf)
        print(self.ur5.base)
        # Compute base offset so DH model FK matches URDF FK
        q_ref = [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0]
        T_dh = self.ur5.fkine(q_ref)
        T_urdf = ur5_urdf.fkine(q_ref)
        self.ur5.base = T_urdf * T_dh.inv()

        q_test = [
            [0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0],
            [0, 0, 0, 0, 0, 0],
            [np.pi / 4, -np.pi / 3, np.pi / 3, -np.pi / 4, np.pi / 2, 0],
        ]
        for q in q_test:
            t1 = ur5_urdf.fkine(q)
            t2 = self.ur5.fkine(q)
            assert (t1 == t2)

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

            # Solve IK analytically with elbow-up constraint (left, up,
            # no-flip)
            sol = self.ur5.ikine_6s(T_target, config='lun')

            if sol.success:
                return sol.q
            else:
                print("IK solution failed")
                return None

        except Exception as e:
            print(f"Error in solve_ik: {e}")
            import traceback
            traceback.print_exc()
            return None

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

    # Elbow-up seeds: base kept from current angles, shoulder/elbow fixed to
    # known good elbow-up configurations so ik_LM converges to the right
    # branch.
    _ELBOW_UP_SEEDS = [
        [None, -0.8, 1.0, -1.5, -1.57, 0.0],   # nominal home-like
        [None, -1.0, 1.2, -1.3, -1.57, 0.0],   # shoulder more raised
        [None, -0.5, 0.8, -1.8, -1.57, 0.0],   # shallower shoulder
        [None, -1.2, 1.5, -1.0, -1.57, 0.0],   # higher elbow bend
    ]

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

            # Build seeds: current angles (with elbow forced up) first,
            # then the fixed elbow-up seeds with the current base angle.
            seeds = []
            primary = current_angles.copy()
            primary[2] = max(primary[2], math.pi / 2)
            seeds.append(primary)

            for template in self._ELBOW_UP_SEEDS:
                seed = current_angles.copy()
                for j, v in enumerate(template):
                    if v is not None:
                        seed[j] = v
                seeds.append(seed)

            best_elbow_up = None
            best_fallback = None

            for seed in seeds:
                result = self.ur5.ik_LM(
                    T_target, q0=seed, tol=IK_SOLVER_TOLERANCE)
                if not bool(result[1]):
                    continue
                solution = result[0]
                if solution[2] >= 0:          # elbow-up: joint 2 positive
                    best_elbow_up = solution
                    break                      # take the first elbow-up solution
                if best_fallback is None:
                    best_fallback = solution   # keep first valid solution as fallback

            solution = best_elbow_up if best_elbow_up is not None else best_fallback
            if solution is not None:
                print(f"IK solution (elbow joint={solution[2]:.3f})")
                return solution

            print("IK solution failed across all seeds")
            return None

        except Exception as e:
            print(f"Error in solve_ik: {e}")
            import traceback
            traceback.print_exc()
            return None

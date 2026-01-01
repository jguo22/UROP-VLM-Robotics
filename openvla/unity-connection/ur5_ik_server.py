"""
UR5 Inverse Kinematics TCP Server

This server provides IK solving capabilities for the UR5 robot using roboticstoolbox.
It accepts TCP connections from Unity and solves IK for target poses or delta actions.

Protocol:
    Client sends:
        - 1 byte: command type (1=SolveIK, 2=SolveIKDelta)
        - Data depends on command type

    Server responds:
        - 1 byte: success flag (1=success, 0=failure)
        - 48 bytes: 6 joint angles as doubles (if success)

Command 1 - SolveIK:
    Request: [cmd(1)] + [target_pos(3d)] + [target_rot(4d)] + [current_angles(6d)]
    Total: 1 + 24 + 32 + 48 = 105 bytes

Command 2 - SolveIKDelta:
    Request: [cmd(1)] + [current_pos(3d)] + [current_rot(4d)] + [delta(7d)] + [current_angles(6d)]
    Total: 1 + 24 + 32 + 56 + 48 = 161 bytes
"""

import socket
import struct
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
import os


class UR5IKServer:
    """TCP server for UR5 inverse kinematics solving"""

    def __init__(self, host='127.0.0.1', port=5010):
        """
        Initialize the IK server.

        Args:
            host (str): Server host address
            port (int): Server port
        """
        self.host = host
        self.port = port
        self.socket = None

        # Load UR5 model from URDF file
        urdf_path = os.path.join(os.path.dirname(__file__), 'ur5.urdf')
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        self.ur5 = rtb.Robot.URDF(urdf_path)
        print(f"Loaded UR5 robot model from: {urdf_path}")
        print(self.ur5)

    def unity_to_ros_position(self, unity_pos):
        """
        Convert Unity position to ROS/RTB coordinate system.

        Unity: X-right, Y-up, Z-forward (left-handed)
        ROS/RTB: X-forward, Y-left, Z-up (right-handed)

        Transformation: [X_ros, Y_ros, Z_ros] = [Z_unity, -X_unity, Y_unity]

        Args:
            unity_pos: np.array([x, y, z]) in Unity coordinates

        Returns:
            np.array([x, y, z]) in ROS coordinates
        """
        return np.array([unity_pos[2], -unity_pos[0], unity_pos[1]])

    def ros_to_unity_position(self, ros_pos):
        """
        Convert ROS/RTB position to Unity coordinate system.

        Transformation: [X_unity, Y_unity, Z_unity] = [-Y_ros, Z_ros, X_ros]

        Args:
            ros_pos: np.array([x, y, z]) in ROS coordinates

        Returns:
            np.array([x, y, z]) in Unity coordinates
        """
        return np.array([-ros_pos[1], ros_pos[2], ros_pos[0]])

    def unity_to_ros_quaternion(self, unity_quat):
        """
        Convert Unity quaternion to ROS/RTB quaternion.

        Unity uses (x, y, z, w) format
        Need to account for coordinate system handedness change

        Args:
            unity_quat: np.array([x, y, z, w]) in Unity

        Returns:
            UnitQuaternion for ROS/RTB
        """
        # Unity quaternion [x, y, z, w]
        # Apply same coordinate transformation as position
        # For quaternions representing rotations, we need to convert the axis
        x, y, z, w = unity_quat

        # Convert to ROS convention: swap and negate components
        # This accounts for the coordinate system change
        ros_quat = UnitQuaternion(
            [w, z, -x, y], norm=True)  # [w, x, y, z] format

        return ros_quat

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
            ros_position = self.unity_to_ros_position(target_position)
            ros_rotation = self.unity_to_ros_quaternion(target_rotation)

            # Create SE3 transform from position and quaternion
            T_target = SE3.Rt(ros_rotation.R, ros_position)

            # Solve IK using Levenberg-Marquardt method
            # q0 is the initial guess (current configuration)
            solution = self.ur5.ik_LM(T_target, q0=current_angles, tol=1e-6)

            if solution.success:
                return solution.q
            else:
                print(f"IK solution failed: {solution.reason}")
                return None

        except Exception as e:
            print(f"Error in solve_ik: {e}")
            import traceback
            traceback.print_exc()
            return None

    def handle_client(self, client_socket, address):
        """
        Handle client connection and IK requests.

        Args:
            client_socket: Connected client socket
            address: Client address
        """
        print(f"Client connected from {address}")

        # Set socket to blocking mode with timeout
        client_socket.setblocking(True)
        client_socket.settimeout(10.0)  # 10 second timeout

        try:
            while True:
                # Read command type (1 byte)
                try:
                    cmd_data = client_socket.recv(1)
                    if not cmd_data or len(cmd_data) == 0:
                        print("Client disconnected (no data)")
                        break
                except socket.timeout:
                    print("Socket timeout, client may have disconnected")
                    break

                # Read data: 3d + 4d + 6d = 13 doubles = 104 bytes
                data = b''
                remaining = 104
                while remaining > 0:
                    chunk = client_socket.recv(remaining)
                    if not chunk:
                        break
                    data += chunk
                    remaining -= len(chunk)

                if len(data) != 104:
                    print(
                        f"Invalid data length for SolveIK: {len(data)}, expected 104")
                    break

                # Unpack: target_pos(3) + target_rot(4) + current_angles(6)
                values = struct.unpack('<13d', data)
                target_pos = np.array(values[0:3])
                target_rot = np.array(values[3:7])
                current_angles = np.array(values[7:13])

                print(
                    f"SolveIK request: pos={target_pos}, rot={target_rot}")

                # Solve IK
                solution = self.solve_ik(
                    target_pos, target_rot, current_angles)

                # Send response
                if solution is not None:
                    # Success: send 1 + 6 doubles
                    response = struct.pack(
                        'B', 1) + struct.pack('<6d', *solution)
                    client_socket.sendall(response)
                    print(f"Solution sent: {solution}")
                else:
                    # Failure: send 0
                    response = struct.pack('B', 0)
                    client_socket.sendall(response)
                    print("No solution found, sent failure response")

        except socket.timeout as e:
            print(f"Socket timeout: {e}")
        except Exception as e:
            print(f"Error handling client: {e}")
            import traceback
            traceback.print_exc()
        finally:
            try:
                client_socket.close()
            except BaseException:
                pass
            print(f"Client {address} disconnected")

    def start(self):
        """Start the TCP server and listen for connections"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            self.socket.bind((self.host, self.port))
            self.socket.listen(5)
            print(f"UR5 IK Server listening on {self.host}:{self.port}")
            print("Waiting for Unity client connections...")

            while True:
                client_socket, address = self.socket.accept()
                # Handle each client in the same thread (simple for now)
                # For production, consider threading or async
                self.handle_client(client_socket, address)

        except KeyboardInterrupt:
            print("\nServer stopped by user")
        except Exception as e:
            print(f"Server error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if self.socket:
                self.socket.close()
                print("Server socket closed")


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description='UR5 IK TCP Server')
    parser.add_argument('--host', type=str, default='127.0.0.1',
                        help='Server host (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=5010,
                        help='Server port (default: 5010)')

    args = parser.parse_args()

    # Create and start server
    server = UR5IKServer(host=args.host, port=args.port)
    server.start()


if __name__ == '__main__':
    main()

"""
UR5 Inverse Kinematics TCP Server

This server provides IK solving capabilities for the UR5 robot using roboticstoolbox.
It accepts TCP connections from Unity and solves IK for target poses.

Protocol:
    Client sends:
        - 104 bytes: [target_pos(3d)] + [target_rot(4d)] + [current_angles(6d)]
        - target_pos: 3 doubles (24 bytes) - target position xyz
        - target_rot: 4 doubles (32 bytes) - target quaternion xyzw
        - current_angles: 6 doubles (48 bytes) - current joint angles in radians

    Server responds:
        - 1 byte: success flag (1=success, 0=failure)
        - 48 bytes: 6 joint angles as doubles (if success)
"""

import socket
import struct
import numpy as np

from constants import (
    DEFAULT_HOST,
    IK_SERVER_PORT,
    IK_REQUEST_BYTES,
    JOINT_ANGLES_COUNT,
)
from ik_server.analytic_solver import AnalyticSolver
from ik_server.numeric_solver import NumericSolver


class UR5IKServer:
    """TCP server for UR5 inverse kinematics solving"""

    def __init__(self, host: str = DEFAULT_HOST, port: int = IK_SERVER_PORT):
        """
        Initialize the IK server.

        Args:
            host (str): Server host address
            port (int): Server port
        """
        self.host = host
        self.port = port
        self.socket = None

        self.ik_solver = NumericSolver()

    def handle_client(self, client_socket, address):
        """
        Handle client connection and IK requests.

        Args:
            client_socket: Connected client socket
            address: Client address
        """
        print(f"Client connected from {address}")

        # Enable TCP keepalive to prevent idle disconnects
        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

        # Set socket to blocking mode with 10-minute timeout
        client_socket.setblocking(True)
        client_socket.settimeout(600.0)  # 10 minute timeout

        try:
            while True:
                # Read data: 3d + 4d + 6d = 13 doubles
                data = b''
                remaining = IK_REQUEST_BYTES
                try:
                    while remaining > 0:
                        chunk = client_socket.recv(remaining)
                        if not chunk:
                            print("Client disconnected (no data)")
                            return
                        data += chunk
                        remaining -= len(chunk)
                except socket.timeout:
                    print("Socket timeout, client may have disconnected")
                    break

                if len(data) != IK_REQUEST_BYTES:
                    print(
                        f"Invalid data length for SolveIK: {len(data)}, expected {IK_REQUEST_BYTES}")
                    break

                # Unpack: target_pos(3) + target_rot(4) + current_angles(6)
                values = struct.unpack('<13d', data)
                target_pos = np.array(values[0:3])
                target_rot = np.array(values[3:7])
                current_angles = np.array(values[7:13])

                print(
                    f"SolveIK request: pos={target_pos}, rot={target_rot}")

                # Solve IK
                solution = self.ik_solver.solve_ik(
                    target_pos, target_rot, current_angles)

                # Send response
                if solution is not None:
                    # Success: send 1 byte header + joint angles
                    response = struct.pack(
                        'B', 1) + struct.pack(f'<{JOINT_ANGLES_COUNT}d', *solution)
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
    parser.add_argument('--host', type=str, default=DEFAULT_HOST,
                        help=f'Server host (default: {DEFAULT_HOST})')
    parser.add_argument('--port', type=int, default=IK_SERVER_PORT,
                        help=f'Server port (default: {IK_SERVER_PORT})')

    args = parser.parse_args()

    # Create and start server
    server = UR5IKServer(host=args.host, port=args.port)
    server.start()


if __name__ == '__main__':
    main()

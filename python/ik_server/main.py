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
import threading
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
    """TCP server for UR5 inverse kinematics solving.

    Spawns a thread per client so multiple UR5IKSolver instances
    (e.g. dual-arm scenes) can be served concurrently.
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = IK_SERVER_PORT):
        self.host = host
        self.port = port
        self.socket = None

        self.ik_solver = AnalyticSolver()
        self._solve_lock = threading.Lock()

    def handle_client(self, client_socket, address):
        print(f"Client connected from {address}")

        client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        client_socket.setblocking(True)
        client_socket.settimeout(600.0)

        try:
            while True:
                data = b''
                remaining = IK_REQUEST_BYTES
                try:
                    while remaining > 0:
                        chunk = client_socket.recv(remaining)
                        if not chunk:
                            print(f"[{address}] Client disconnected (no data)")
                            return
                        data += chunk
                        remaining -= len(chunk)
                except socket.timeout:
                    print(
                        f"[{address}] Socket timeout, client may have disconnected")
                    break

                if len(data) != IK_REQUEST_BYTES:
                    print(
                        f"[{address}] Invalid data length: {len(data)}, expected {IK_REQUEST_BYTES}")
                    break

                values = struct.unpack('<13d', data)
                target_pos = np.array(values[0:3])
                target_rot = np.array(values[3:7])
                current_angles = np.array(values[7:13])

                print(
                    f"[{address}] SolveIK request: pos={target_pos}, rot={target_rot}")

                with self._solve_lock:
                    solution = self.ik_solver.solve_ik(
                        target_pos, target_rot, current_angles)

                if solution is not None:
                    response = struct.pack(
                        'B', 1) + struct.pack(f'<{JOINT_ANGLES_COUNT}d', *solution)
                    client_socket.sendall(response)
                    print(f"[{address}] Solution sent: {solution}")
                else:
                    response = struct.pack('B', 0)
                    client_socket.sendall(response)
                    print(f"[{address}] No solution found, sent failure response")

        except socket.timeout as e:
            print(f"[{address}] Socket timeout: {e}")
        except Exception as e:
            print(f"[{address}] Error handling client: {e}")
            import traceback
            traceback.print_exc()
        finally:
            try:
                client_socket.close()
            except BaseException:
                pass
            print(f"[{address}] Client disconnected")

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
                thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, address),
                    daemon=True,
                )
                thread.start()

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

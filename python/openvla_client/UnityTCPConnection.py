import socket
import time
import struct
import numpy as np
import io
from PIL import Image

from constants import (
    DEFAULT_HOST,
    UNITY_VLA_PORT,
    MSG_TYPE_IMAGE_REQUEST,
    MSG_TYPE_ACTION,
    ACK_SUCCESS,
    DEFAULT_MAX_RETRIES,
    DEFAULT_RETRY_DELAY,
    BYTES_PER_INT,
)


class UnityTCPConnection:
    """Handles TCP communication with a Unity application for UR5 robot control.

    This class manages the low-level TCP connection to Unity, specifically designed
    for sending robot actions and receiving camera images.

    Protocol:
    - Image request: Send byte 0x00, receive 4-byte size (big-endian) then image data
    - Action send: Send byte 0x01, 4-byte action size, then action data (float32[])
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = UNITY_VLA_PORT):
        """Initialize the Unity TCP connection.

        Args:
                        host (str): IP address of the Unity application. Defaults to localhost.
                        port (int): Port number the Unity application is listening on. Defaults to 5000.
        """
        self.host = host
        self.port = port
        self.socket: socket.socket = None  # type: ignore
        self._connect()

    def _connect(self, max_retries: int = DEFAULT_MAX_RETRIES, retry_delay: float = DEFAULT_RETRY_DELAY) -> None:
        """Establish connection to Unity with retry logic.

        Args:
                        max_retries: Maximum number of connection attempts
                        retry_delay: Delay between retry attempts in seconds

        Raises:
                        ConnectionRefusedError: If connection fails after max_retries
        """
        for attempt in range(max_retries):
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port))
                print(f"Connected to Unity at {self.host}:{self.port}")
                return
            except ConnectionRefusedError as e:
                if attempt < max_retries - 1:
                    print(
                        f"Connection refused, retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    raise ConnectionRefusedError(
                        f"Failed to connect to Unity after {max_retries} attempts") from e

    def _receive_data(self, size: int) -> bytes:
        """Receive exactly 'size' bytes from the socket.

        Args:
                        size: Number of bytes to receive

        Returns:
                        bytes: The received data

        Raises:
                        ConnectionError: If connection is closed unexpectedly
        """
        data = bytearray()
        while len(data) < size:
            packet = self.socket.recv(size - len(data))
            if not packet:
                raise ConnectionError("Connection closed by Unity")
            data.extend(packet)
        return bytes(data)

    def receive_image(self) -> Image.Image:
        """Request and receive an image from Unity.

        Returns:
                        Image.Image: The received PIL Image.
        """
        # Send image request
        self.socket.sendall(bytes([MSG_TYPE_IMAGE_REQUEST]))

        # Get image size (4 bytes, big-endian)
        size_data = self._receive_data(BYTES_PER_INT)
        img_size = struct.unpack("!i", size_data)[0]

        # Get image data
        img_data = self._receive_data(img_size)
        return Image.open(io.BytesIO(img_data))

    def send_action(self, action: np.ndarray) -> bool:
        """Send a robot action to Unity.

        Args:
                        action: NumPy array of float32 values representing the robot action

        Returns:
                        bool: True if the action was sent successfully and acknowledged
        """
        # Convert action to bytes
        action_bytes = action.astype(np.float32).tobytes()

        # Send action message
        self.socket.sendall(bytes([MSG_TYPE_ACTION]))  # Message type
        self.socket.sendall(struct.pack("!i", len(action_bytes)))  # Size
        self.socket.sendall(action_bytes)  # Data

        # Wait for ACK (1 = success, 0 = failure)
        ack = self.socket.recv(1)
        return ack == ACK_SUCCESS

    def close(self) -> None:
        """Close the connection to Unity and clean up resources."""
        if self.socket:
            try:
                self.socket.close()
                print(f"Closed connection to Unity at {self.host}:{self.port}")
            except Exception as e:
                print(f"Error closing connection: {e}")
            finally:
                self.socket = None  # type: ignore

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

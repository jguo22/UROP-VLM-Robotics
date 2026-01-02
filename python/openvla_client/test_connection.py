# test_connection.py
import socket
import numpy as np
from PIL import Image
import io


class UnityTester:
    def __init__(self, host="127.0.0.1", port=5000):
        self.host = host
        self.port = port
        self.socket = None

    def connect(self):
        """Establish connection to Unity server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"✅ Connected to Unity at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False

    def test_image(self):
        """Test receiving an image from Unity"""
        try:
            # Request image (type=0)
            self.socket.sendall(b'\x00')

            # Get image size (4 bytes, big-endian)
            size_data = self.socket.recv(4)
            img_size = int.from_bytes(size_data, byteorder='big', signed=True)
            print(f"📷 Receiving {img_size} bytes...")

            # Get image data
            img_data = bytearray()
            while len(img_data) < img_size:
                chunk = self.socket.recv(min(4096, img_size - len(img_data)))
                if not chunk:
                    raise Exception("Connection closed")
                img_data.extend(chunk)

            # Save and show image info
            with open("unity_snapshot.jpg", "wb") as f:
                f.write(img_data)

            img = Image.open(io.BytesIO(img_data))
            print(
                f"🖼️  Received image: {img.size[0]}x{img.size[1]}, Mode: {img.mode}")
            return True

        except Exception as e:
            print(f"❌ Image test failed: {e}")
            return False

    def test_action(self, action=None):
        """Test sending an action to Unity"""
        try:
            if action is None:
                # Default action: small movement
                action = np.array([0.1, 0, 0, 0, 0, 0, 1, 0], dtype=np.float32)

            # Send action header (type=1)
            self.socket.sendall(b'\x01')

            # Send action size and data
            action_bytes = action.tobytes()
            self.socket.sendall(len(action_bytes).to_bytes(4, 'big'))
            self.socket.sendall(action_bytes)

            # Get ACK
            ack = self.socket.recv(1)
            print(f"🎯 Action sent: {action}")
            return ack == b'\x01'

        except Exception as e:
            print(f"❌ Action test failed: {e}")
            return False

    def close(self):
        if self.socket:
            self.socket.close()
            print("🔌 Disconnected from Unity")


if __name__ == "__main__":
    tester = UnityTester()

    try:
        if tester.connect():
            print("\n--- Testing Image Reception ---")
            if tester.test_image():
                print("\n--- Testing Action Sending ---")
                tester.test_action()
    finally:
        tester.close()

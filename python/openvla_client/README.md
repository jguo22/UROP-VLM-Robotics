# OpenVLA Unity Client

This folder contains the OpenVLA client implementation that connects to Unity for vision-language-action control.

## Contents

- **er5_client.py**: Main VLA client that runs OpenVLA model and communicates with Unity
- **UnityTCPConnection.py**: TCP connection handler for Unity communication
- **profiler.py**: Performance profiling utility
- **test_connection.py**: Connection testing utility
- **constants.py**: Shared constants for network protocols

## Quick Start

### Running the VLA Client

```bash
cd openvla_client
python er5_client.py
```

The client will:
1. Load the OpenVLA-7B model
2. Connect to Unity on port 5000
3. Request images from Unity
4. Run VLA inference
5. Send actions back to Unity

### Testing the Connection

Before running the full VLA client, test the Unity connection:

```bash
python test_connection.py
```

This will:
- Connect to Unity
- Request and save a test image
- Send a test action

## Protocol

The client uses a binary TCP protocol to communicate with Unity:

### Image Request
- Send: 1 byte (0x00) - Image request message type
- Receive: 4 bytes (big-endian int) - Image size
- Receive: N bytes - JPEG image data

### Action Send
- Send: 1 byte (0x01) - Action message type
- Send: 4 bytes (big-endian int) - Action data size
- Send: N bytes - Action as float32 array
- Receive: 1 byte - Acknowledgment (0x01 = success)

## Performance Profiling

The profiler automatically tracks:
- Image receive time
- Processor time
- Model inference time
- Action send time

Profiles are saved to `../profiles/` directory every time the client stops.

## Device Support

The client automatically detects and uses:
- **CUDA GPU**: Uses bfloat16 precision
- **Apple MPS (Metal)**: Uses float16 precision
- **CPU**: Uses float32 precision

## Dependencies

- torch
- transformers
- Pillow (PIL)
- numpy
- openvla-7b model (downloaded automatically)

## Configuration

Edit `constants.py` to change:
- Default host/port settings
- Protocol constants
- Timeout values

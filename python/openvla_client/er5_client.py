import torch
import traceback
import warnings
from transformers import AutoModelForVision2Seq, AutoProcessor
import time
from ur_analytic_ik import ur5

from profiler import Profiler
from UnityTCPConnection import UnityTCPConnection


# Suppress warnings
warnings.filterwarnings("ignore", message="`resume_download` is deprecated")

if __name__ == "__main__":
    # Initialize VLA model with optimizations
    print("Loading VLA model...")

    # Set device and dtype
    if torch.cuda.is_available():
        device = torch.device("cuda")
        dtype = torch.bfloat16
        print("Using CUDA GPU with bfloat16")
    elif hasattr(torch.backends, "mps") and torch.backends.mps.is_available():
        device = torch.device("mps")
        dtype = torch.float16  # MPS is optimized for float16
        print("Using MPS (Metal) with float16")
    else:
        device = torch.device("cpu")
        dtype = torch.float32
        print("Using CPU with float32")

    processor = AutoProcessor.from_pretrained(
        "openvla/openvla-7b", trust_remote_code=True)
    vla = AutoModelForVision2Seq.from_pretrained(
        "openvla/openvla-7b",
        torch_dtype=dtype,
        low_cpu_mem_usage=True,
        trust_remote_code=True,
    ).to(device)

    prompt = "In: What action should the robot take to move the orange gear on top of the gray box?\nOut:"

    print("done loading openvla model")

    # Main processing loop
    profiler = Profiler()
    while True:
        try:
            with UnityTCPConnection() as unity:
                frame_count = 0

                while True:
                    profiler.start()
                    frame_count += 1
                    print(f"\n--- Frame {frame_count} ---")

                    # 1. Get and preprocess image
                    image = unity.receive_image()
                    profiler.record("01_image_receive")

                    # 2. Process with VLA
                    with torch.no_grad():
                        # Process with model
                        inputs = processor(
                            prompt, image).to(
                            device=device, dtype=dtype)
                        profiler.record("02_processor")

                        action = vla.predict_action(
                            **inputs, unnorm_key="bridge_orig", do_sample=False)
                        print(action)
                        profiler.record("03_model_inference")

                    # 4. Send action back to Unity
                    success = unity.send_action(action)
                    profiler.record("04_send_action")
                    print(
                        f"Action sent: {'Success' if success else 'Failed'}")

                    # Update frame stats
                    profiler.end_frame()

        except ConnectionError as e:
            print(e)
            time.sleep(5)
        except KeyboardInterrupt:
            print("\nStopping...")
            profiler.save_profile()
            raise
        except Exception as e:
            print("\n=== Error ===")
            traceback.print_exc()
            profiler.save_profile()
            raise

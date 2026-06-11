"""Pixel-parity test for the two Unity image pipelines.

Question: does saving a Unity render as a PNG and reading it back in Python
(file flow, used by EpisodeRecorder -> ur5_unity_dataset_builder.py) yield the
same pixels as sending the render over the wire (wire flow, used by
OpenVLAController -> deploy.py)?

Both pipelines start from the same ObservationCapture buffer for a single render.
ImageFlowParityDump.cs writes both artifacts for one frame:

    <parity_dir>/file_flow.png    - EncodeToPNG() bytes (what the recorder saves)
    <parity_dir>/wire_flow.json   - json_numpy envelope the controller sends

This test loads each through the *exact* decode path its real consumer uses and
asserts the resulting (224, 224, 3) uint8 arrays are identical:

    file flow:  PIL.Image.open(...).convert("RGB")        (ur5_unity_dataset_builder.py)
    wire flow:  np.frombuffer(...).reshape(payload shape)  (offline_replay.decode_numpy)

Run as a script (auto-picks the newest Exports/parity/<ts>/ dump):
    python python/tests/test_image_flow_parity.py
    python python/tests/test_image_flow_parity.py <parity_dir>

Run under pytest (set PARITY_DIR to override auto-discovery):
    pytest python/tests/test_image_flow_parity.py
"""

import base64
import json
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image

WIDTH = 224
HEIGHT = 224
CHANNELS = 3

_EXPORTS_PARITY = (
    Path(__file__).resolve().parents[2] / "ur5_simulation" / "Exports" / "parity"
)


def _latest_parity_dir() -> Path:
    """Newest Exports/parity/<ts>/ dir that has both dump artifacts."""
    if not _EXPORTS_PARITY.is_dir():
        raise FileNotFoundError(
            f"No parity dumps found under {_EXPORTS_PARITY}. "
            "Run ImageFlowParityDump.DumpOnce() in Unity first."
        )
    candidates = [
        d
        for d in _EXPORTS_PARITY.iterdir()
        if d.is_dir()
        and (d / "file_flow.png").is_file()
        and (d / "wire_flow.json").is_file()
    ]
    if not candidates:
        raise FileNotFoundError(
            f"No complete dumps (file_flow.png + wire_flow.json) under {_EXPORTS_PARITY}."
        )
    return max(candidates, key=lambda d: d.stat().st_mtime)


def _resolve_parity_dir() -> Path:
    arg = os.environ.get("PARITY_DIR")
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        arg = sys.argv[1]
    return Path(arg) if arg else _latest_parity_dir()


def load_file_flow(parity_dir: Path) -> np.ndarray:
    """Decode file_flow.png exactly as ur5_unity_dataset_builder._parse_step does."""
    img = np.array(Image.open(parity_dir / "file_flow.png").convert("RGB"))
    assert img.shape == (HEIGHT, WIDTH, CHANNELS), f"unexpected PNG shape {img.shape}"
    return img


def load_wire_flow(parity_dir: Path) -> np.ndarray:
    """Decode wire_flow.json exactly as offline_replay.decode_numpy does."""
    payload = json.loads((parity_dir / "wire_flow.json").read_text())
    raw = base64.b64decode(payload["__numpy__"])
    return np.frombuffer(raw, dtype=np.dtype(payload["dtype"])).reshape(payload["shape"])


def compare(parity_dir: Path) -> tuple[int, int]:
    """Returns (num_mismatched_pixels, max_abs_channel_diff)."""
    file_img = load_file_flow(parity_dir)
    wire_img = load_wire_flow(parity_dir)
    diff = np.abs(file_img.astype(np.int16) - wire_img.astype(np.int16))
    mismatches = int(np.count_nonzero(diff.any(axis=-1)))
    return mismatches, int(diff.max())


def test_image_flow_parity() -> None:
    parity_dir = _resolve_parity_dir()
    mismatches, max_diff = compare(parity_dir)
    total = HEIGHT * WIDTH
    assert mismatches == 0, (
        f"{mismatches}/{total} pixels differ between file flow and wire flow "
        f"(max channel diff {max_diff}) in {parity_dir}. "
        "The PNG round-trip and the wire flip do not agree — check EncodeToPNG "
        "orientation vs OpenVLAController.FlipImageVertically."
    )


def main() -> int:
    parity_dir = _resolve_parity_dir()
    mismatches, max_diff = compare(parity_dir)
    total = HEIGHT * WIDTH
    print(f"parity dir : {parity_dir}")
    print(f"mismatched : {mismatches}/{total} pixels")
    print(f"max diff   : {max_diff} (per channel, 0-255)")
    if mismatches == 0:
        print("PASS — file flow and wire flow are pixel-identical.")
        return 0
    print("FAIL — pipelines disagree.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

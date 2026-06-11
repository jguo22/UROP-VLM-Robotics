"""Definitive format/orientation/channel parity test for the two image flows.

Comparing two real Unity renders only catches a mismatch when the scene is
asymmetric, and at 224x224 a height/width transpose doesn't even change the array
shape -- so it can silently hide a transpose or channel swap. This test removes
that ambiguity: ImageFlowParityDump.DumpSyntheticParity() runs a *known
positional pattern* through the exact two code paths under test and we check each
decoded array against the absolute expected pattern, not just against each other.

Intended top-down image P (what a correct decode must produce):

    P[row, col] = (R = col, G = row, B = 255)      (col, row in 0..223)

R is a purely horizontal gradient, G a purely vertical one, and B a constant
sentinel, so each axis and channel is independently identifiable:

    vertical flip      -> rows (G) reversed
    horizontal flip    -> cols (R) reversed
    H/W transpose      -> R and G gradients swap roles
    channel swap       -> the 255 sentinel moves out of B

Each lands at a known location, so a failure pinpoints the exact defect.

The dump writes, into Exports/parity/synthetic_<ts>/:
    file_flow.png   - EncodeToPNG bytes (recorder / dataset-builder path)
    wire_flow.json  - the real json_numpy envelope {__numpy__ base64, dtype, shape}
                      that OpenVLAController sends (controller / deploy.py path)
    meta.json       - {type, width, height, channels, pattern}

Decode paths mirror the real consumers exactly:
    file flow:  PIL.Image.open(...).convert("RGB")        (ur5_unity_dataset_builder.py)
    wire flow:  np.frombuffer(...).reshape(payload shape)  (offline_replay.decode_numpy)

Run as a script (auto-picks the newest synthetic dump):
    python python/tests/test_image_format_parity.py [<dump_dir>]

Run under pytest (set PARITY_DIR to override auto-discovery):
    pytest python/tests/test_image_format_parity.py
"""

import base64
import json
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image

_EXPORTS_PARITY = (
    Path(__file__).resolve().parents[2] / "ur5_simulation" / "Exports" / "parity"
)


def _latest_synthetic_dir() -> Path:
    if not _EXPORTS_PARITY.is_dir():
        raise FileNotFoundError(
            f"No parity dumps under {_EXPORTS_PARITY}. "
            "Run ImageFlowParityDump.DumpSyntheticParity() in Unity first."
        )
    candidates = [
        d
        for d in _EXPORTS_PARITY.iterdir()
        if d.is_dir() and (d / "meta.json").is_file()
        and (d / "file_flow.png").is_file() and (d / "wire_flow.json").is_file()
    ]
    candidates = [
        d for d in candidates
        if json.loads((d / "meta.json").read_text()).get("type")
        == "synthetic_positional"
    ]
    if not candidates:
        raise FileNotFoundError(
            f"No synthetic_positional dumps under {_EXPORTS_PARITY}."
        )
    return max(candidates, key=lambda d: d.stat().st_mtime)


def _resolve_dir() -> Path:
    arg = os.environ.get("PARITY_DIR")
    if len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
        arg = sys.argv[1]
    return Path(arg) if arg else _latest_synthetic_dir()


def expected_pattern(h: int, w: int) -> np.ndarray:
    """P[row, col] = (col % 256, row % 256, 255); mirrors the C# dumper."""
    cols = np.broadcast_to(np.arange(w) % 256, (h, w))
    rows = np.broadcast_to((np.arange(h) % 256)[:, None], (h, w))
    out = np.empty((h, w, 3), dtype=np.uint8)
    out[..., 0] = cols
    out[..., 1] = rows
    out[..., 2] = 255
    return out


def load_file_flow(d: Path) -> np.ndarray:
    """Decode file_flow.png as ur5_unity_dataset_builder._parse_step does."""
    return np.array(Image.open(d / "file_flow.png").convert("RGB"))


def load_wire_flow(d: Path) -> np.ndarray:
    """Decode wire_flow.json as offline_replay.decode_numpy does.

    Uses the payload's own self-declared dtype/shape (not meta.json), so a wrong
    shape declaration like [W, H, 3] is exercised and caught here too.
    """
    payload = json.loads((d / "wire_flow.json").read_text())
    raw = base64.b64decode(payload["__numpy__"])
    return np.frombuffer(raw, dtype=np.dtype(payload["dtype"])).reshape(payload["shape"])


def diagnose(actual: np.ndarray, expected: np.ndarray) -> str:
    """Name the geometric/channel transform that maps expected -> actual, if any."""
    if actual.shape != expected.shape:
        return f"shape mismatch: got {actual.shape}, expected {expected.shape}"
    hypotheses = {
        "vertical flip (rows reversed)": expected[::-1, :, :],
        "horizontal flip (cols reversed)": expected[:, ::-1, :],
        "180 rotation": expected[::-1, ::-1, :],
    }
    if expected.shape[0] == expected.shape[1]:
        hypotheses["H/W transpose"] = np.transpose(expected, (1, 0, 2))
        hypotheses["transpose + vertical flip (rot90)"] = np.transpose(
            expected, (1, 0, 2))[::-1, :, :]
    # Channel permutations (e.g. RGB vs BGR).
    perms = {"RBG": (0, 2, 1), "GRB": (1, 0, 2), "GBR": (1, 2, 0),
             "BRG": (2, 0, 1), "BGR (R/B swap)": (2, 1, 0)}
    for name, p in perms.items():
        hypotheses[f"channel order {name}"] = expected[..., list(p)]
    for name, cand in hypotheses.items():
        if np.array_equal(actual, cand):
            return f"looks like: {name}"
    n_bad = int(np.count_nonzero(np.any(actual != expected, axis=-1)))
    return (f"unrecognized: {n_bad}/{expected.shape[0]*expected.shape[1]} pixels "
            f"differ (max channel diff {int(np.abs(actual.astype(int)-expected).max())})")


def check(d: Path) -> list[str]:
    """Returns a list of failure messages; empty list means definitive PASS."""
    meta = json.loads((d / "meta.json").read_text())
    h, w = int(meta["height"]), int(meta["width"])
    expected = expected_pattern(h, w)

    failures: list[str] = []

    file_img = load_file_flow(d)
    if not np.array_equal(file_img, expected):
        failures.append(f"file flow (PNG) != expected — {diagnose(file_img, expected)}")

    wire_img = load_wire_flow(d)
    if not np.array_equal(wire_img, expected):
        failures.append(f"wire flow (json) != expected — {diagnose(wire_img, expected)}")

    # Redundant if both match expected, but pinpoints a flow-vs-flow drift if one
    # side silently changed shape/format.
    if file_img.shape == wire_img.shape and not np.array_equal(file_img, wire_img):
        failures.append("file flow and wire flow disagree with each other")

    return failures


def test_image_format_parity() -> None:
    d = _resolve_dir()
    failures = check(d)
    assert not failures, (
        f"Image format/orientation mismatch in {d}:\n  " + "\n  ".join(failures)
    )


def main() -> int:
    d = _resolve_dir()
    meta = json.loads((d / "meta.json").read_text())
    print(f"dump dir : {d}")
    print(f"pattern  : {meta.get('pattern')}  ({meta['width']}x{meta['height']})")
    failures = check(d)
    if not failures:
        print("PASS — both flows decode to the exact expected pattern. "
              "No orientation, H/W, or channel mismatch.")
        return 0
    print("FAIL — definitive mismatch detected:")
    for f in failures:
        print(f"  - {f}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

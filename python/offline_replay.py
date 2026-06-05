"""Offline replay test for the deployed OpenVLA-OFT server.

Feeds recorded episodes (poses.csv + images/) to the server frame-by-frame,
in the same wire format OpenVLAController.cs uses, and compares the server's
returned actions to the ground-truth actions reconstructed from the CSV.

Interpretation:
  - Small per-dim error  -> the fine-tuned checkpoint learned the training
                            distribution. Any failure at runtime is a deploy-
                            time issue (camera, scene, init pose, etc.).
  - Large per-dim error  -> the fine-tune did not converge on this data, or
                            the unnorm_key on the server is wrong.

The target action is reconstructed to match the model's actual training target:
   ur5_unity_dataset_builder.py: [Δpos, Δrotvec, suction(-1/+1), terminate]
   ur5_unity_dataset_transform : drop terminate, map suction -> gripper {0,1}
                                  (suction on/+1 -> 0 closed, off/-1 -> 1 open)

Usage:
  # Whole dataset (parent dir containing recording_*/ subdirs):
  python offline_replay.py <dataset_dir> [--server URL] [--max-episodes N]
                                          [--max-frames N] [--csv OUT.csv]

  # Single episode:
  python offline_replay.py <dataset_dir>/recording_<ts>
"""

import argparse
import base64
import csv
import json
from pathlib import Path

import numpy as np
import requests
from PIL import Image
from scipy.spatial.transform import Rotation

ACTION_DIMS = ["dpos_x", "dpos_y", "dpos_z", "drot_x", "drot_y", "drot_z", "gripper"]


def encode_image(img: np.ndarray) -> dict:
    h, w, _ = img.shape
    return {
        "__numpy__": base64.b64encode(img.tobytes()).decode("ascii"),
        "dtype": "uint8",
        "shape": [w, h, 3],
    }


def decode_numpy(d: dict) -> np.ndarray:
    raw = base64.b64decode(d["__numpy__"])
    return np.frombuffer(raw, dtype=np.dtype(d["dtype"])).reshape(d["shape"])


def build_state(row: dict) -> list[float]:
    joints_rad = [float(row[f"joint_{j}"]) * np.pi / 180.0 for j in range(6)]
    suction = 1.0 if row["suctionOn"] == "True" else 0.0
    return joints_rad + [0.0, suction]


def build_recorded_action(row: dict) -> np.ndarray:
    delta_pos = np.array(
        [float(row["delta_pos_x"]), float(row["delta_pos_y"]), float(row["delta_pos_z"])],
        dtype=np.float64,
    )
    quat_xyzw = np.array(
        [
            float(row["delta_rot_x"]),
            float(row["delta_rot_y"]),
            float(row["delta_rot_z"]),
            float(row["delta_rot_w"]),
        ],
        dtype=np.float64,
    )
    rotvec = Rotation.from_quat(quat_xyzw).as_rotvec()
    suction_on = row["suctionOn"] == "True"
    gripper = 0.0 if suction_on else 1.0
    return np.concatenate([delta_pos, rotvec, [gripper]]).astype(np.float32)


def parse_response(body_text: str) -> np.ndarray:
    parsed = json.loads(body_text)
    if isinstance(parsed, list):
        parsed = parsed[0]
    return decode_numpy(parsed)


def replay_episode(
    recording_dir: Path,
    server_url: str,
    instruction: str,
    max_frames: int | None,
    print_steps: bool = False,
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    """Returns (preds, recs, frame_indices). preds and recs are (N, 7)."""
    poses_path = recording_dir / "poses.csv"
    images_dir = recording_dir / "images"

    with open(poses_path) as f:
        rows = list(csv.DictReader(f))
    # Drop terminal row: its recorded delta is zero (no successor frame).
    rows = [r for r in rows if int(r["is_last"]) == 0]
    if max_frames is not None:
        rows = rows[:max_frames]

    preds: list[np.ndarray] = []
    recs: list[np.ndarray] = []
    indices: list[int] = []

    # Open-loop chunked replay: query at frame i, then roll the whole predicted
    # action chunk (the server is configured for chunks of 8) against the next
    # consecutive recorded frames, then jump ahead by the chunk length and
    # re-query. This mirrors how the chunk is executed at deploy time.
    i = 0
    while i < len(rows):
        row = rows[i]
        frame_idx = int(row["frame"])
        img_path = images_dir / f"{frame_idx:06d}.png"
        img = np.asarray(Image.open(img_path).convert("RGB"))

        payload = {
            "full_image": encode_image(img),
            "state": build_state(row),
            "instruction": instruction,
        }
        resp = requests.post(server_url, json=payload, timeout=30)
        resp.raise_for_status()

        action_chunk = parse_response(resp.text)
        if action_chunk.ndim == 1:
            action_chunk = action_chunk[None, :]
        chunk_len = action_chunk.shape[0]

        if print_steps:
            print(f"  query @ frame {frame_idx:>4}  (chunk_len={chunk_len})")

        # Compare each action in the chunk to the recorded action at the
        # corresponding successor frame; stop early at the episode end.
        for k in range(chunk_len):
            if i + k >= len(rows):
                break
            step_row = rows[i + k]
            step_frame = int(step_row["frame"])
            predicted = action_chunk[k].astype(np.float64)
            recorded = build_recorded_action(step_row).astype(np.float64)

            if print_steps:
                fmt = lambda a: "[" + " ".join(f"{v:+7.4f}" for v in a) + "]"
                err = np.abs(predicted - recorded)
                print(f"    frame {step_frame:>4} (chunk[{k}])  "
                      f"|err|_2={np.linalg.norm(err):.4f}")
                print(f"      pred {fmt(predicted)}")
                print(f"      rec  {fmt(recorded)}")
                print(f"      err  {fmt(err)}")

            preds.append(predicted)
            recs.append(recorded)
            indices.append(step_frame)

        i += chunk_len

    if not preds:
        empty = np.zeros((0, 7), dtype=np.float64)
        return empty, empty, []
    return np.stack(preds), np.stack(recs), indices


def find_episodes(root: Path) -> list[Path]:
    # If root itself is a recording dir, use it. Otherwise glob children.
    if (root / "poses.csv").is_file() and (root / "images").is_dir():
        return [root]
    return sorted(
        d for d in root.glob("recording_*")
        if (d / "poses.csv").is_file() and (d / "images").is_dir()
    )


def print_per_dim_table(preds: np.ndarray, recs: np.ndarray) -> None:
    errs = preds - recs
    print(f"{'dim':<8} {'mean err':>10} {'std err':>10} {'max |err|':>10}  "
          f"{'rec range':>22}  {'pred range':>22}")
    for i, name in enumerate(ACTION_DIMS):
        rec_min, rec_max = recs[:, i].min(), recs[:, i].max()
        pred_min, pred_max = preds[:, i].min(), preds[:, i].max()
        print(
            f"{name:<8} {errs[:, i].mean():>10.5f} {errs[:, i].std():>10.5f} "
            f"{np.abs(errs[:, i]).max():>10.5f}  "
            f"[{rec_min:>8.4f}, {rec_max:>8.4f}]  "
            f"[{pred_min:>8.4f}, {pred_max:>8.4f}]"
        )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "dataset_dir",
        type=Path,
        help="Either a dataset root (contains recording_*/) or a single recording_*/ dir.",
    )
    ap.add_argument("--server", default="http://localhost:8777/act")
    ap.add_argument(
        "--instruction",
        default="put blue, red, and green gears into planetary gearbox",
    )
    ap.add_argument("--max-episodes", type=int, default=None)
    ap.add_argument("--max-frames", type=int, default=None,
                    help="Per-episode frame cap (for quick smoke tests).")
    ap.add_argument("--print-steps", action="store_true",
                    help="Print predicted vs recorded action for every frame.")
    ap.add_argument("--csv", type=Path, default=None)
    args = ap.parse_args()

    episodes = find_episodes(args.dataset_dir)
    if not episodes:
        raise SystemExit(f"No episodes found under {args.dataset_dir}")
    if args.max_episodes is not None:
        episodes = episodes[: args.max_episodes]

    print(f"Root         : {args.dataset_dir}")
    print(f"Server       : {args.server}")
    print(f"Instruction  : {args.instruction!r}")
    print(f"Episodes     : {len(episodes)}")
    print()

    all_preds: list[np.ndarray] = []
    all_recs: list[np.ndarray] = []
    rows_for_csv: list[tuple[str, int, np.ndarray, np.ndarray]] = []

    for ep in episodes:
        if args.print_steps:
            print(f"--- {ep.name}  (dims: {' '.join(ACTION_DIMS)}) ---")
        preds, recs, indices = replay_episode(
            ep, args.server, args.instruction, args.max_frames, args.print_steps
        )
        if len(preds) == 0:
            print(f"  {ep.name:<40}  (empty)")
            continue

        errs = preds - recs
        mean_l2 = float(np.linalg.norm(errs, axis=1).mean())
        gripper_acc = ((preds[:, 6] < 0.5) == (recs[:, 6] < 0.5)).mean() * 100.0
        print(f"  {ep.name:<40}  frames={len(preds):>4}  "
              f"mean|err|_2={mean_l2:.5f}  gripper={gripper_acc:5.1f}%")

        all_preds.append(preds)
        all_recs.append(recs)
        if args.csv is not None:
            for idx, p, r in zip(indices, preds, recs):
                rows_for_csv.append((ep.name, idx, p, r))

    if not all_preds:
        raise SystemExit("All episodes were empty.")

    preds = np.concatenate(all_preds, axis=0)
    recs = np.concatenate(all_recs, axis=0)
    errs = preds - recs

    print()
    print(f"==== Aggregate over {len(all_preds)} episodes / {len(preds)} frames ====")
    print_per_dim_table(preds, recs)
    print()
    print(f"Mean |err|_2 (chunk[0])    : {np.linalg.norm(errs, axis=1).mean():.5f}")
    gripper_acc = ((preds[:, 6] < 0.5) == (recs[:, 6] < 0.5)).mean() * 100.0
    n_closed = int((recs[:, 6] < 0.5).sum())
    print(f"Gripper-threshold agreement: {gripper_acc:.1f}%  "
          f"(recorded closed/grasping on {n_closed}/{len(recs)} frames)")

    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                ["episode", "frame"]
                + [f"pred_{n}" for n in ACTION_DIMS]
                + [f"rec_{n}" for n in ACTION_DIMS]
            )
            for ep_name, idx, p, r in rows_for_csv:
                w.writerow([ep_name, idx, *p.tolist(), *r.tolist()])
        print(f"\nPer-frame log written to {args.csv}")


if __name__ == "__main__":
    main()

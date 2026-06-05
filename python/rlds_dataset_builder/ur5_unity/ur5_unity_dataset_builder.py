"""TFDS builder for UR5 Unity episodes recorded by EpisodeRecorder.cs.

Input: DATASET_DIR/<episode>/{poses.csv, images/NNNNNN.png}
       DATASET_DIR is the dataset root; every immediate child directory is treated
       as one episode (those missing poses.csv or images/ are skipped).
       (see EpisodeRecorder.cs header for poses.csv column spec).

Coordinate frame is preserved: Unity world (left-handed, Y-up, Z-forward, meters,
Hamiltonian xyzw quaternions). No axis remap or handedness flip is applied.

Per-step output (one entry in `steps`):
  observation.image — uint8 RGB, asserted to already be 224x224.
  observation.state — float32[7] = [joint_0..5 RAD, suction state].
                      Joint angles are converted deg → rad here; CSV stores degrees.
                      suction state: 1.0 = suction on, 0.0 = suction off.
  action            — float32[7] = [Δpos_xyz (world, m),
                                    Δrot_rotvec_xyz (axis-angle, rad),
                                    suction_cmd].
                      suction_cmd: +1.0 = suction on, -1.0 = suction off.
                      (Note the action and state use different off-values:
                       action off = -1.0, state off = 0.0; both use on = 1.0.)
                      Δrot comes from CSV's quaternion delta via
                      scipy Rotation.from_quat(xyzw).as_rotvec().
                      Episode-end is carried by the separate is_terminal field,
                      not the action vector.
  The following are derived from the step's position in the episode, not read
  from poses.csv:
  is_first    — True on the first step (step_idx == 0).
  is_last     — True on the last step (step_idx == num_steps - 1).
  is_terminal — same as is_last (demos terminate on the last step).
  reward      — 1.0 on the last step, 0.0 otherwise.
  discount    — fixed at 1.0.
  language_instruction — hard-coded LANGUAGE_INSTRUCTION (not stored in CSV).
  language_embedding   — USE-large embedding of LANGUAGE_INSTRUCTION.

Split: first TRAIN_SPLIT (90%) of sorted child dirs of DATASET_DIR → train, rest → val.
Env: override input dir with DATASET_DIR=...
"""

from typing import Iterator, Tuple, Any

import csv
import numpy as np
import os
import tensorflow_datasets as tfds
import tensorflow_hub as hub
from PIL import Image
from scipy.spatial.transform import Rotation


_DEFAULT_DATASET_DIR = os.path.normpath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "ur5_simulation",
        "Exports",
        "overfit_dataset"))
DATASET_DIR = os.environ.get("DATASET_DIR", _DEFAULT_DATASET_DIR)
TRAIN_SPLIT = 0.9
LANGUAGE_INSTRUCTION = "put blue, red, and green gears into planetary gearbox"


class Ur5Unity(tfds.core.GeneratorBasedBuilder):
    """DatasetBuilder for UR5 Unity simulation pick-and-place episodes."""

    VERSION = tfds.core.Version("1.0.0")
    RELEASE_NOTES = {"1.0.0": "Initial release."}

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._embed = hub.load(
            "https://tfhub.dev/google/universal-sentence-encoder-large/5")

    def _info(self) -> tfds.core.DatasetInfo:
        return self.dataset_info_from_configs(
            features=tfds.features.FeaturesDict({
                "steps": tfds.features.Dataset({
                    "observation": tfds.features.FeaturesDict({
                        "image": tfds.features.Image(
                            shape=(224, 224, 3),
                            dtype=np.uint8,
                            encoding_format="png",
                            doc="Main camera RGB observation (224x224).",
                        ),
                        "state": tfds.features.Tensor(
                            shape=(7,),
                            dtype=np.float32,
                            doc="Robot state: [6x joint angles in radians, "
                                "1x suction state (1.0 = on, 0.0 = off)].",
                        ),
                    }),
                    "action": tfds.features.Tensor(
                        shape=(7,),
                        dtype=np.float32,
                        doc="Robot action: [3x delta EE pos, 3x delta EE rot (rotation vector), "
                            "1x suction command (+1.0 = on, -1.0 = off)].",
                    ),
                    "discount": tfds.features.Scalar(dtype=np.float32, doc="Discount if provided, default to 1."),
                    "reward": tfds.features.Scalar(dtype=np.float32, doc="Reward if provided, 1 on final step for demos."),
                    "is_first": tfds.features.Scalar(dtype=np.bool_, doc="True on first step of the episode."),
                    "is_last": tfds.features.Scalar(dtype=np.bool_, doc="True on last step of the episode."),
                    "is_terminal": tfds.features.Scalar(dtype=np.bool_, doc="True on last step of the episode if it is a terminal step, True for demos."),
                    "language_instruction": tfds.features.Text(doc="Language Instruction."),
                    "language_embedding": tfds.features.Tensor(
                        shape=(512,),
                        dtype=np.float32,
                        doc="Kona language embedding. See https://tfhub.dev/google/universal-sentence-encoder-large/5",
                    ),
                }),
                "episode_metadata": tfds.features.FeaturesDict({
                    "file_path": tfds.features.Text(doc="Path to the original episode folder."),
                }),
            })
        )

    def _split_generators(self, dl_manager: tfds.download.DownloadManager):
        # DATASET_DIR is the dataset root; each immediate child directory is one
        # recorded episode (containing poses.csv + an images/ folder). Take every
        # child dir, not just recording_*-named ones.
        episode_dirs = sorted(
            os.path.join(DATASET_DIR, name)
            for name in os.listdir(DATASET_DIR)
            if os.path.isdir(os.path.join(DATASET_DIR, name))
        )
        # Keep only valid episodes (skips any stray non-episode child dirs).
        episode_dirs = [
            d for d in episode_dirs
            if os.path.isfile(os.path.join(d, "poses.csv"))
            and os.path.isdir(os.path.join(d, "images"))
        ]
        language_embedding = self._embed([LANGUAGE_INSTRUCTION])[0].numpy()
        split_idx = int(len(episode_dirs) * TRAIN_SPLIT)
        return {
            "train": self._generate_examples(episode_dirs[:split_idx], language_embedding),
            "val": self._generate_examples(episode_dirs[split_idx:], language_embedding),
        }

    def _generate_examples(self, episode_dirs,
                           language_embedding) -> Iterator[Tuple[str, Any]]:
        for episode_dir in episode_dirs:
            result = self._parse_episode(episode_dir, language_embedding)
            if result is not None:
                yield result

    def _parse_episode(self, episode_dir, language_embedding):
        poses_path = os.path.join(episode_dir, "poses.csv")
        images_dir = os.path.join(episode_dir, "images")

        with open(poses_path, "r") as f:
            rows = list(csv.DictReader(f))

        if not rows:
            return None

        num_steps = len(rows)
        episode = [
            self._parse_step(
                row,
                step_idx,
                num_steps,
                images_dir,
                language_embedding) for step_idx, row in enumerate(rows)]
        return episode_dir, {
            "steps": episode,
            "episode_metadata": {"file_path": episode_dir},
        }

    def _parse_step(self, row, step_idx, num_steps, images_dir,
                    language_embedding):
        image_path = os.path.join(images_dir, f"{int(row['frame']):06d}.png")
        image = np.array(Image.open(image_path).convert("RGB"))
        assert image.shape == (224, 224, 3), (
            f"Expected 224x224 RGB image, got {image.shape} at {image_path}")
        # RLDS demo bookkeeping is derived from the step's position in the
        # episode, not read from poses.csv: first/last by index, demos are
        # terminal on the last step with reward 1 there and 0 elsewhere,
        # discount fixed at 1.
        is_first = step_idx == 0
        is_last = step_idx == num_steps - 1
        return {
            "observation": {
                "image": image,
                "state": self._parse_state(row),
            },
            "action": self._parse_action(row),
            "discount": 1.0,
            "reward": 1.0 if is_last else 0.0,
            "is_first": is_first,
            "is_last": is_last,
            "is_terminal": is_last,
            "language_instruction": LANGUAGE_INSTRUCTION,
            "language_embedding": language_embedding,
        }

    def _parse_state(self, row) -> np.ndarray:
        joint_angles_rad = np.array(
            [float(row[f"joint_{j}"]) * np.pi / 180.0 for j in range(6)],
            dtype=np.float32,
        )
        # State suction: 1.0 = suction on, 0.0 = suction off.
        suction_on = 1.0 if row["suctionOn"] == "True" else 0.0
        return np.concatenate(
            [joint_angles_rad, [suction_on]]).astype(np.float32)

    def _parse_action(self, row) -> np.ndarray:
        delta_pos = np.array([float(row["delta_pos_x"]), float(
            row["delta_pos_y"]), float(row["delta_pos_z"])], dtype=np.float32, )
        delta_quat_xyzw = np.array([
            float(row["delta_rot_x"]),
            float(row["delta_rot_y"]),
            float(row["delta_rot_z"]),
            float(row["delta_rot_w"]),
        ])
        delta_rotvec = Rotation.from_quat(
            delta_quat_xyzw).as_rotvec().astype(np.float32)
        # Action suction command: +1.0 = suction on, -1.0 = suction off.
        suction_cmd = 1.0 if row["suctionOn"] == "True" else -1.0
        return np.concatenate(
            [delta_pos, delta_rotvec, [suction_cmd]]).astype(np.float32)

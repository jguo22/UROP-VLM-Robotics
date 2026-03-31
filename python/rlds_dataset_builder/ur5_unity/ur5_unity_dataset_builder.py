from typing import Iterator, Tuple, Any

import csv
import glob
import numpy as np
import os
import tensorflow_datasets as tfds
import tensorflow_hub as hub
from PIL import Image
from scipy.spatial.transform import Rotation


# Path to the Unity exports dataset directory
DATASET_DIR = os.path.normpath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "ur5_simulation",
        "Exports",
        "dataset"))
TRAIN_SPLIT = 0.9  # fraction of episodes used for training


class Ur5Unity(tfds.core.GeneratorBasedBuilder):
    """DatasetBuilder for UR5 Unity simulation pick-and-place episodes."""

    VERSION = tfds.core.Version("1.0.0")
    RELEASE_NOTES = {
        "1.0.0": "Initial release.",
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._embed = hub.load(
            "https://tfhub.dev/google/universal-sentence-encoder-large/5")

    def _info(self) -> tfds.core.DatasetInfo:
        return self.dataset_info_from_configs(
            features=tfds.features.FeaturesDict(
                {
                    "steps": tfds.features.Dataset(
                        {
                            "observation": tfds.features.FeaturesDict(
                                {
                                    "image": tfds.features.Image(
                                        shape=(None, None, 3),
                                        dtype=np.uint8,
                                        encoding_format="png",
                                        doc="Main camera RGB observation.",
                                    ),
                                    "state": tfds.features.Tensor(
                                        shape=(7,),
                                        dtype=np.float32,
                                        doc="Robot state: [6x joint angles in radians, 1x suction on/off].",
                                    ),
                                }
                            ),
                            "action": tfds.features.Tensor(
                                shape=(8,),
                                dtype=np.float32,
                                doc="Robot action: [3x delta EE pos, 3x delta EE rot (rotation vector), "
                                "1x suction command (-1/1), 1x terminate episode].",
                            ),
                            "discount": tfds.features.Scalar(
                                dtype=np.float32,
                                doc="Discount if provided, default to 1.",
                            ),
                            "reward": tfds.features.Scalar(
                                dtype=np.float32,
                                doc="Reward if provided, 1 on final step for demos.",
                            ),
                            "is_first": tfds.features.Scalar(
                                dtype=np.bool_,
                                doc="True on first step of the episode.",
                            ),
                            "is_last": tfds.features.Scalar(
                                dtype=np.bool_,
                                doc="True on last step of the episode.",
                            ),
                            "is_terminal": tfds.features.Scalar(
                                dtype=np.bool_,
                                doc="True on last step of the episode if it is a terminal step, True for demos.",
                            ),
                            "language_instruction": tfds.features.Text(
                                doc="Language Instruction.",
                            ),
                            "language_embedding": tfds.features.Tensor(
                                shape=(512,),
                                dtype=np.float32,
                                doc="Kona language embedding. "
                                "See https://tfhub.dev/google/universal-sentence-encoder-large/5",
                            ),
                        }
                    ),
                    "episode_metadata": tfds.features.FeaturesDict(
                        {
                            "file_path": tfds.features.Text(
                                doc="Path to the original episode folder.",
                            ),
                        }
                    ),
                }
            )
        )

    def _split_generators(self, dl_manager: tfds.download.DownloadManager):
        episode_dirs = sorted(
            glob.glob(
                os.path.join(
                    DATASET_DIR,
                    "recording_*")))
        # Filter to only directories that have poses.csv and images/
        episode_dirs = [
            d for d in episode_dirs
            if os.path.isfile(os.path.join(d, "poses.csv"))
            and os.path.isdir(os.path.join(d, "images"))
        ]
        split_idx = int(len(episode_dirs) * TRAIN_SPLIT)
        return {
            "train": self._generate_examples(episode_dirs[:split_idx]),
            "val": self._generate_examples(episode_dirs[split_idx:]),
        }

    def _generate_examples(self, episode_dirs) -> Iterator[Tuple[str, Any]]:
        def _parse_example(episode_dir):
            poses_path = os.path.join(episode_dir, "poses.csv")
            images_dir = os.path.join(episode_dir, "images")

            rows = []
            with open(poses_path, "r") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    rows.append(row)

            if len(rows) == 0:
                return None

            # Language instruction — empty for now, populate before training
            language_instruction = ""
            language_embedding = self._embed([language_instruction])[0].numpy()

            episode = []
            for row in rows:
                frame_idx = int(row["frame"])

                # Load image
                img_path = os.path.join(images_dir, f"{frame_idx:06d}.png")
                image = np.array(Image.open(img_path).convert("RGB"))

                # State: 6 joint angles (degrees -> radians) + suction (0/1)
                joint_angles_rad = np.array(
                    [float(row[f"joint_{j}"]) * np.pi / 180.0 for j in range(6)],
                    dtype=np.float32,
                )
                suction_on = 1.0 if row["suctionOn"] == "True" else 0.0
                state = np.concatenate(
                    [joint_angles_rad, [suction_on]]).astype(np.float32)

                # Action: delta pos (3) + delta rot as rotation vector (3) +
                # suction cmd (1) + terminate (1)
                delta_pos = np.array([float(row["delta_pos_x"]), float(
                    row["delta_pos_y"]), float(row["delta_pos_z"])], dtype=np.float32, )
                delta_quat_xyzw = np.array(
                    [
                        float(row["delta_rot_x"]),
                        float(row["delta_rot_y"]),
                        float(row["delta_rot_z"]),
                        float(row["delta_rot_w"]),
                    ]
                )
                # scipy uses scalar-last (xyzw) convention
                delta_rotvec = Rotation.from_quat(
                    delta_quat_xyzw).as_rotvec().astype(np.float32)

                suction_cmd = 1.0 if row["suctionOn"] == "True" else -1.0
                terminate = float(row["is_terminal"])

                action = np.concatenate([delta_pos, delta_rotvec, [suction_cmd], [
                                        terminate]]).astype(np.float32)

                episode.append(
                    {
                        "observation": {
                            "image": image,
                            "state": state,
                        },
                        "action": action,
                        "discount": float(row["discount"]),
                        "reward": float(row["reward"]),
                        "is_first": int(row["is_first"]) == 1,
                        "is_last": int(row["is_last"]) == 1,
                        "is_terminal": int(row["is_terminal"]) == 1,
                        "language_instruction": language_instruction,
                        "language_embedding": language_embedding,
                    }
                )

            sample = {
                "steps": episode,
                "episode_metadata": {
                    "file_path": episode_dir,
                },
            }
            return episode_dir, sample

        for episode_dir in episode_dirs:
            result = _parse_example(episode_dir)
            if result is not None:
                yield result

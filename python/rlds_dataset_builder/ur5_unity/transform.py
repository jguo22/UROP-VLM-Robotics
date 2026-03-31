from typing import Any, Dict
import numpy as np
from PIL import Image


def transform_step(step: Dict[str, Any]) -> Dict[str, Any]:
    """Maps step from ur5_unity_dataset to the standard 8D action target config.
    Input is dict of numpy arrays."""
    img = Image.fromarray(step['observation']['image']).resize(
        (128, 128), Image.Resampling.LANCZOS)
    transformed_step = {
        'observation': {
            'image': np.array(img),
        },
        # Action is already 8D: [3x delta pos, 3x delta rot vec, 1x suction, 1x terminate]
        'action': step['action'].astype(np.float32),
    }

    for copy_key in ['discount', 'reward', 'is_first', 'is_last', 'is_terminal',
                     'language_instruction', 'language_embedding']:
        transformed_step[copy_key] = step[copy_key]

    return transformed_step

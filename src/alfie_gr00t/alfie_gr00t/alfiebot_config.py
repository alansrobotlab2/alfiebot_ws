from gr00t.configs.data.embodiment_configs import register_modality_config
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import (
    ActionConfig,
    ActionFormat,
    ActionRepresentation,
    ActionType,
    ModalityConfig,
)


alfiebot_config = {
    "video": ModalityConfig(
        delta_indices=[0],
        modality_keys=[
            "left_wide",
            "right_wide",
            "left_center",
            "right_center",
        ],
    ),
    "state": ModalityConfig(
        delta_indices=[0],
        modality_keys=[
            "base",
            "left_arm",
            "left_hand",
            "right_arm",
            "right_hand",
            "head",
        ],
    ),
    "action": ModalityConfig(
        delta_indices=[
            0,
            1,
            2,
            3,
            4,
            5,
            6,
            7,
            8,
            9,
            10,
            11,
            12,
            13,
            14,
            15,
        ],
        modality_keys=[
            "base",
            "left_arm",
            "left_hand",
            "right_arm",
            "right_hand",
            "neck",
        ],
        action_configs=[
            ActionConfig(
                rep=ActionRepresentation.RELATIVE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
        ],
    ),
    "language": ModalityConfig(
        delta_indices=[0],
        modality_keys=["human.task_description"],
    ),
}

register_modality_config(alfiebot_config, embodiment_tag=EmbodimentTag.NEW_EMBODIMENT)

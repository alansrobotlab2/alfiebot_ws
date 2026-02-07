from gr00t.configs.data.embodiment_configs import register_modality_config
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import (
    ActionConfig,
    ActionFormat,
    ActionRepresentation,
    ActionType,
    ModalityConfig,
)


# Modality config for Alfiebot GR00T fine-tuning.
# Used by training scripts and rosbag_to_groot converter.
# NOT imported by the inference server at runtime — the server loads
# config directly from the checkpoint's processor_config.json.
#
# All action_configs are ABSOLUTE to avoid compounding error in
# closed-loop execution (RELATIVE deltas accumulate prediction noise).
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
            "back",
            "left_arm",
            "left_hand",
            "right_arm",
            "right_hand",
            "head",
        ],
    ),
    "action": ModalityConfig(
        delta_indices=list(range(16)),
        modality_keys=[
            "base",
            "back",
            "left_arm",
            "left_hand",
            "right_arm",
            "right_hand",
            "head",
        ],
        action_configs=[
            ActionConfig(  # base — velocity commands (m/s, rad/s)
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # back — position (meters)
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # left_arm — joint positions (rad)
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # left_hand — gripper position (rad)
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # right_arm — joint positions (rad)
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # right_hand — gripper position (rad)
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # head — joint positions (rad)
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
        ],
    ),
    "language": ModalityConfig(
        delta_indices=[0],
        modality_keys=["annotation.human.action.task_description"],
    ),
}

register_modality_config(alfiebot_config, embodiment_tag=EmbodimentTag.NEW_EMBODIMENT)

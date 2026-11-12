from gr00t.configs.data.embodiment_configs import register_modality_config
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import (
    ActionConfig,
    ActionFormat,
    ActionRepresentation,
    ActionType,
    ModalityConfig,
)


# Modality config for Alfiebot GR00T N1.7 fine-tuning.
# Used by launch_finetune.py (--modality-config-path) and gr00t/data/stats.py.
# NOT imported by the inference server at runtime — the server loads
# config directly from the checkpoint's processor_config.json.
#
# Horizon: 16 steps at 15 FPS = 1.07 s, matching checkpoint-20000 (trained
# with delta_indices 0..15). The client executes 8 steps per chunk and keeps
# the rest as overlap/overflow. Inference cost does not depend on this (the
# model pads every chunk to 40 steps). The deployment guide suggests >= 32 for
# Real-Time Chunking; RTC was validated at 16 on this checkpoint (seam jumps
# -50%, jerk -61%, Isaac-GR00T rtc_smoothness_eval.py), so 16 is kept for now.
#
# Representation (N1.7 guidance, getting_started/real_world_deployment.md):
# RELATIVE (delta from the current state) for position-controlled joints,
# which converges faster and keeps consecutive chunks consistent; ABSOLUTE for
# the grippers (binary open/close) and for the base, which is a velocity
# command with no positional state to be relative to. The dataset stores
# absolute values for everything; the processor converts at train/infer time.
#
# Changing delta_indices or the representations requires regenerating
# meta/stats.json and meta/relative_stats.json with gr00t/data/stats.py.
ACTION_HORIZON = 16

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
        delta_indices=list(range(ACTION_HORIZON)),
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
            ActionConfig(  # base — velocity commands (m/s, rad/s): no pose to be relative to
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # back — position (meters), delta from current height
                rep=ActionRepresentation.RELATIVE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # left_arm — joint positions (rad), delta from current
                rep=ActionRepresentation.RELATIVE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # left_hand — gripper position (rad), open/close target
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # right_arm — joint positions (rad), delta from current
                rep=ActionRepresentation.RELATIVE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # right_hand — gripper position (rad), open/close target
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
            ),
            ActionConfig(  # head — joint positions (rad), delta from current
                rep=ActionRepresentation.RELATIVE,
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

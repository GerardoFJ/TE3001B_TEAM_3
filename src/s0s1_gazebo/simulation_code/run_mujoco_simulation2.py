# run_mujoco_simulation.py
import mujoco
import mujoco.viewer

from so101_mujoco_utils2 import (
    set_initial_pose,
    move_to_pose,
    hold_position,
)

MODEL_PATH = "model/scene_urdf.xml"

m = mujoco.MjModel.from_xml_path(MODEL_PATH)
d = mujoco.MjData(m)

# Starting pose from your screenshot (converted to degrees + gripper 0-100)
starting_position = {
    "shoulder_pan":  -4.4003158666,
    "shoulder_lift": -92.2462050161,
    "elbow_flex":     89.9543738355,
    "wrist_flex":     55.1185398916,
    "wrist_roll":      0.0,
    "gripper":         0.0,
}

desired_zero = {
    "shoulder_pan":  0.0,
    "shoulder_lift": 0.0,
    "elbow_flex":    0.0,
    "wrist_flex":    0.0,
    "wrist_roll":    0.0,
    "gripper":       0.0,
}

# Put robot at starting pose before opening viewer
set_initial_pose(m, d, starting_position)

with mujoco.viewer.launch_passive(m, d) as viewer:
    # Move to desired position
    move_to_pose(m, d, viewer, desired_zero, duration=2.0, realtime=True)

    # Hold
    hold_position(m, d, viewer, duration=2.0, realtime=True)

    # Return
    move_to_pose(m, d, viewer, starting_position, duration=2.0, realtime=True)

    # Optional: keep window open a bit at the end
    hold_position(m, d, viewer, duration=20.0, realtime=True)

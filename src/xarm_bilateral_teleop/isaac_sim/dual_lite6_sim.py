#!/usr/bin/env python3
"""
Isaac Sim standalone script: Two xArm Lite6 robots for bilateral teleoperation.

Loads two Lite6 USD models from NVIDIA's asset server, sets up ROS2 bridge
OmniGraph nodes to publish joint states and subscribe to joint commands
for each arm (namespaced as /master and /slave).

Usage:
    cd /home/danielh/isaacsim/_build/linux-x86_64/release
    ./python.sh /home/danielh/sch_ws/TE3001B_TEAM_3/src/xarm_bilateral_teleop/isaac_sim/dual_lite6_sim.py

Then in another terminal, launch the teleop stack:
    ros2 launch xarm_bilateral_teleop bilateral_teleop_isaac.launch.py
"""

import argparse
import sys

# -- Parse args before SimulationApp (it consumes argv) --
parser = argparse.ArgumentParser(description="Dual Lite6 Isaac Sim")
parser.add_argument("--headless", action="store_true", help="Run headless")
parser.add_argument("--master-pos", type=float, nargs=3,
                    default=[0.0, -0.5, 0.0],
                    help="Master arm base position [x y z]")
parser.add_argument("--slave-pos", type=float, nargs=3,
                    default=[0.0, 0.5, 0.0],
                    help="Slave arm base position [x y z]")
args, unknown = parser.parse_known_args()

# ============================================================
# 1. Launch Isaac Sim
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "width": 1280,
    "height": 720,
    "anti_aliasing": 0,
})

# ============================================================
# 2. Imports (must come AFTER SimulationApp)
# ============================================================
import carb
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.nucleus import get_assets_root_path
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema

# ============================================================
# 3. Setup world and load robots
# ============================================================
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 240.0, rendering_dt=1.0 / 60.0)
world.scene.add_default_ground_plane()

# Find Isaac Sim asset root
assets_root = get_assets_root_path()
if assets_root is None:
    carb.log_error("Could not find Isaac Sim assets root path. "
                   "Make sure you have internet or a local Nucleus server.")
    simulation_app.close()
    sys.exit(1)

lite6_usd = assets_root + "/Isaac/Robots/Ufactory/lite6/lite6.usd"
carb.log_info(f"Loading Lite6 from: {lite6_usd}")

stage = omni.usd.get_context().get_stage()

# -- Master arm --
master_prim_path = "/World/master"
add_reference_to_stage(usd_path=lite6_usd, prim_path=master_prim_path)

# -- Slave arm --
slave_prim_path = "/World/slave"
add_reference_to_stage(usd_path=lite6_usd, prim_path=slave_prim_path)

# Let the stage compose
for _ in range(5):
    simulation_app.update()

# ============================================================
# 3b. Inspect prim hierarchy to find the articulation root
# ============================================================
def find_articulation_root(base_path: str) -> str:
    """Walk child prims to find which one has the ArticulationRootAPI."""
    prim = stage.GetPrimAtPath(base_path)
    if not prim.IsValid():
        carb.log_error(f"Prim not found: {base_path}")
        return base_path

    # Check the prim itself
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        carb.log_info(f"Articulation root found at: {base_path}")
        return base_path

    # Check immediate children
    for child in prim.GetChildren():
        child_path = str(child.GetPath())
        if child.HasAPI(UsdPhysics.ArticulationRootAPI):
            carb.log_info(f"Articulation root found at: {child_path}")
            return child_path
        # Check grandchildren
        for grandchild in child.GetChildren():
            gc_path = str(grandchild.GetPath())
            if grandchild.HasAPI(UsdPhysics.ArticulationRootAPI):
                carb.log_info(f"Articulation root found at: {gc_path}")
                return gc_path

    # Fallback: print hierarchy for debugging
    carb.log_warn(f"No ArticulationRootAPI found under {base_path}. "
                  "Printing hierarchy:")
    def print_tree(p, depth=0):
        apis = [s.typeName for s in p.GetAppliedSchemas()] if hasattr(p, 'GetAppliedSchemas') else []
        carb.log_warn(f"{'  ' * depth}{p.GetPath()} type={p.GetTypeName()} apis={apis}")
        for c in p.GetChildren():
            if depth < 4:
                print_tree(c, depth + 1)
    print_tree(prim)

    return base_path


master_art_path = find_articulation_root(master_prim_path)
slave_art_path = find_articulation_root(slave_prim_path)

print(f"  Master articulation root: {master_art_path}")
print(f"  Slave articulation root:  {slave_art_path}")

# ============================================================
# 3c. Set positions via USD XformOps (reliable method)
# ============================================================
def set_prim_translate(prim_path: str, position: list):
    """Set the translation on a prim using USD XformOps."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        carb.log_error(f"Cannot set position — prim not found: {prim_path}")
        return
    xformable = UsdGeom.Xformable(prim)
    # Clear existing xform ops to avoid conflicts
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*position))
    carb.log_info(f"Set {prim_path} position to {position}")


set_prim_translate(master_prim_path, args.master_pos)
set_prim_translate(slave_prim_path, args.slave_pos)

# Let transforms propagate
for _ in range(5):
    simulation_app.update()

# ============================================================
# 4. Enable ROS2 bridge extension
# ============================================================
import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)

# Allow extension to initialize
for _ in range(10):
    simulation_app.update()


# ============================================================
# 5. Create ROS2 OmniGraph for each arm
# ============================================================
def create_ros2_action_graph(arm_ns: str, robot_prim_path: str, graph_path: str):
    """Create an OmniGraph that publishes joint states and subscribes to
    joint commands for a single arm, using ROS2 namespacing."""

    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    # Joint state publisher
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    # Joint state subscriber (for receiving commands)
                    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    # Articulation controller (applies commands to robot)
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    # Publish TF
                    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    # Publish clock
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    # Tick drives everything
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    # Sim time for joint state timestamps
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    # Joint command flow: subscriber -> articulation controller
                    ("SubscribeJointState.outputs:jointNames",
                     "ArticulationController.inputs:jointNames"),
                    ("SubscribeJointState.outputs:positionCommand",
                     "ArticulationController.inputs:positionCommand"),
                    ("SubscribeJointState.outputs:velocityCommand",
                     "ArticulationController.inputs:velocityCommand"),
                    ("SubscribeJointState.outputs:effortCommand",
                     "ArticulationController.inputs:effortCommand"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Use the articulation root path (not the reference Xform)
                    ("ArticulationController.inputs:robotPath", robot_prim_path),
                    ("PublishJointState.inputs:targetPrim", robot_prim_path),
                    ("PublishTF.inputs:targetPrims", [robot_prim_path]),
                    # ROS2 topic namespacing
                    ("PublishJointState.inputs:topicName", f"/{arm_ns}/joint_states"),
                    ("SubscribeJointState.inputs:topicName", f"/{arm_ns}/joint_command"),
                    ("PublishTF.inputs:topicName", f"/{arm_ns}/tf"),
                    ("PublishClock.inputs:topicName", "/clock"),
                ],
            },
        )
        carb.log_info(f"Created ROS2 action graph for {arm_ns} at {graph_path}")
    except Exception as e:
        carb.log_error(f"Failed to create action graph for {arm_ns}: {e}")


# Create action graphs using the ACTUAL articulation root paths
create_ros2_action_graph("master", master_art_path, "/World/MasterActionGraph")
create_ros2_action_graph("slave", slave_art_path, "/World/SlaveActionGraph")

# NOTE: JointTrajectory bridging is handled on the ROS2 side by the
# traj_to_js node, which converts JointTrajectory → JointState and
# publishes to /{arm_ns}/joint_command (subscribed by the action graph above).

# ============================================================
# 7. Reset world and run simulation
# ============================================================
world.reset()

# Set initial joint positions (non-singular "ready pose" from teleop config)
ready_pose = np.array([0.0, -0.5, 0.1, 0.0, 0.8, 0.0])

# Wait a few frames for physics to settle
for _ in range(30):
    world.step(render=True)

# Try to set initial joint positions via the articulation API
try:
    from isaacsim.core.prims import SingleArticulation

    master_art = SingleArticulation(prim_path=master_art_path)
    slave_art = SingleArticulation(prim_path=slave_art_path)
    master_art.initialize()
    slave_art.initialize()

    num_master_dof = master_art.num_dof
    num_slave_dof = slave_art.num_dof
    print(f"  Master DOF: {num_master_dof}")
    print(f"  Slave DOF:  {num_slave_dof}")

    # Only set as many joints as the robot has
    master_art.set_joint_positions(ready_pose[:num_master_dof])
    slave_art.set_joint_positions(ready_pose[:num_slave_dof])
    carb.log_info("Both arms set to ready pose")
except Exception as e:
    carb.log_warn(f"Could not set initial joint positions: {e}")

print("\n" + "=" * 60)
print("  Dual Lite6 Isaac Sim simulation running!")
print("  Master arm at:", args.master_pos)
print("  Slave arm at: ", args.slave_pos)
print(f"  Master articulation: {master_art_path}")
print(f"  Slave articulation:  {slave_art_path}")
print()
print("  ROS2 topics published:")
print("    /master/joint_states")
print("    /slave/joint_states")
print("    /master/tf, /slave/tf")
print("    /clock")
print()
print("  ROS2 topics subscribed:")
print("    /master/joint_command")
print("    /slave/joint_command")
print("    /master/lite6_traj_controller/joint_trajectory")
print("    /slave/lite6_traj_controller/joint_trajectory")
print()
print("  Now launch the teleop stack in another terminal:")
print("    ros2 launch xarm_bilateral_teleop bilateral_teleop_isaac.launch.py")
print("=" * 60 + "\n")

# ============================================================
# 8. Main loop
# ============================================================
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()

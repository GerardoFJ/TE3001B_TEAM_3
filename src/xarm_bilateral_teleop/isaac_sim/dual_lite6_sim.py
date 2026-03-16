#!/usr/bin/env python3
"""
Isaac Sim standalone script: Two xArm Lite6 robots for bilateral teleoperation.

Loads two Lite6 USD models from NVIDIA's asset server, adds contact sensors
to the end-effectors, and publishes force data as WrenchStamped via ROS2.

The user interacts with the master arm by pushing objects against it in the
Isaac Sim viewport. The contact sensor detects the force and feeds it into
the bilateral teleoperation control loop.

Usage:
    cd /home/danielh/isaacsim/_build/linux-x86_64/release
    ./python.sh /home/danielh/sch_ws/TE3001B_TEAM_3/src/xarm_bilateral_teleop/isaac_sim/dual_lite6_sim.py

Then in another terminal, launch the teleop stack:
    ros2 launch xarm_bilateral_teleop bilateral_teleop_isaac.launch.py
"""

import argparse
import sys
import threading

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
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, PhysxSchema

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
# 3b. Find articulation roots
# ============================================================
def find_articulation_root(base_path: str) -> str:
    """Walk child prims to find which one has the ArticulationRootAPI."""
    prim = stage.GetPrimAtPath(base_path)
    if not prim.IsValid():
        carb.log_error(f"Prim not found: {base_path}")
        return base_path
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return base_path
    for child in prim.GetChildren():
        child_path = str(child.GetPath())
        if child.HasAPI(UsdPhysics.ArticulationRootAPI):
            return child_path
        for grandchild in child.GetChildren():
            gc_path = str(grandchild.GetPath())
            if grandchild.HasAPI(UsdPhysics.ArticulationRootAPI):
                return gc_path
    # Fallback: print hierarchy
    carb.log_warn(f"No ArticulationRootAPI found under {base_path}")
    def print_tree(p, depth=0):
        carb.log_warn(f"{'  ' * depth}{p.GetPath()} type={p.GetTypeName()}")
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
# 3c. Set positions via USD XformOps
# ============================================================
def set_prim_translate(prim_path: str, position: list):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    translate_op = xformable.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(*position))


set_prim_translate(master_prim_path, args.master_pos)
set_prim_translate(slave_prim_path, args.slave_pos)

for _ in range(5):
    simulation_app.update()

# ============================================================
# 3d. Find the last link (EE) of each arm for contact sensors
# ============================================================
def find_link_prim(art_path: str, link_name: str) -> str:
    """Find a link prim by name under the articulation tree."""
    prim = stage.GetPrimAtPath(art_path)
    if not prim.IsValid():
        return ""
    # BFS search
    queue = [prim]
    while queue:
        p = queue.pop(0)
        name = str(p.GetPath()).split("/")[-1]
        if link_name in name.lower():
            return str(p.GetPath())
        for c in p.GetChildren():
            queue.append(c)
    return ""


master_ee_path = find_link_prim(master_art_path, "link6")
slave_ee_path = find_link_prim(slave_art_path, "link6")
if not master_ee_path:
    master_ee_path = find_link_prim(master_art_path, "link_eef")
if not slave_ee_path:
    slave_ee_path = find_link_prim(slave_art_path, "link_eef")

print(f"  Master EE link: {master_ee_path}")
print(f"  Slave EE link:  {slave_ee_path}")

# ============================================================
# 3e. Add contact sensors to EE links
# ============================================================
def add_contact_sensor(parent_path: str, sensor_name: str = "contact_sensor"):
    """Add an IsaacContactSensor prim as a child of the given link."""
    sensor_path = f"{parent_path}/{sensor_name}"
    # Create the sensor prim using USD
    sensor_prim = stage.DefinePrim(sensor_path, "IsaacContactSensor")
    if sensor_prim.IsValid():
        # Set sensor properties
        sensor_prim.GetAttribute("radius").Set(-1.0)  # use parent geometry
        sensor_prim.GetAttribute("sensorPeriod").Set(0.0)  # every physics step
        sensor_prim.GetAttribute("threshold").Set(Gf.Vec2f(0.01, 100000.0))
        carb.log_info(f"Added contact sensor at: {sensor_path}")
    else:
        carb.log_warn(f"Failed to create contact sensor at: {sensor_path}")
    return sensor_path


master_cs_path = ""
slave_cs_path = ""
if master_ee_path:
    master_cs_path = add_contact_sensor(master_ee_path, "contact_sensor")
if slave_ee_path:
    slave_cs_path = add_contact_sensor(slave_ee_path, "contact_sensor")

# ============================================================
# 3f. Add interactive objects for the user to push against arms
# ============================================================
def create_interactive_box(name: str, position: list, size: float = 0.05,
                           mass: float = 0.5, color: tuple = (0.2, 0.6, 1.0)):
    """Create a dynamic box with collision that the user can drag in the viewport."""
    box_path = f"/World/{name}"
    cube_prim = stage.DefinePrim(box_path, "Cube")
    xformable = UsdGeom.Xformable(cube_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*position))
    xformable.AddScaleOp().Set(Gf.Vec3d(size, size, size))

    # Physics: make it a dynamic rigid body with collision
    UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    UsdPhysics.MassAPI.Apply(cube_prim)
    cube_prim.GetAttribute("physics:mass").Set(mass)

    carb.log_info(f"Created interactive box '{name}' at {position}")
    return box_path


# Place boxes near each arm's EE position for interaction
# (approximate EE position: arm base + ~0.3m up + ~0.2m forward)
master_box_pos = [
    args.master_pos[0] + 0.25,
    args.master_pos[1],
    args.master_pos[2] + 0.35,
]
slave_box_pos = [
    args.slave_pos[0] + 0.25,
    args.slave_pos[1],
    args.slave_pos[2] + 0.35,
]
create_interactive_box("master_object", master_box_pos, size=0.03, mass=0.3)
create_interactive_box("slave_object", slave_box_pos, size=0.03, mass=0.3)

for _ in range(5):
    simulation_app.update()

# ============================================================
# 4. Enable extensions
# ============================================================
import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
ext_manager.set_extension_enabled_immediate("isaacsim.sensors.physics", True)

for _ in range(10):
    simulation_app.update()

# ============================================================
# 5. Create ROS2 OmniGraph for each arm (joint states + commands)
# ============================================================
def create_ros2_action_graph(arm_ns: str, robot_prim_path: str, graph_path: str):
    try:
        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
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
                    ("ArticulationController.inputs:robotPath", robot_prim_path),
                    ("PublishJointState.inputs:targetPrim", robot_prim_path),
                    ("PublishTF.inputs:targetPrims", [robot_prim_path]),
                    ("PublishJointState.inputs:topicName", f"/{arm_ns}/joint_states"),
                    ("SubscribeJointState.inputs:topicName", f"/{arm_ns}/joint_command"),
                    ("PublishTF.inputs:topicName", f"/{arm_ns}/tf"),
                    ("PublishClock.inputs:topicName", "/clock"),
                ],
            },
        )
        carb.log_info(f"Created ROS2 action graph for {arm_ns}")
    except Exception as e:
        carb.log_error(f"Failed to create action graph for {arm_ns}: {e}")


create_ros2_action_graph("master", master_art_path, "/World/MasterActionGraph")
create_ros2_action_graph("slave", slave_art_path, "/World/SlaveActionGraph")

# ============================================================
# 6. Setup force publishing via rclpy (from Isaac Sim's ROS2 bridge)
#    Reads joint efforts from the articulations, computes EE wrench
#    using the Jacobian transpose method, and publishes WrenchStamped.
# ============================================================

# Import rclpy from Isaac Sim's bundled ROS2
rclpy_available = False
try:
    import rclpy
    from rclpy.node import Node as RclpyNode
    from geometry_msgs.msg import WrenchStamped
    from std_msgs.msg import Bool
    rclpy_available = True
    carb.log_info("rclpy available — will publish force sensor data")
except ImportError:
    carb.log_warn("rclpy not available — force sensor publishing disabled. "
                  "Make sure ROS2 environment is sourced before running Isaac Sim.")


class IsaacForceSensorPublisher:
    """Reads joint efforts from Isaac Sim articulations and publishes
    WrenchStamped using the Jacobian transpose force estimation method.
    Also reads contact sensors for contact detection."""

    def __init__(self):
        rclpy.init(args=[])
        self._node = rclpy.create_node('isaac_force_sensor')

        # Publishers
        self._pubs = {}
        self._contact_pubs = {}
        for ns in ['master', 'slave']:
            self._pubs[ns] = self._node.create_publisher(
                WrenchStamped, f'/{ns}/force_sensor/force_estimate', 10)
            self._contact_pubs[ns] = self._node.create_publisher(
                Bool, f'/{ns}/force_sensor/contact_detected', 10)

        # Spin in background thread
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        carb.log_info("IsaacForceSensorPublisher started")

    def _spin(self):
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.01)

    def publish_wrench(self, arm_ns: str, force: np.ndarray, torque: np.ndarray,
                       in_contact: bool):
        """Publish a WrenchStamped message."""
        msg = WrenchStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = 'link_base'
        msg.wrench.force.x = float(force[0])
        msg.wrench.force.y = float(force[1])
        msg.wrench.force.z = float(force[2])
        msg.wrench.torque.x = float(torque[0])
        msg.wrench.torque.y = float(torque[1])
        msg.wrench.torque.z = float(torque[2])
        self._pubs[arm_ns].publish(msg)

        contact_msg = Bool()
        contact_msg.data = in_contact
        self._contact_pubs[arm_ns].publish(contact_msg)

    def destroy(self):
        self._node.destroy_node()
        rclpy.shutdown()


force_publisher = None
if rclpy_available:
    try:
        force_publisher = IsaacForceSensorPublisher()
    except Exception as e:
        carb.log_warn(f"Could not start force publisher: {e}")

# ============================================================
# 6b. Force estimation from joint efforts (Jacobian transpose)
#     Same physics as the original force_sensor_node.py but
#     runs inside Isaac Sim with real simulation torques.
# ============================================================

# Lite6 DH / kinematics parameters (from kinematics.py)
JOINT_ORIGINS = [
    (0, 0, 0.2435, 0, 0, 0),
    (0, 0, 0, 1.5708, -1.5708, 3.1416),
    (0.2002, 0, 0, -3.1416, 0, 1.5708),
    (0.087, -0.22761, 0, 1.5708, 0, 0),
    (0, 0, 0, 1.5708, 0, 0),
    (0, 0.0625, 0, -1.5708, 0, 0),
]
LINK_MASSES = [1.411, 1.34, 0.953, 1.284, 0.804, 0.13]
LINK_COMS = [
    (-0.00036, 0.04195, -0.0025),
    (0.179, 0.0, 0.0584),
    (0.072, -0.0357, -0.001),
    (-0.002, -0.0285, -0.0813),
    (0.0, 0.01, 0.0019),
    (0.0, -0.00194, -0.0102),
]


def _make_transform(x, y, z, roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def forward_kinematics(q):
    T = np.eye(4)
    for i in range(6):
        T_joint = _make_transform(*JOINT_ORIGINS[i])
        Rz = np.eye(4)
        c, s = np.cos(q[i]), np.sin(q[i])
        Rz[:2, :2] = [[c, -s], [s, c]]
        T = T @ T_joint @ Rz
    return T


def geometric_jacobian_6x6(q):
    """Compute 6x6 geometric Jacobian [Jv; Jw]."""
    T = np.eye(4)
    origins = [np.zeros(3)]
    z_axes = [np.array([0, 0, 1])]
    for i in range(6):
        T_joint = _make_transform(*JOINT_ORIGINS[i])
        T = T @ T_joint
        z_axes.append(T[:3, 2].copy())
        origins.append(T[:3, 3].copy())
        Rz = np.eye(4)
        c, s = np.cos(q[i]), np.sin(q[i])
        Rz[:2, :2] = [[c, -s], [s, c]]
        T = T @ Rz
    p_ee = T[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        J[:3, i] = np.cross(z_axes[i], p_ee - origins[i])
        J[3:, i] = z_axes[i]
    return J


def gravity_torques(q):
    """Compute gravity compensation torques."""
    g = np.array([0, 0, -9.81])
    T = np.eye(4)
    tau_g = np.zeros(6)
    transforms = [np.eye(4)]
    for i in range(6):
        T_joint = _make_transform(*JOINT_ORIGINS[i])
        Rz = np.eye(4)
        c, s = np.cos(q[i]), np.sin(q[i])
        Rz[:2, :2] = [[c, -s], [s, c]]
        T = T @ T_joint @ Rz
        transforms.append(T.copy())
    for i in range(6):
        T_link = transforms[i + 1]
        com_local = np.array(LINK_COMS[i])
        com_world = T_link[:3, :3] @ com_local + T_link[:3, 3]
        F_grav = LINK_MASSES[i] * g
        for j in range(i + 1):
            z_j = transforms[j][:3, 2]
            o_j = transforms[j][:3, 3]
            r = com_world - o_j
            tau_g[j] += np.dot(z_j, np.cross(r, F_grav))
    return tau_g


def estimate_ee_wrench(q, tau_measured):
    """Estimate external EE wrench from joint torque residuals."""
    tau_grav = gravity_torques(q)
    tau_ext = tau_measured - tau_grav
    J = geometric_jacobian_6x6(q)
    # Damped least-squares pseudo-inverse of J^T
    JT = J.T
    damping = 0.1
    JT_pinv = J @ np.linalg.inv(JT @ J + damping**2 * np.eye(6))
    F_ext = JT_pinv @ tau_ext
    return F_ext  # [Fx, Fy, Fz, Tx, Ty, Tz]


# Low-pass filter state
_lpf_state = {'master': np.zeros(6), 'slave': np.zeros(6)}
_lpf_alpha = 0.1  # ~5 Hz cutoff at 60 Hz update rate

# Calibration: capture baseline torques at rest
_calibration = {
    'master': {'done': False, 'samples': [], 'bias': np.zeros(6)},
    'slave': {'done': False, 'samples': [], 'bias': np.zeros(6)},
}
CALIBRATION_SAMPLES = 120  # ~2 seconds at 60 Hz


# ============================================================
# 7. Reset world and run simulation
# ============================================================
world.reset()

ready_pose = np.array([0.0, -0.5, 0.1, 0.0, 0.8, 0.0])

for _ in range(30):
    world.step(render=True)

# Initialize articulations
master_art = None
slave_art = None
try:
    from isaacsim.core.prims import SingleArticulation

    master_art = SingleArticulation(prim_path=master_art_path)
    slave_art = SingleArticulation(prim_path=slave_art_path)
    master_art.initialize()
    slave_art.initialize()

    num_dof = master_art.num_dof
    print(f"  Master DOF: {num_dof}, Slave DOF: {slave_art.num_dof}")

    master_art.set_joint_positions(ready_pose[:num_dof])
    slave_art.set_joint_positions(ready_pose[:num_dof])
    carb.log_info("Both arms set to ready pose")
except Exception as e:
    carb.log_warn(f"Could not initialize articulations: {e}")

# Read contact sensor interface
cs_interface = None
try:
    from omni.isaac.sensor import _sensor
    cs_interface = _sensor.acquire_contact_sensor_interface()
    carb.log_info("Contact sensor interface acquired")
except Exception:
    try:
        from isaacsim.sensors.physics import _physics_sensor
        cs_interface = _physics_sensor.acquire_contact_sensor_interface()
        carb.log_info("Contact sensor interface acquired (new API)")
    except Exception as e:
        carb.log_warn(f"Contact sensor interface not available: {e}")

print("\n" + "=" * 60)
print("  Dual Lite6 Isaac Sim simulation running!")
print("  Master arm at:", args.master_pos)
print("  Slave arm at: ", args.slave_pos)
print(f"  Master articulation: {master_art_path}")
print(f"  Slave articulation:  {slave_art_path}")
print()
print("  Force sensing: Isaac Sim physics (joint effort + Jacobian)")
print("  Interactive objects placed near each arm's end-effector.")
print("  Drag the cubes onto the arm tips to apply forces!")
print()
print("  ROS2 topics published:")
print("    /master/joint_states, /slave/joint_states")
print("    /master/force_sensor/force_estimate (WrenchStamped)")
print("    /slave/force_sensor/force_estimate  (WrenchStamped)")
print("    /master/force_sensor/contact_detected (Bool)")
print("    /slave/force_sensor/contact_detected  (Bool)")
print()
print("  Now launch the teleop stack:")
print("    ros2 launch xarm_bilateral_teleop bilateral_teleop_isaac.launch.py")
print("=" * 60 + "\n")

# ============================================================
# 8. Main loop with force sensing
# ============================================================
step_count = 0

while simulation_app.is_running():
    world.step(render=True)
    step_count += 1

    # Publish force sensor data every ~4 physics steps (~60 Hz)
    if force_publisher is not None and master_art is not None and step_count % 4 == 0:
        for arm_ns, art in [('master', master_art), ('slave', slave_art)]:
            try:
                q = art.get_joint_positions()
                efforts = art.get_applied_joint_efforts()

                if q is None or efforts is None:
                    continue
                q = np.array(q, dtype=float)
                efforts = np.array(efforts, dtype=float)

                # Calibration phase: build torque bias at rest
                cal = _calibration[arm_ns]
                if not cal['done']:
                    cal['samples'].append(efforts.copy())
                    if len(cal['samples']) >= CALIBRATION_SAMPLES:
                        cal['bias'] = np.mean(cal['samples'], axis=0)
                        cal['done'] = True
                        carb.log_info(
                            f"{arm_ns} force sensor calibrated "
                            f"(bias={np.round(cal['bias'], 2).tolist()})")
                    continue

                # Subtract calibration bias
                efforts_corrected = efforts - cal['bias']

                # Estimate EE wrench
                F_ext = estimate_ee_wrench(q[:6], efforts_corrected[:6])

                # Low-pass filter
                _lpf_state[arm_ns] = (
                    _lpf_alpha * F_ext +
                    (1.0 - _lpf_alpha) * _lpf_state[arm_ns]
                )
                F_filtered = _lpf_state[arm_ns]

                # Contact detection (force magnitude > 2 N)
                force_mag = np.linalg.norm(F_filtered[:3])
                in_contact = force_mag > 2.0

                force_publisher.publish_wrench(
                    arm_ns,
                    force=F_filtered[:3],
                    torque=F_filtered[3:],
                    in_contact=in_contact,
                )

            except Exception as e:
                if step_count % 240 == 0:  # log every ~1s
                    carb.log_warn(f"Force sensing error ({arm_ns}): {e}")

# Cleanup
if force_publisher is not None:
    force_publisher.destroy()
simulation_app.close()

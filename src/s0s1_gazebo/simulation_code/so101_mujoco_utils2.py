# so101_mujoco_utils2.py
from __future__ import annotations

import time
import math
import threading
from collections import deque
from typing import Optional

import mujoco

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def _deg2rad(x: float) -> float:
    return x * math.pi / 180.0


def _rad2deg(x: float) -> float:
    return x * 180.0 / math.pi


def convert_to_dictionary(qpos):
    """
    Assumes qpos order matches JOINT_NAMES (6 joints).
    Outputs degrees + gripper 0-100.
    NOTE: This is NOT robust to qpos ordering changes; prefer get_positions_dict(m, d).
    """
    return {
        "shoulder_pan": _rad2deg(qpos[0]),
        "shoulder_lift": _rad2deg(qpos[1]),
        "elbow_flex": _rad2deg(qpos[2]),
        "wrist_flex": _rad2deg(qpos[3]),
        "wrist_roll": _rad2deg(qpos[4]),
        "gripper": float(qpos[5]) * 100.0 / math.pi,  # map rad -> 0..100 (assuming 0..pi)
    }


def convert_to_list(position_dict):
    """Degrees + gripper 0-100 -> radians list (6)."""
    return [
        _deg2rad(float(position_dict["shoulder_pan"])),
        _deg2rad(float(position_dict["shoulder_lift"])),
        _deg2rad(float(position_dict["elbow_flex"])),
        _deg2rad(float(position_dict["wrist_flex"])),
        _deg2rad(float(position_dict["wrist_roll"])),
        float(position_dict["gripper"]) * math.pi / 100.0,
    ]


def _joint_qpos_index(m, joint_name: str) -> int:
    """Return the qpos index (qposadr) for a hinge/slide joint."""
    j_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if j_id < 0:
        raise ValueError(f"Joint '{joint_name}' not found in model.")
    return int(m.jnt_qposadr[j_id])


def _actuator_index(m, actuator_name: str) -> int:
    """Return ctrl index for an actuator."""
    a_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
    if a_id < 0:
        raise ValueError(f"Actuator '{actuator_name}' not found in model.")
    return int(a_id)


def get_positions_dict(m, d):
    """
    Returns joint positions in degrees (first 5) + gripper 0..100,
    using joint-name -> qposadr mapping (robust to qpos ordering).
    """
    out = {}
    for name in JOINT_NAMES:
        qadr = _joint_qpos_index(m, name)
        q = float(d.qpos[qadr])
        if name == "gripper":
            out[name] = q * 100.0 / math.pi
        else:
            out[name] = _rad2deg(q)
    return out


def set_initial_pose(m, d, position_dict):
    target = convert_to_list(position_dict)
    for i, name in enumerate(JOINT_NAMES):
        qadr = _joint_qpos_index(m, name)
        d.qpos[qadr] = target[i]
    mujoco.mj_forward(m, d)


def send_position_command(m, d, position_dict):
    """
    Sends position commands through MuJoCo <position> actuators by setting d.ctrl.
    Expects degrees + gripper 0..100 in position_dict.
    """
    target = convert_to_list(position_dict)
    for i, name in enumerate(JOINT_NAMES):
        a_id = _actuator_index(m, name)
        d.ctrl[a_id] = target[i]


def _step_realtime(m, step_start: float):
    dt = float(m.opt.timestep)
    remaining = dt - (time.time() - step_start)
    if remaining > 0:
        time.sleep(remaining)


# ---------------------------
# Plotly realtime plotter (Dash server in a thread)
# ---------------------------
class RealtimeJointPlotter:
    """
    Opens a Dash+Plotly page showing joint positions vs time.

    Usage:
      plotter = RealtimeJointPlotter(max_points=4000)
      plotter.start(host="127.0.0.1", port=8050, update_ms=100)
      ... inside sim loop: plotter.sample(m, d)
    """

    def __init__(self, joint_names=JOINT_NAMES, max_points: int = 2000):
        self.joint_names = list(joint_names)
        self.max_points = int(max_points)

        self._lock = threading.Lock()
        self._t0: Optional[float] = None
        self._t = deque(maxlen=self.max_points)
        self._y = {jn: deque(maxlen=self.max_points) for jn in self.joint_names}

        self._dash_thread: Optional[threading.Thread] = None
        self._running = False

    def sample(self, m, d, now: Optional[float] = None) -> None:
        """Record one sample from MuJoCo state."""
        if now is None:
            now = time.time()
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0

        pos = get_positions_dict(m, d)  # degrees + gripper 0..100

        with self._lock:
            self._t.append(float(t))
            for jn in self.joint_names:
                self._y[jn].append(float(pos[jn]))

    def start(self, host: str = "127.0.0.1", port: int = 8050, update_ms: int = 100) -> None:
        """
        Starts the Dash server in a daemon thread.
        Install deps: pip install dash plotly
        """
        if self._running:
            return
        self._running = True

        def _run_dash():
            try:
                from dash import Dash, dcc, html, no_update
                from dash.dependencies import Input, Output
                import plotly.graph_objects as go
            except Exception as e:
                self._running = False
                raise RuntimeError(
                    "Dash/Plotly not available. Install with: pip install dash plotly"
                ) from e

            app = Dash(__name__)

            # Create traces ONCE so the plot is never "empty"
            fig = go.Figure()
            for jn in self.joint_names:
                fig.add_trace(go.Scatter(x=[], y=[], mode="lines", name=jn))

            fig.update_layout(
                title="SO101 Joint Positions (deg, gripper=0..100)",
                xaxis_title="time (s)",
                yaxis_title="position",
                margin=dict(l=40, r=20, t=50, b=40),
                legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
                template="plotly_white",
            )

            app.layout = html.Div(
                style={"maxWidth": "1200px", "margin": "0 auto"},
                children=[
                    html.H3("Realtime Joint Positions"),
                    dcc.Graph(id="live-graph", figure=fig, animate=False),
                    dcc.Interval(id="interval", interval=int(update_ms), n_intervals=0),
                    html.Div(f"Open: http://{host}:{port}", style={"opacity": 0.6}),
                    html.Pre(id="debug-text", style={"opacity": 0.7}),
                ],
            )

            @app.callback(
                Output("live-graph", "extendData"),
                Output("debug-text", "children"),
                Input("interval", "n_intervals"),
            )
            def _update(_):
                with self._lock:
                    n = len(self._t)
                    if n == 0:
                        return no_update, "No samples yet (buffer empty). Is plotter.sample() being called?"

                    # Stream ONLY the last sample (fast + stable)
                    t_last = self._t[-1]
                    y_last = [self._y[jn][-1] for jn in self.joint_names]

                update = {
                    "x": [[t_last]] * len(self.joint_names),
                    "y": [[y] for y in y_last],
                }
                trace_indices = list(range(len(self.joint_names)))
                debug = f"samples={n}, last_t={t_last:.3f}, last={dict(zip(self.joint_names, y_last))}"

                # Keep last max_points points (Plotly trims client-side)
                return (update, trace_indices, self.max_points), debug

            # Prevent the reloader from spawning a second process/thread
            app.run(host=host, port=port, debug=False, use_reloader=False)

        self._dash_thread = threading.Thread(target=_run_dash, daemon=True)
        self._dash_thread.start()

    def stop(self) -> None:
        # Dash doesn't provide a clean stop-from-thread API. Daemon thread exits when process exits.
        self._running = False


# ---------------------------
# Existing motion helpers, with OPTIONAL plotting hook
# ---------------------------
def move_to_pose(m, d, viewer, desired_position, duration, realtime: bool = True, plotter: Optional[RealtimeJointPlotter] = None):
    start_time = time.time()
    starting_pose = get_positions_dict(m, d)

    while viewer.is_running():
        t = time.time() - start_time
        if t >= duration:
            break

        alpha = min(t / duration, 1.0)

        cmd = {}
        for joint in desired_position.keys():
            p0 = starting_pose[joint]
            pf = desired_position[joint]
            cmd[joint] = (1 - alpha) * p0 + alpha * pf

        step_start = time.time()
        send_position_command(m, d, cmd)
        mujoco.mj_step(m, d)

        if plotter is not None:
            plotter.sample(m, d)

        viewer.sync()
        if realtime:
            _step_realtime(m, step_start)


def hold_position(m, d, viewer, duration, realtime: bool = True, plotter: Optional[RealtimeJointPlotter] = None):
    hold_cmd = get_positions_dict(m, d)
    start_time = time.time()

    while viewer.is_running():
        t = time.time() - start_time
        if t >= duration:
            break

        step_start = time.time()
        send_position_command(m, d, hold_cmd)
        mujoco.mj_step(m, d)

        if plotter is not None:
            plotter.sample(m, d)

        viewer.sync()
        if realtime:
            _step_realtime(m, step_start)

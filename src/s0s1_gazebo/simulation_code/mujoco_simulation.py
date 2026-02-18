import time
import mujoco
import mujoco.viewer

# IMPORTANT: import the plotter from wherever you defined it
# (example: so101_mujoco_utils2.py)
from so101_mujoco_utils2 import RealtimeJointPlotter

m = mujoco.MjModel.from_xml_path("model/scene_urdf.xml")
d = mujoco.MjData(m)

# Start Plotly realtime plotter (Dash)
plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(host="127.0.0.1", port=8050, update_ms=100)  # open http://127.0.0.1:8050

with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        mujoco.mj_step(m, d)

        # NEW: record one sample per step (after mj_step = realized state)
        plotter.sample(m, d)

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

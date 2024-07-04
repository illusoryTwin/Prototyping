import time
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path('example.xml')
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(model, data)

        for q1 in range(0, 360, 5):
            for q2 in range(0, 360, 5):
                for q3 in range(0, 360, 5):

                    if data.contact.geom.size > 0:
                        print("collision")
                        mujoco.mj_step(model, data)
                        break
                    data.qpos[0] = np.deg2rad(q1)
                    data.qpos[1] = np.deg2rad(q2)
                    data.qpos[2] = np.deg2rad(q3)
                    print("data.qpos", data.qpos)
                    mujoco.mj_step(model, data)

                    data.qvel = [0, 0, 0]
                    data.qacc = [0, 0, 0]
                    mujoco.mj_inverse(model, data)

                    mujoco.mj_step(model, data)

                    # Pick up changes to the physics state, apply perturbations, update options from GUI.
                    viewer.sync()

                    # Rudimentary time keeping, will drift relative to wall clock.
                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
#
# for i in range(0, 360, 15):
#     for j in range(0, 360, 15):
#         for k in range(0, 360, 15):
#             if k > 180:
#                 break
#             print(i, j, k)
#
#

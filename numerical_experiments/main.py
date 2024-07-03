import time
import mujoco
import mujoco.viewer
import numpy as np
import itertools

model = mujoco.MjModel.from_xml_path('example.xml')
data = mujoco.MjData(model)

angles = np.arange(0, 360, 5)
configurations = list(itertools.product(angles, repeat=3))
qpos_list = [np.deg2rad(config) for config in configurations]

# def check_torques(model_, data_):


#     with mujoco.viewer.launch_passive(model, data) as viewer:
#         # Close the viewer automatically after 30 wall-seconds.
#         start = time.time()

#         while viewer.is_running() and time.time() - start < 30:
#             data.qpos = np.deg2rad([15, 15, 15])
#             data.qvel = [0, 0, 0]
#             data.qacc = [0, 0, 0]


#             step_start = time.time()

#             mujoco.mj_inverse(model, data)
#             print("tau: ", data.qfrc_inverse)

#             mujoco.mj_step(model, data)
#             print("contact", data.contact.geom.size > 0)

#             # if data.contact.geom.size > 0:
#                 # "collision"


#             # data.qvel = [0, 0, 0]
#             # data.qacc = []
#             viewer.sync()

#             # Rudimentary time keeping, will drift relative to wall clock.
#             time_until_next_step = model.opt.timestep - (time.time() - step_start)
#             if time_until_next_step > 0:
#                 time.sleep(time_until_next_step)


with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()

    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()
        mujoco.mj_step(data, model)
        while data.contact.geom.size > 0:

            for qpos_ in qpos_list:
                print(qpos_)

                data.qpos = qpos_
                # data.qvel = [0, 0, 0]
                # data.qacc = [0, 0, 0]

                step_start = time.time()

                # mujoco.mj_inverse(model, data)
                # qfrc_ = data.qfrc_passive + data.qfrc_actuator + data.qfrc_applied
                # print("data.qfrc_passive", data.qfrc_passive)
                # print("data.qfrc_actuator", data.qfrc_actuator)
                # print("data.qfrc_applied", data.qfrc_applied)
                # print("qfrc_", qfrc_)
                # print("tau: ", data.qfrc_inverse)

                mujoco.mj_step(model, data)
                mujoco.mj_inverse(model, data)
                # print("contact", data.contact.geom.size > 0)

                # # if data.contact.geom.size > 0:
                #     # "collision"

                # # data.qvel = [0, 0, 0]
                # # data.qacc = []

            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
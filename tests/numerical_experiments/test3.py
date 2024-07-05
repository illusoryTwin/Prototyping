import time
import mujoco
import mujoco.viewer
import numpy as np
import csv

model = mujoco.MjModel.from_xml_path('example.xml')
data = mujoco.MjData(model)
measurements_list = []

# def collision_handler(model_, data_):
#     if data_.contact.geom.size != 0:
#         data_.qpos += [np.deg2rad(30, 30, 30)]
#         # if data.contact:
#         print("collision occurred")
#         mujoco.mj_step(model_, data_)
#         break


with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        for q1 in range(0, 360, 5):
            for q2 in range(0, 360, 5):
                for q3 in range(0, 360, 5):
                    mujoco.mj_step(model, data)

                    if data.contact.geom.size > 0:
                        print("collision occurred")
                        # mujoco.mj_step(model, data)
                        # data.qpos += [np.deg2rad(30), np.deg2rad(30), np.deg2rad(30)]
                        break

                    data.qpos = [np.deg2rad(q1), np.deg2rad(q2), np.deg2rad(q3)]

                    mujoco.mj_step(model, data)

                    data.qvel = [0, 0, 0]
                    data.qacc = [0, 0, 0]
                    mujoco.mj_inverse(model, data)
                    tau = data.qfrc_inverse
                    measurements_list.append([q1, q2, q3, tau[0], tau[1], tau[2]])
                    # print("tau: ", data.qfrc_inverse)

                    mujoco.mj_step(model, data)

                    viewer.sync()

                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)


with open('angles_and_torques.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Angle1", "Angle2", "Angle3", "Tau1", "Tau2", "Tau3"])
    writer.writerows(measurements_list)

print("Data saved to angles_and_torques.csv")
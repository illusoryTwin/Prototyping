import time
import mujoco
import mujoco.viewer
import numpy as np
import csv

model = mujoco.MjModel.from_xml_path('example.xml')
data = mujoco.MjData(model)
measurements_list = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 10:
        step_start = time.time()

        for q1 in range(0, 360, 25):
            for q2 in range(0, 360, 25):
                for q3 in range(0, 360, 25):
                    # mujoco.mj_step(model, data)
                    data.qpos = [np.deg2rad(q1), np.deg2rad(q2), np.deg2rad(q3)]
                    data.qvel = [0, 0, 0]
                    data.qacc = [0, 0, 0]
                    mujoco.mj_step(model, data)
                    mujoco.mj_inverse(model, data)

                    if data.contact.geom.size > 0:
                        print("collision occurred")
                    else:
                        tau = data.qfrc_inverse
                        measurements_list.append([q1, q2, q3, tau[0], tau[1], tau[2]])
                        # print("tau: ", data.qfrc_inverse)

                    viewer.sync()

                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)

with open('angles_and_torques5.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Angle1", "Angle2", "Angle3", "Tau1", "Tau2", "Tau3"])
    writer.writerows(measurements_list)

print("Data saved to angles_and_torques.csv")

import mujoco
from mujoco import viewer
import numpy as np
from itertools import product
import time
import csv

# Load the model and data
model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)
end_time = 30

# Generate angle combinations
angle_combinations = product(range(0, 40, 5), repeat=4)
time_list = []
q_t = []
dq_t = []
torques = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < end_time:
        step_start = time.time()
        for angle in angle_combinations:
            # Set joint angles using the combinations
            data.qvel[:] = [0] * 13
            data.qacc[:] = [0] * 13
            data.qpos[2], data.qpos[5] = np.deg2rad(angle[0]), np.deg2rad(angle[1])
            data.qpos[9], data.qpos[12] = np.deg2rad(angle[2]), np.deg2rad(angle[3])
            # data.qpos[0], data.qpos[3] = np.deg2rad(angle[0]), np.deg2rad(angle[1])
            # data.qpos[7], data.qpos[10] = np.deg2rad(angle[2]), np.deg2rad(angle[3])
            data.qpos[6] = 0

            # data.qpos[6], data.qpos[9] = np.deg2rad(angle[2]), np.deg2rad(angle[3])
            # data.qpos[2], data.qpos[5] = np.deg2rad(angle[0]), np.deg2rad(angle[1])
            # data.qpos[8], data.qpos[11] = np.deg2rad(angle[2]), np.deg2rad(angle[3])
            mujoco.mj_inverse(model, data)
            mujoco.mj_step(model, data)

            # if data.contact:
            if data.contact.geom.size != 0:
                print("contact")
            else:
                print("ok")
            # tau = data.qfrc_inverse
            tau_ = data.qfrc_inverse
            tau = tau_/1000
            torques.append(tau.copy().tolist())

            mujoco.mj_step(model, data)
            viewer.sync()
            q_t.append(data.qpos.copy().tolist())
            q_vel_ = data.qvel.copy()/18
            dq_t.append(q_vel_.tolist())
            time_list.append(time.time() - start)

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

print("Simulation completed. Saving data to CSV...")

# Write data to CSV
with open('simulation_data3.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    # Write headers
    writer.writerow(['time', 'qpos', 'qvel', 'torque'])

    # Write rows
    for t, qpos, qvel, torque in zip(time_list, q_t, dq_t, torques):
        writer.writerow([t, qpos, qvel, torque])

print("Data saved to simulation_data3.csv")
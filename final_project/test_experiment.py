import mujoco
import numpy as np
from mujoco import viewer
from itertools import product
import time

model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)
end_time = 2
print("len(data.qpos)", len(data.qpos))

angle_combinations = product(range(0, 10, 2), repeat=13)

measurements_list = []
q_t = []
dq_t = []
#
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     while viewer.is_running() and time.time() - start < end_time:
#         step_start = time.time()
#
#         for angles in angle_combinations:
#             # data.qpos[0], data.qpos[3] = np.deg2rad(angles[0]), np.deg2rad(angles[1])
#             # data.qpos[7], data.qpos[10] = np.deg2rad(angles[2]), np.deg2rad(angles[3])
#             # data.qpos[6] = 0
#             data.qpos[:] = np.deg2rad([angles])
#         # for i in range(15):
#             # data.qpos[6] = 0
#             # # data.qpos[13:13] = [0] * 1
#             # data.qpos[0], data.qpos[3] = np.deg2rad(i), np.deg2rad(i)
#             # data.qpos[7], data.qpos[10] = np.deg2rad(i), np.deg2rad(i)
#
#             # data.qpos[1], data.qpos[4] = np.deg2rad(i), np.deg2rad(i)
#             # data.qpos[8], data.qpos[11] = np.deg2rad(i), np.deg2rad(i)
#             # data.qpos[:] = [0]*13
#
#
#             # print(angles)
#             mujoco.mj_step(model, data)
#             mujoco.mj_inverse(model, data)
#
#             if data.contact:
#                 print('contact')
#             else:
#                 print("ok")
#                 tau = data.qfrc_inverse
#                 measurements_list.append(tau)
#
#             mujoco.mj_step(model, data)
#             viewer.sync()
#             time_until_next_step = model.opt.timestep - (time.time() - step_start)
#             if time_until_next_step > 0:
#                 time.sleep(time_until_next_step)


with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < end_time:
        step_start = time.time()
        for i in range(10):
            data.qpos[:] = np.deg2rad(0)*13
            data.qpos[1], data.qpos[4] = np.deg2rad(i), np.deg2rad(i)
            data.qpos[8], data.qpos[11] = np.deg2rad(i), np.deg2rad(i)

            # data.qpos[0:6] = [np.deg2rad(5)] * 6
            # data.qpos[6] = np.deg2rad(-10)
            # data.qpos[7:13] = [np.deg2rad(5)] * 6
            #
            # mujoco.mj_step(model, data)
            mujoco.mj_inverse(model, data)

            if data.contact:
                print('contact')
            else:
                print("ok")
            tau = data.qfrc_inverse
            measurements_list.append(tau)
            q_t.append(data.qpos.copy())
            dq_t.append(data.qvel.copy())

            mujoco.mj_step(model, data)
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


# Write data to CSV file
with open('torques_data.csv', 'w') as f:
    for measurement in measurements_list:
        f.write(','.join(map(str, measurement)) + '\n')

with open('pos_data.csv', 'w') as f:
    for measurement in q_t:
        f.write(','.join(map(str, measurement)) + '\n')

with open('vel_data.csv', 'w') as f:
    for measurement in dq_t:
        f.write(','.join(map(str, measurement)) + '\n')

# print("Data written to torques_data.csv")
# with open('torques_data.csv', 'w') as f:
#     for tau in measurements_list:
#         f.write(f"{tau[0]}, {tau[1]}, {tau[2]}, {tau[3]}, {tau[4]}, {tau[5]}, {tau[6]}, {tau[7]}, {tau[8]}, {tau[9]}, {tau[10]}, {tau[11]}, {tau[12]} \n")

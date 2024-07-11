# import time
# import mujoco
# import mujoco.viewer
# import numpy as np
# import csv
#
# model = mujoco.MjModel.from_xml_path('example2.xml')
# data = mujoco.MjData(model)
#
#
# def all_conf(i0, i1, i2, i3):
#     if i3 > 360:
#         i3 = 0
#         i2 += 25
#     if i2 > 360:
#         i2 = 0
#         i1 += 25
#     if i1 > 360:
#         i1 = 0
#         i0 += 25
#     if i0 > 360:
#         exit(0)
#     i3 += 25
#     return i0, i1, i2, i3
#
#
# with open(csv_filename, mode='w', newline='') as csv_file:
#     csv_writer = csv.writer(csv_file)
#
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     i0, i1, i2, i3 = 0, 0, 0, 0
#     while viewer.is_running() and time.time() - start < 30:
#         step_start = time.time()
#         i0, i1, i2, i3 = all_conf(i0, i1, i2, i3)
#         data.qpos = np.deg2rad([i0, i1, i2, i3])
#         mujoco.mj_step(model, data)
#         mujoco.mj_inverse(model, data)
#         if data.contact:
#             print('contact')
#         else:
#             with open(csv_filename, mode='a', newline='') as csv_file:
#                 csv_writer = csv.writer(csv_file)
#                 csv_writer.writerow(data.qfrc_inverse)
#
#         viewer.sync()
#
#         time_until_next_step = model.opt.timestep - (time.time() - step_start)
#         if time_until_next_step > 0:
#             time.sleep(time_until_next_step)


import time
import mujoco
import mujoco.viewer
import numpy as np
import csv

model = mujoco.MjModel.from_xml_path('example.xml')
data = mujoco.MjData(model)



csv_filename = 'angles_and_torques6.csv'
with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)

measurements_list=[]

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 10:
        step_start = time.time()
        for q1 in range(0, 360, 5):
            for q2 in range(0, 360, 5):
                for q3 in range(0, 360, 5):
                    data.qpos = np.deg2rad([q1, q2, q3])
                    mujoco.mj_step(model, data)
                    mujoco.mj_inverse(model, data)
                    if data.contact:
                        print('contact')
                    else:
                        with open( 'angles_and_torques6.csv', mode='a', newline='') as csv_file:
                            csv_writer = csv.writer(csv_file)
                            csv_writer.writerow(data.qfrc_inverse)

                    viewer.sync()

                    time_until_next_step = model.opt.timestep - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)

# with open(csv_filename, mode='a', newline='') as csv_file:
#     csv_writer = csv.writer(csv_file)
#     csv_writer.writerow(data.qfrc_inverse)

# with open('angles_and_torques6.csv', 'w', newline='') as file:
#     writer = csv.writer(file)
#     writer.writerow(["Angle1", "Angle2", "Angle3", "Tau1", "Tau2", "Tau3"])
#     writer.writerows(measurements_list)
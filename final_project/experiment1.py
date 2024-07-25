import mujoco
import numpy as np
from mujoco import viewer
from itertools import product
import time

model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)
end_time = 30
print("len(data.qpos)", len(data.qpos))

angle_combinations = product(range(0, 10, 2), repeat=13)

measurements_list = []
q_t = []
dq_t = []

print(len(data.qpos))

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < end_time:
        step_start = time.time()
        # for i in range(30):
        data.qpos[0], data.qpos[3] = np.deg2rad(15), np.deg2rad(15)
        data.qpos[6], data.qpos[9] = np.deg2rad(15), np.deg2rad(15)
        data.qpos[2], data.qpos[5] = np.deg2rad(30), np.deg2rad(30)
        data.qpos[8], data.qpos[11] = np.deg2rad(30), np.deg2rad(30)
        if data.contact:
            print("contact")
        else:
            print("ok")

        mujoco.mj_step(model, data)
        viewer.sync()
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
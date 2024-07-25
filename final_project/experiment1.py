import mujoco
import numpy as np
from mujoco import viewer
import time

model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)
end_time = 10
print("len(data.qpos)", len(data.qpos))

measurements_list = []
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < end_time:
        step_start = time.time()

        for i in range(30):
            data.qpos[0] = np.deg2rad([i])
            data.qpos[0] = np.deg2rad([i])


            mujoco.mj_step(model, data)
            # mujoco.mj_inverse(model, data)
            # if data.contact:
            #     print('contact')
            #     continue
            # else:
            #     tau = data.qfrc_inverse
            #     measurements_list.append([data.qpos, tau])

            mujoco.mj_step(model, data)
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


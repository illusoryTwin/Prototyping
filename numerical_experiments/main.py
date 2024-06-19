import time
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path('example.xml')
data = mujoco.MjData(model)


def check_torques(model_, data_):
        

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Close the viewer automatically after 30 wall-seconds.
        start = time.time()

        while viewer.is_running() and time.time() - start < 30:
            data.qpos = np.deg2rad([15, 15, 15])
            data.qvel = [0, 0, 0]
            data.qacc = [0, 0, 0]





            step_start = time.time()

            mujoco.mj_inverse(model, data)
            print("tau: ", data.qfrc_inverse)

            mujoco.mj_step(model, data)
            print("contact", data.contact.geom.size > 0)

            # if data.contact.geom.size > 0:
                # "collision"


            # data.qvel = [0, 0, 0]
            # data.qacc = []
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

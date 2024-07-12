# import numpy as np
# import sympy as sp
# import matplotlib.pyplot as plt
# import pandas as pd
# import mujoco
# import time
#
#
# model = mujoco.MjModel.from_xml_path('model.xml')
# data = mujoco.MjData(model)
#
# # X_start = np.array([[0, 0, 0.04], []])
# X_start = np.array([0, 0, 0.04])
# X_end = np.array([0, 0.02, 0.02])
# v_max = 0.4
# T = (15 / 8) * ((np.transpose(X_end - X_start) @ (X_end - X_start)) ** 0.5) / v_max
#
# a0 = a1 = a2 = 0
# a3 = 10 / T ** 3
# a4 = -15 / T ** 4
# a5 = 6 / T ** 5
#
# dt = 0.01
# end_timestamp = 1
#
#
# def generate_s_t(t):
#     return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5
#
#
# def generate_s_dot(t):
#     return a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4
#
#
# def generate_s_ddot(t):
#     return 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3
#
#
# def calculate_x_t(s):
#     return X_start + s * (X_end - X_start)
#
#
# def calculate_dot_x(dot_s):
#     return dot_s * (X_end - X_start)
#
#
# def calculate_ddot_x(ddot_s):
#     return ddot_s * (X_end - X_start)
#
#
# def ik_residual(q, pos, model, data, q_prev, body_id):
#     # Set qpos, compute forward kinematics.
#     data.qpos = q
#     data.qvel = np.zeros(q.shape)
#     data.qacc = np.zeros(q.shape)
#
#     mujoco.mj_kinematics(model, data)
#
#     # Position residual.
#     res_pos = data.body(body_id).xpos - pos
#     l1 = 0.00001
#     q_dif = (q - q_prev) / 0.001
#     l2 = 10
#     return np.hstack([res_pos, l1 * q.T @ q, l2 * q_dif])
#
# # body_id = 'wrist'
# body_id = 'end_effector'
# # body_id = 'target'
# end_time = 10
# time_range = np.linspace(0, end_time, 100)
# q_prev = np.zeros(model.nq)  # Initialize q_prev with the correct shape
#
# results = []
# q_t = []
# for t in time_range:
#     print(t)
#     s_t, dot_s, ddot_s = generate_s_t(t), generate_s_dot(t), generate_s_ddot(t)
#     x_t = calculate_x_t(s_t)
#     dot_x = calculate_dot_x(dot_s)
#     ddot_x = calculate_ddot_x(ddot_s)
#     print("t, x_t, dot_x, ddot_x", t, x_t, dot_x, ddot_x)
#     print("t, s_t, dot_s, ddot_s", t, s_t, dot_s, ddot_s)
#
#     #
#     q = data.qpos.copy()
#     opt = ik_residual(q, x_t, model, data, q_prev, body_id)
#     #
#     # # if t < 0.1:
#     # #     print(len(q), opt[:4])
#     q_t.append(opt[:4])
#     # # print("opt", opt)
#     # q_prev = q
#     #
#     # J = np.zeros(3, model.nv)
#     # jacr = np.zeros(3, model.nv)
#     # J_prev = J.copy()
#     # J = mujoco.mj_jac(model, data, J, jacr, pos, body_id)  # - position to which we are aiming
#
#
# # print("q_t", q_t)
#
# import mujoco.viewer
# # Run the simulation and collect data
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     while viewer.is_running() and time.time() - start < end_time:
#         step_start = time.time()
#         for i in range(len(time_range)):
#
#             data.qpos = q_t[i]
#             data.qacc = 0
#             # print("q_t[t]", q_t[i])
#             mujoco.mj_step(model, data)
#
#             viewer.sync()
#
#             time_until_next_step = model.opt.timestep - (time.time() - step_start)
#             if time_until_next_step > 0:
#                 time.sleep(time_until_next_step)
#
#
#
#
#
#
#
# # print("len(q_t)", q_t.shape)
#
#     # mujoco.mj_fwdPosition(model, data)
#     # J = np.zeros((3, model.nv))
#     # mujoco.mj_jacSite(model, data, J, None, 0)
#     # J_pinv = np.linalg.pinv(J)
#     #
#     # dq_t = J_pinv @ dot_x
#     # dJ_dt = (J - J_prev) / dt if t > 0 else np.zeros_like(J)
#     #
#     # ddX_t = ddot_x.reshape((3, 1))
#     # print(f"dJ_dt: {dJ_dt.shape}")
#     # print(f"dX_t: {dot_x.shape}")
#     # print(f"J_pinv: {J_pinv.shape}")
#     # print(f"ddX_t: {ddX_t.shape}")
#     # print(f"dq_t: {dq_t.shape}")
#     # print(f"J: {J.shape}")
#     # print(f"J: {J.shape}")
#     # print(f"dof: {model.nv}")
#     # # ddq_t = J_pinv @ (ddX_t - dJ_dt @ dX_t)
#     # ddq_t = np.ndarray((4, 1))
#     #
#     # # Set state in the data object
#     # data.qpos[:] = q_t
#     # # data.qvel[:] = dq_t.reshape((4, 1))
#     # data.qacc[:] = ddq_t.reshape((4,))
#     #
#     # # Compute inverse dynamics to get joint torques
#     # mujoco.mj_inverse(model, data)
#     # tau_t = data.qfrc_inverse.copy()
#     #
#     # # Record the results
#     # results.append({
#     #     'time': t,
#     #     'qpos': q_t.copy(),
#     #     'qvel': dq_t.copy(),
#     #     'qacc': ddq_t.copy(),
#     #     'tau': tau_t.copy(),
#     #     'X': x_t.copy(),
#     #     'dX': dot_x.copy(),
#     #     'ddX': ddX_t.copy()
#     # })
#     #
#     # # Store the previous Jacobian
#     # J_prev = J.copy()
#     #
#     # # Convert results to DataFrame and save to CSV
#     # df = pd.DataFrame(results)
#     # df.to_csv('robot_trajectory_data.csv', index=False)
#     #
#     # # Plot the results
#     # fig, axs = plt.subplots(4, 1, figsize=(10, 20), sharex=True)
#     #
#     # for i in range(model.nq):
#     #     axs[0].plot(df['time'], df['qpos'].apply(lambda x: x[i]), label=f'Joint {i + 1}')
#     #     axs[1].plot(df['time'], df['qvel'].apply(lambda x: x[i]), label=f'Joint {i + 1}')
#     #     axs[2].plot(df['time'], df['qacc'].apply(lambda x: x[i]), label=f'Joint {i + 1}')
#     #     axs[3].plot(df['time'], df['tau'].apply(lambda x: x[i]), label=f'Joint {i + 1}')
#     #
#     # axs[0].set_ylabel('Position (rad)')
#     # axs[1].set_ylabel('Velocity (rad/s)')
#     # axs[2].set_ylabel('Acceleration (rad/s^2)')
#     # axs[3].set_ylabel('Torque (Nm)')
#     # axs[3].set_xlabel('Time (s)')
#     #
#     # for ax in axs:
#     #     ax.legend()
#     #     ax.grid(True)
#     #
#     # plt.tight_layout()
#     # plt.savefig('robot_trajectory_plots.png')
#     # plt.close()
#
# # import time
# #
# # with mujoco.viewer.launch_passive(model, data) as viewer:
# #     start = time.time()
# #     while viewer.is_running() and time.time() - start < 30:
# #         step_start = time.time()
# #         # print(step_start-start)
# #         mujoco.mj_step(model, data)
# #         x_t =[0, 0, traj.subs(t, step_start-start)]
# #         fk_q = data.body('wrist').xpos.copy()
# #         print(x_t, fk_q)
# #
# #         viewer.sync()
# #
# #         # Rudimentary time keeping, will drift relative to wall clock.
# #         time_until_next_step = model.opt.timestep - (time.time() - step_start)
# #         if time_until_next_step > 0:
# #             time.sleep(time_until_next_step)
# #
# #     import time
# #
# #     with mujoco.viewer.launch_passive(model, data) as viewer:
# #         start = time.time()
# #         while viewer.is_running() and time.time() - start < 30:
# #             step_start = time.time()
# #             # print(step_start-start)
# #             mujoco.mj_step(model, data)
# #             X_t = X_start + s_t * (X_end - X_start)
# #             q_t = df.loc[df['time'] == t, 'qpos'].values[0]
# #             print(len(data.qpos), len(q_t))
# #             # data.qpos[:] = q_t[:3]
# #             viewer.sync()
# #
# #             # Rudimentary time keeping, will drift relative to wall clock.
# #             time_until_next_step = model.opt.timestep - (time.time() - step_start)
# #             if time_until_next_step > 0:
# #                 time.sleep(time_until_next_step)
# #
# #
# #


import time
import mujoco
import mujoco.viewer
import numpy as np
import csv

# Define parameters
amplitude = 0.04  # 4 cm converted to meters
max_velocity = 0.4  # m/s

# Initialize Mujoco model and data
model = mujoco.MjModel.from_xml_path('test_model.xml')
data = mujoco.MjData(model)

j_l = np.zeros((3, 3))
j_r = np.zeros((3, 3))

mujoco.mj_jac(model, data, j_l, j_r)

# Calculate trajectory points
timestep = model.opt.timestep
num_points = int(2 * amplitude / max_velocity / timestep) + 1
time_points = np.linspace(0, 2 * np.pi, num_points)
x_trajectory = amplitude * np.sin(time_points)
y_trajectory = amplitude * np.cos(time_points)

# Main simulation loop
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    for i in range(len(time_points)):

        # Set desired position in cartesian space
        data.site_xpos[0] = [x_trajectory[i], y_trajectory[i], 0]

        # Set desired position in joint space (assuming joint index 0 here)
        data.qpos[0] = x_trajectory[i]
        data.qvel[0] = max_velocity * np.cos(time_points[i])

        # Step forward the simulation
        step_start = time.time()
        mujoco.mj_step(model, data)
        mujoco.mj_inverse(model, data)
        print("Position:", data.qpos[0])
        print("Velocity:", data.qvel[0])
        print("Acceleration:", data.qacc[0])
        print("Cartesian Position:", data.site_xpos[0])
        print("Motor Torques:", data.qfrc_inverse[0])

        viewer.sync()

        # Control the time step
        time_until_next_step = timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

        # Check for simulation timeout
        if time.time() - start >= 30:
            break
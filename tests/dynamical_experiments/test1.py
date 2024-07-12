# import time
# import mujoco
# import mujoco.viewer
# import numpy as np
# import csv
#
# model = mujoco.MjModel.from_xml_path('model.xml')
# data = mujoco.MjData(model)
# measurements_list = []
#
# import sympy as sp
# import numpy as np
#
# t = sp.symbols('t')
# a0, a1, a2, a3, a4, a5 = sp.symbols('a0 a1 a2 a3 a4 a5')
#
# s = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5
# s_prime = sp.diff(s, t)
# s_double_prime = sp.diff(s_prime, t)
#
# print(f"s(t): {s}")
# print(f"s'(t): {s_prime}")
# print(f"s''(t): {s_double_prime}")
#
# # values_at_t_0 = {
# #     's': s.subs(t, 0),
# #     "s'": s_prime.subs(t, 0),
# #     "s''": s_double_prime.subs(t, 0)
# # }
# # print(values_at_t_0)
#
# end_pos = 0.04
# eq1 = sp.Eq(s.subs(t, 0), 0)  # s(0) = 5
# eq2 = sp.Eq(s.subs(t, 0.1875), end_pos)  # s(10) = 50
# eq3 = sp.Eq(s_prime.subs(t, 0), 0)  # s'(5) = 3 (an example additional condition)
# eq4 = sp.Eq(s_prime.subs(t, 0.1875), 0)  # s'(5) = 3 (an example additional condition)
# eq5 = sp.Eq(s_double_prime.subs(t, 0), 0)  # s'(5) = 3 (an example additional condition)
# eq6 = sp.Eq(s_double_prime.subs(t, 0.1875), 0)  # s'(5) = 3 (an example additional condition)
#
# # Solve the system of equations
# solution = sp.solve((eq1, eq2, eq3, eq4, eq5, eq6), (a0, a1, a2, a3, a4, a5))
# print(solution)
#
#
# def calculate_T(X_start, X_end, v_max):
#     T = 15 / 8 * ((np.transpose(X_end - X_start) @ (X_end - X_start)) ** 0.5) / v_max
#     return T
#
#
# X_start = np.array([[0], [0]])
# X_end = np.array([[0.04], [0]])
# v_max = 0.4
#
# T = calculate_T(X_start, X_end, v_max)
# # print(T)
# # T = 0.1875
#
# a3 = 10 / T ** 3
# a4 = -15 / T ** 4
# a5 = 6 / T ** 5
# a0 = a1 = a2 = 0
# s_t = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5
# print(s_t)
# # #
# # #
# # # def trajectory(s, X_start, X_end):
# # #     return X_start + s @ (X_end - X_start)
# # #
# # # X_start = 0
# # # X_end = 0.04
# # # print(X_end - X_start)
# # #
# # # print(trajectory(s_t, X_start, X_end))
# #
# #
# # import sympy as sp
# # import numpy as np
# #
# # # Define symbols
# # t = sp.symbols('t')
# # a0, a1, a2, a3, a4, a5 = sp.symbols('a0 a1 a2 a3 a4 a5')
# #
# # # Define the polynomial s(t)
# # s = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
# #
# # # Calculate the first and second derivatives of s(t)
# # s_prime = sp.diff(s, t)
# # s_double_prime = sp.diff(s_prime, t)
# #
# # # Print the polynomial and its derivatives
# # print(f"s(t): {s}")
# # print(f"s'(t): {s_prime}")
# # print(f"s''(t): {s_double_prime}")
# #
# # # Define the boundary conditions
# # end_pos = 0.04
# # eq1 = sp.Eq(s.subs(t, 0), 0)  # s(0) = 0
# # eq2 = sp.Eq(s.subs(t, 0.1875), end_pos)  # s(0.1875) = 0.04
# # eq3 = sp.Eq(s_prime.subs(t, 0), 0)  # s'(0) = 0
# # eq4 = sp.Eq(s_prime.subs(t, 0.1875), 0)  # s'(0.1875) = 0
# # eq5 = sp.Eq(s_double_prime.subs(t, 0), 0)  # s''(0) = 0
# # eq6 = sp.Eq(s_double_prime.subs(t, 0.1875), 0)  # s''(0.1875) = 0
# #
# # # Solve the system of equations
# # solution = sp.solve((eq1, eq2, eq3, eq4, eq5, eq6), (a0, a1, a2, a3, a4, a5))
# # print(solution)
# #
# # def calculate_T(X_start, X_end, v_max):
# #     T = 15 / 8 * (np.linalg.norm(X_end - X_start)) / v_max
# #     return T
# #
# # X_start = np.array([0, 0])
# # X_end = np.array([0.04, 0])
# # v_max = 0.4
# #
# # # Calculate T
# # T = calculate_T(X_start, X_end, v_max)
# # print(T)
# #
# # # Substitute the coefficients back into the polynomial
# # a3 = 10 / T**3
# # a4 = -15 / T**4
# # a5 = 6 / T**5
# # a0 = a1 = a2 = 0
# # s_t = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
# # print(s_t)
# #
# # def trajectory(s, X_start, X_end):
# #     return X_start + s * (X_end - X_start)
# #
# # # Compute the trajectory
# # X_start = 0
# # X_end = 0.04
# # print(X_end - X_start)
# #
# # # Evaluate trajectory at different values of t
# # t_values = np.linspace(0, T, 100)
# # s_t_values = [s_t.subs(t, val) for val in t_values]
# # trajectory_values = [trajectory(s_val, X_start, X_end) for s_val in s_t_values]
# #
# # # Print the results
# # print(trajectory_values)
# #
# #
# #
# # with mujoco.viewer.launch_passive(model, data) as viewer:
# #     # Close the viewer automatically after 30 wall-seconds.
# #     start = time.time()
# #     while viewer.is_running() and time.time() - start < 30:
# #         step_start = time.time()
# #
# #         # mj_step can be replaced with code that also evaluates
# #         # a policy and applies a control signal before stepping the physics.
# #         mujoco.mj_step(model, data)
# #
# #         for q1 in range(0, 360, 5):
# #             for q2 in range(0, 360, 5):
# #                 for q3 in range(0, 360, 5):
# #                     if data.contact.geom.size > 0:
# #                         print("collision occured")
# #                         mujoco.mj_step(model, data)
# #                         break
# #                     data.qpos[0] = np.deg2rad(q1)
# #                     data.qpos[1] = np.deg2rad(q2)
# #                     data.qpos[2] = np.deg2rad(q3)
# #                     mujoco.mj_step(model, data)
# #
# #                     data.qvel = [0, 0, 0]
# #                     data.qacc = [0, 0, 0]
# #                     mujoco.mj_inverse(model, data)
# #                     tau = data.qfrc_inverse
# #                     measurements_list.append([q1, q2, q3, tau[0], tau[1], tau[2]])
# #                     # print("tau: ", data.qfrc_inverse)
# #
# #                     mujoco.mj_step(model, data)
# #
# #                     # Pick up changes to the physics state, apply perturbations, update options from GUI.
# #                     viewer.sync()
# #
# #                     # Rudimentary time keeping, will drift relative to wall clock.
# #                     time_until_next_step = model.opt.timestep - (time.time() - step_start)
# #                     if time_until_next_step > 0:
# #                         time.sleep(time_until_next_step)
# #
# #
# # with open('angles_and_torques.csv', 'w', newline='') as file:
# #     writer = csv.writer(file)
# #     writer.writerow(["Angle1", "Angle2", "Angle3", "Tau1", "Tau2", "Tau3"])
# #     writer.writerows(measurements_list)
# #
# # print("Data saved to angles_and_torques.csv")
#
#
# # Minimize Rosenbrock function.
# def rosenbrock(x):
#   return np.stack([1 - x[0, :], 10 * (x[1, :] - x[0, :] ** 2)])
#
# x0 = np.array((0.0, 0.0))
# x, rb_trace = mujoco.minimize.least_squares(x0, rosenbrock);
# print(x, rb_trace)


# import time
# import mujoco
# import mujoco.viewer
# import numpy as np
# import csv
#
# model = mujoco.MjModel.from_xml_path('model.xml')
# data = mujoco.MjData(model)
# measurements_list = []
#
# import sympy as sp
# import numpy as np
#
# t = sp.symbols('t')
# a0, a1, a2, a3, a4, a5 = sp.symbols('a0 a1 a2 a3 a4 a5')
#
# s = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5
# s_prime = sp.diff(s, t)
# s_double_prime = sp.diff(s_prime, t)
#
# print(f"s(t): {s}")
# print(f"s'(t): {s_prime}")
# print(f"s''(t): {s_double_prime}")
#
# # values_at_t_0 = {
# #     's': s.subs(t, 0),
# #     "s'": s_prime.subs(t, 0),
# #     "s''": s_double_prime.subs(t, 0)
# # }
# # print(values_at_t_0)
#
# end_pos = 0.04
# eq1 = sp.Eq(s.subs(t, 0), 0)  # s(0) = 5
# eq2 = sp.Eq(s.subs(t, 0.1875), end_pos)  # s(10) = 50
# eq3 = sp.Eq(s_prime.subs(t, 0), 0)  # s'(5) = 3 (an example additional condition)
# eq4 = sp.Eq(s_prime.subs(t, 0.1875), 0)  # s'(5) = 3 (an example additional condition)
# eq5 = sp.Eq(s_double_prime.subs(t, 0), 0)  # s'(5) = 3 (an example additional condition)
# eq6 = sp.Eq(s_double_prime.subs(t, 0.1875), 0)  # s'(5) = 3 (an example additional condition)
#
# # Solve the system of equations
# solution = sp.solve((eq1, eq2, eq3, eq4, eq5, eq6), (a0, a1, a2, a3, a4, a5))
# print(solution)
#


import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import pandas as pd
import mujoco
import time


model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)

X_start = np.array([0, 0, 0.04])
X_end = np.array([0, 0.02, 0.02])
v_max = 0.4
T = (15 / 8) * ((np.transpose(X_end - X_start) @ (X_end - X_start)) ** 0.5) / v_max

print("T", T)

a0 = a1 = a2 = 0
a3 = 10 / T ** 3
a4 = -15 / T ** 4
a5 = 6 / T ** 5

t = sp.symbols('t')
dt = 0.01
end_timestamp = 1


def generate_s_t(t):
    return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5


def generate_s_dot(t):
    return a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4


def generate_s_ddot(t):
    return 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3


def calculate_x_t(s):
    return X_start + s * (X_end - X_start)


def calculate_dot_x(dot_s):
    return dot_s * (X_end - X_start)


def calculate_ddot_x(ddot_s):
    return ddot_s * (X_end - X_start)

site_name = "end_effector"
site_index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)

# Inverse kinematics residual function
def residual(q, x_desired):
    data.qpos[:] = q
    mujoco.mj_fwdPosition(model, data)
    x_current = data.site_xpos[site_index]
    return np.linalg.norm(x_current - x_desired)

# def ik_residual(q, pos, model, data, q_prev, body_id):
#     """Residual for inverse kinematics.
#
#     Args:
#     q: joint angles.
#     pos: target position for the end effector.
#
#     Returns:
#     The residual of the Inverse Kinematics task.
#     """
#
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

from scipy.optimize import minimize



body_id = 'wrist'
# body_id = 'target'
end_time = 10
time_range = np.linspace(0, end_time, 10000)
q_prev = np.zeros(model.nq)  # Initialize q_prev with the correct shape

results = []
q_t = []
for t in time_range:
    s_t, dot_s, ddot_s = generate_s_t(t), generate_s_dot(t), generate_s_ddot(t)
    x_t = calculate_x_t(s_t)
    dot_x = calculate_dot_x(dot_s)
    ddot_x = calculate_ddot_x(ddot_s)
    # print("x_t, dot_x, ddot_x", x_t, dot_x, ddot_x)

    q = data.qpos.copy()
    # q0 = data.qpos.copy()  # Initial guess for the optimization
    res = minimize(residual, q, args=(x_t,), bounds=[(-np.pi, np.pi)]*model.nq)
    q_t.append(res.x)

    # opt = ik_residual(q, x_t, model, data, q_prev, body_id)

    # if t < 0.1:
    #     print(len(q), opt[:4])
    # q_t.append(opt[:4])
    # print("opt", opt)
    q_prev = q

print("q_t", q_t)

import mujoco.viewer
# Run the simulation and collect data
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < end_time:
        step_start = time.time()
        for i in range(len(time_range)):

            data.qpos = q_t[i]
            data.qacc = 0
            # print("q_t[t]", q_t[i])
            mujoco.mj_step(model, data)

            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

# print("len(q_t)", q_t.shape)

    # mujoco.mj_fwdPosition(model, data)
    # J = np.zeros((3, model.nv))
    # mujoco.mj_jacSite(model, data, J, None, 0)
    # J_pinv = np.linalg.pinv(J)
    #
    # dq_t = J_pinv @ dot_x
    # dJ_dt = (J - J_prev) / dt if t > 0 else np.zeros_like(J)
    #
    # ddX_t = ddot_x.reshape((3, 1))
    # print(f"dJ_dt: {dJ_dt.shape}")
    # print(f"dX_t: {dot_x.shape}")
    # print(f"J_pinv: {J_pinv.shape}")
    # print(f"ddX_t: {ddX_t.shape}")
    # print(f"dq_t: {dq_t.shape}")
    # print(f"J: {J.shape}")
    # print(f"J: {J.shape}")
    # print(f"dof: {model.nv}")
    # # ddq_t = J_pinv @ (ddX_t - dJ_dt @ dX_t)
    # ddq_t = np.ndarray((4, 1))
    #
    # # Set state in the data object
    # data.qpos[:] = q_t
    # # data.qvel[:] = dq_t.reshape((4, 1))
    # data.qacc[:] = ddq_t.reshape((4,))
    #
    # # Compute inverse dynamics to get joint torques
    # mujoco.mj_inverse(model, data)
    # tau_t = data.qfrc_inverse.copy()
    #
    # # Record the results
    # results.append({
    #     'time': t,
    #     'qpos': q_t.copy(),
    #     'qvel': dq_t.copy(),
    #     'qacc': ddq_t.copy(),
    #     'tau': tau_t.copy(),
    #     'X': x_t.copy(),
    #     'dX': dot_x.copy(),
    #     'ddX': ddX_t.copy()
    # })
    #
    # # Store the previous Jacobian
    # J_prev = J.copy()
    #
    # # Convert results to DataFrame and save to CSV
    # df = pd.DataFrame(results)
    # df.to_csv('robot_trajectory_data.csv', index=False)
    #
    # # Plot the results
    # fig, axs = plt.subplots(4, 1, figsize=(10, 20), sharex=True)
    #
    # for i in range(model.nq):
    #     axs[0].plot(df['time'], df['qpos'].apply(lambda x: x[i]), label=f'Joint {i + 1}')
    #     axs[1].plot(df['time'], df['qvel'].apply(lambda x: x[i]), label=f'Joint {i + 1}')
    #     axs[2].plot(df['time'], df['qacc'].apply(lambda x: x[i]), label=f'Joint {i + 1}')
    #     axs[3].plot(df['time'], df['tau'].apply(lambda x: x[i]), label=f'Joint {i + 1}')
    #
    # axs[0].set_ylabel('Position (rad)')
    # axs[1].set_ylabel('Velocity (rad/s)')
    # axs[2].set_ylabel('Acceleration (rad/s^2)')
    # axs[3].set_ylabel('Torque (Nm)')
    # axs[3].set_xlabel('Time (s)')
    #
    # for ax in axs:
    #     ax.legend()
    #     ax.grid(True)
    #
    # plt.tight_layout()
    # plt.savefig('robot_trajectory_plots.png')
    # plt.close()

# import time
#
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     while viewer.is_running() and time.time() - start < 30:
#         step_start = time.time()
#         # print(step_start-start)
#         mujoco.mj_step(model, data)
#         x_t =[0, 0, traj.subs(t, step_start-start)]
#         fk_q = data.body('wrist').xpos.copy()
#         print(x_t, fk_q)
#
#         viewer.sync()
#
#         # Rudimentary time keeping, will drift relative to wall clock.
#         time_until_next_step = model.opt.timestep - (time.time() - step_start)
#         if time_until_next_step > 0:
#             time.sleep(time_until_next_step)
#
#     import time
#
#     with mujoco.viewer.launch_passive(model, data) as viewer:
#         start = time.time()
#         while viewer.is_running() and time.time() - start < 30:
#             step_start = time.time()
#             # print(step_start-start)
#             mujoco.mj_step(model, data)
#             X_t = X_start + s_t * (X_end - X_start)
#             q_t = df.loc[df['time'] == t, 'qpos'].values[0]
#             print(len(data.qpos), len(q_t))
#             # data.qpos[:] = q_t[:3]
#             viewer.sync()
#
#             # Rudimentary time keeping, will drift relative to wall clock.
#             time_until_next_step = model.opt.timestep - (time.time() - step_start)
#             if time_until_next_step > 0:
#                 time.sleep(time_until_next_step)
#
#
#

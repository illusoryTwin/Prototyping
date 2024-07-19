import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import pandas as pd
import mujoco
import time
# from mujoco import minimize
import mujoco.viewer
from scipy.optimize import minimize

model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)

# Define the trajectory points
trajectory_points = [
    [0, -0.07, 0],
    [0, 0.07, 0],
    [0, 0, 0.07],
    [0, 0, -0.07]
]
# trajectory_points = [
#     [0, 0, -0.07],
#     [0, 0, 0],
#     [0, 0, 0.07],
#     [0, 0.07, 0.07]
# ]


# Function to generate trajectory coefficients for each segment
def generate_trajectory_coefficients(X_start, X_end, T):
    a0 = a1 = a2 = 0
    a3 = 10 / T ** 3
    a4 = -15 / T ** 4
    a5 = 6 / T ** 5
    return a0, a1, a2, a3, a4, a5


# Functions to generate s, s_dot, and s_ddot
def generate_s_t(t, coeffs):
    a0, a1, a2, a3, a4, a5 = coeffs
    return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5


def generate_s_dot(t, coeffs):
    a0, a1, a2, a3, a4, a5 = coeffs
    return a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4


def generate_s_ddot(t, coeffs):
    a0, a1, a2, a3, a4, a5 = coeffs
    return 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3


# Functions to calculate x_t, dot_x, and ddot_x
def calculate_x_t(X_start, X_end, s):
    return X_start + s * (X_end - X_start)


def calculate_dot_x(X_start, X_end, dot_s):
    return dot_s * (X_end - X_start)


def calculate_ddot_x(X_start, X_end, ddot_s):
    return ddot_s * (X_end - X_start)


# Define time parameters
dt = 0.01
end_time = 10


# IK residual function
# def ik_residual(x, pos):
#     """Residual for inverse kinematics.
#
#     Args:
#     x: joint angles.
#     pos: target position for the end effector.
#
#     Returns:
#     The residual of the Inverse Kinematics task.
#     """
#
#     # Set qpos, compute forward kinematics.
#     data.qpos = x
#     data.qvel = np.zeros(x.shape)
#     data.qacc = np.zeros(x.shape)
#
#     mujoco.mj_kinematics(model, data)
#
#     # Position residual.
#     res_pos = data.body(body_id).xpos - pos
#
#     return res_pos

# Inverse kinematics residual function
def residual(q, x_desired):
    data.qpos[:] = q
    mujoco.mj_fwdPosition(model, data)
    x_current = data.site_xpos[body_id]
    return np.linalg.norm(x_current - x_desired)


# Initialize variables for simulation
body_id = mujoco.mj_name2id(model,  mujoco.mjtObj.mjOBJ_SITE, 'end_effector')
v_max = 0.4
# time_range = np.arange(0, end_time, dt)
# q_prev = np.zeros(model.nq)
q_t = []
dq_t = []
# # data.qpos = np.deg2rad([15, 15, 15, 15])
# Calculate the trajectory for each segment
for i in range(len(trajectory_points) - 1):
    X_start = np.array(trajectory_points[i])
    X_end = np.array(trajectory_points[i + 1])
    T = (15 / 8) * (np.linalg.norm(X_end - X_start) / v_max)
    coeffs = generate_trajectory_coefficients(X_start, X_end, T)
    print(coeffs)

    segment_time_range = np.arange(0, T, dt)
    for t in segment_time_range:
        s_t = generate_s_t(t, coeffs)
        dot_s = generate_s_dot(t, coeffs)
        ddot_s = generate_s_ddot(t, coeffs)

        x_t = calculate_x_t(X_start, X_end, s_t)
        dot_x = calculate_dot_x(X_start, X_end, dot_s)
        ddot_x = calculate_ddot_x(X_start, X_end, ddot_s)

        target_pos = x_t
        # Solve inverse kinematics to find joint angles
        q0 = data.qpos.copy()  # Initial guess for the optimization
        res = minimize(residual, q0, args=(x_t,), bounds=[(-np.pi, np.pi)] * model.nq)
        q_val = res.x
        q_t.append(q_val)

        # Compute Jacobian and its derivative
        mujoco.mj_fwdPosition(model, data)
        J = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, J, None, body_id)
        J_pinv = np.linalg.pinv(J)

        dq_val = J_pinv @ dot_x
        dq_t.append(dq_val)

        # # J = np.zeros((4, model.nv))
        # jacr = np.zeros((3, model.nv))
        # jacp = np.zeros((3, model.nv))
        #
        # # J_prev = J.copy()
        # J = mujoco.mj_jac(model, data, J, jacr, X_end, body_id)  # - position to which we are aiming
        # J_pinv = np.linalg.pinv(J)
        #
        # dq_t = J_pinv @ dot_x


        data.qpos[:] = q_val
        data.qvel[:] = dq_val


#         if t != 0:
#             q0 = data.qpos.copy()
#         else:
#             q0 = np.deg2rad([15, 15, 15, 15])
# # #
#         print("q0", q0)
#         # opt = ik_residual(q0, target_pos, q_prev)
#         # print("opt", opt)
#         ik_target = lambda x : ik_residual(q0, target_pos)
#
#         # ik_target = lambda x : ik_residual(q0, target_pos, q_prev)
#         q, res = minimize.least_squares(q0, ik_target)
#         print("q", q)
#         q_t.append(q)
#         data.qpos = q
#         mujoco.mj_step(model, data)

# #
#         ik_target = lambda x: ik_residual(x, pos=x_t, q_prev=q0)
# #         print("q, res", q, res)
# #         q_prev = q
# #         # q_opt = opt[:4]
# #         # q_new = q+q_opt
# #         # data.qpos = q_new
# #         # q_t.append(q_new)
# #         # q_prev = q
# #
# #
# #         # dJ_dt = (J - J_prev) / dt if t > 0 else np.zeros_like(J)
# #         #
# #         # ddot_x = ddot_x.reshape((3, 1))
# # #
print("q_t", q_t)


with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    data.qpos = np.deg2rad([15, 15, 15, 15])

    while viewer.is_running() and time.time() - start < end_time:
        step_start = time.time()
        for i in range(len(q_t)):
        # for i, q_val in enumerate(q_t):
            data.qpos = q_t[i]
            data.qvel = dq_t[i]
            # data.qacc = 0
            mujoco.mj_step(model, data)
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

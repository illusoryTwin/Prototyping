import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
import pandas as pd
import mujoco
import time
import mujoco.viewer


model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)

# Define the trajectory points
trajectory_points = [
    # [0, 0, 0.04],
    # [0, 0, 0],
    [0, 0, 0.02],
    [0, 0.04, 0.02]
]

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
def ik_residual(q, pos, model, data, q_prev, body_id):
    data.qpos = q
    data.qvel = np.zeros(q.shape)
    data.qacc = np.zeros(q.shape)
    mujoco.mj_kinematics(model, data)
    res_pos = data.body(body_id).xpos - pos
    l1 = 0.00001
    q_dif = (q - q_prev) / 0.001
    l2 = 10
    return np.hstack([res_pos, l1 * q.T @ q, l2 * q_dif])
    # return res_pos


# Initialize variables for simulation
body_id = 'end_effector'
v_max = 0.4
time_range = np.arange(0, end_time, dt)
q_prev = np.zeros(model.nq)
q_t = []

# Calculate the trajectory for each segment
for i in range(len(trajectory_points) - 1):
    X_start = np.array(trajectory_points[i])
    X_end = np.array(trajectory_points[i + 1])
    T = (15 / 8) * (np.linalg.norm(X_end - X_start) / v_max)
    coeffs = generate_trajectory_coefficients(X_start, X_end, T)

    segment_time_range = np.arange(0, T, dt)
    for t in segment_time_range:
        s_t = generate_s_t(t, coeffs)
        dot_s = generate_s_dot(t, coeffs)
        ddot_s = generate_s_ddot(t, coeffs)

        x_t = calculate_x_t(X_start, X_end, s_t)
        dot_x = calculate_dot_x(X_start, X_end, dot_s)
        ddot_x = calculate_ddot_x(X_start, X_end, ddot_s)

        q = data.qpos.copy()
        opt = ik_residual(q, x_t, model, data, q_prev, body_id)
        print("opt", opt)
        #
        # q_opt = opt[:4]
        # q_new = q+q_opt
        # data.qpos = q_new
        # q_t.append(q_new)
        # q_prev = q

        # J = np.zeros(4, model.nv)
        # jacr = np.zeros(4, model.nv)
        # J_prev = J.copy()
        # J = mujoco.mj_jac(model, data, J, jacr, X_end, body_id)  # - position to which we are aiming
        # J_pinv = np.linalg.pinv(J)
        #
        # dq_t = J_pinv @ dot_x
        # dJ_dt = (J - J_prev) / dt if t > 0 else np.zeros_like(J)
        #
        # ddot_x = ddot_x.reshape((3, 1))
#
# print("q_t", q_t)
#
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start = time.time()
#     while viewer.is_running() and time.time() - start < end_time:
#         step_start = time.time()
#         for i, q_val in enumerate(q_t):
#             data.qpos = q_val
#             data.qvel = [1, 1, 1, 1]
#             # data.qacc = 0
#             mujoco.mj_step(model, data)
#             viewer.sync()
#             time_until_next_step = model.opt.timestep - (time.time() - step_start)
#             if time_until_next_step > 0:
#                 time.sleep(time_until_next_step)
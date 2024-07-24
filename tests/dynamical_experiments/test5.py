import mujoco
import mujoco.viewer
from mujoco import minimize
import time
import numpy as np

model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)
end_time = 5

v_max = 0.4
dt = 0.05

# trajectory_points = [[0.01, 0.01, 0.01],
#                      [0.05, 0.01, 0.01],
#                      [0.03, 0.01, -0.01],
#                      [0.03, 0.01, 0.03]]

# trajectory_points = [[0.04, 0.02, 0.06],
#                      [0.04, 0.02, 0.02],
#                      [0.04, 0.00, 0.04],
#                      [0.04, 0.04, 0.04]]

trajectory_points = [[0.6, 0.2, 0.2],
                     [0.6, 0.2, -0.2],
                     [0.6, 0.0, 0.0],
                     [0.6, 0.4, 0.0]]
# print("len(data.qpos)", len(data.qpos))

body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'end_effector')


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
def calculate_x_t(x_start_, x_end_, s_):
    return x_start_ + s_ * (x_end_ - x_start_)


def calculate_dot_x(x_start_, x_end_, dot_s_):
    return dot_s_ * (x_end_ - x_start_)


def calculate_ddot_x(x_start_, x_end_, ddot_s_):
    return ddot_s_ * (x_end_ - x_start_)


def ik_residual(x, target_pos):
    data.qpos[:] = x
    data.qvel = np.zeros(x.shape)
    data.qacc = np.zeros(x.shape)
    mujoco.mj_kinematics(model, data)

    pos_residual = data.body(body_id).xpos - target_pos

    return np.hstack(pos_residual)


q_t = []
dq_t = []
# Calculate the trajectory for each segment
for i in range(len(trajectory_points) - 1):
    X_start = np.array(trajectory_points[i])
    X_end = np.array(trajectory_points[i + 1])
    T = (15 / 8) * (np.linalg.norm(X_end - X_start) / v_max)
    coeffs = generate_trajectory_coefficients(X_start, X_end, T)
    segment_time_range = np.arange(0, T, dt)
    # data.qpos[:] = np.deg2rad([15, 15, 15, 15])
    # print("new interation")
    data.qpos[:] = np.deg2rad([5, 5, 5, 5])
    mujoco.mj_step(model, data)
    print("data.body(body_id).xpos", data.body(body_id).xpos)
    for t in segment_time_range:
        s_t = generate_s_t(t, coeffs)
        dot_s = generate_s_dot(t, coeffs)
        ddot_s = generate_s_ddot(t, coeffs)

        x_t = calculate_x_t(X_start, X_end, s_t)
        dot_x = calculate_dot_x(X_start, X_end, dot_s)
        ddot_x = calculate_ddot_x(X_start, X_end, ddot_s)
        desired_pos = x_t
        # print("ik_residual(desired_pos)", ik_residual(desired_pos), "data.body(body_id).xpos", data.body(body_id).xpos,
        #       "desired_pos", desired_pos)
        q0 = data.qpos.copy()
        ik_target = lambda x: ik_residual(x, desired_pos)
        print(type(ik_target))
        q, _ = minimize.least_squares(q0, ik_target)
        q_t.append(q)

        # V1.0
        # mujoco.mj_step(model, data)
        # J = np.zeros((3, model.nv))
        # mujoco.mj_jacSite(model, data, J, None, body_id)
        # print("jac", J)

        #V3.0
        J_v = np.zeros((3, model.nv), dtype=np.float64)
        J_w = np.zeros((3, model.nv), dtype=np.float64)
        mujoco.mj_jacSite(model, data, J_v, J_w, body_id)
        J_v, J_w = J_v[:, :4], J_w[:, :4]
        J = np.vstack((J_v, J_w))
        dq = np.linalg.pinv(J_v) @ dot_x
        print("dq", dq)
        dq_t.append(dq)


        #V2.0
        # mujoco.mj_step(model, data)
        # jacp = np.zeros((3, model.nv))
        # jacr = np.zeros((3, model.nv))
        # mujoco.mj_step(model, data)
        # mujoco.mj_jacSite(model, data, jacp, jacr, body_id)
        # jac = np.concatenate((jacp[:, :4],
        #                       jacr[:, :4]), axis=0)
        # print("jac", jac)
        # print("np.linalg.pinv(jac)", np.linalg.pinv(jac))

        # dq = np.linalg.pinv(jac) @ dot_x
        # dq = np.linalg.pinv(J) @ dot_x
        # dq_t.append(dq)


with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < end_time:
        step_start = time.time()
        data.qpos[:] = np.deg2rad([5, 5, 5, 5])
        for i in range(len(q_t)):
            data.qpos[:] = q_t[i]
            # data.qvel[:] = [0.1, 0.1, 0.1, 0.1]
            data.qvel[:] = dq_t[i]
            # data.qacc[:] = [1000, 1000, 1000, 1000]
            mujoco.mj_step(model, data)
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

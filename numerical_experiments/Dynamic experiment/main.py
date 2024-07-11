# import time
# import mujoco
# import mujoco.viewer
# import numpy as np
# import csv
# import matplotlib.pyplot as plt
# import seaborn as sns
# import pandas as pd
# import scipy.linalg
#
# # Initialize the MuJoCo model
# model = mujoco.MjModel.from_xml_path('model.xml')
# data = mujoco.MjData(model)
#
# # Define parameters
# X_start = np.array([0, 0, 0])
# X_end = np.array([0.04, 0.04, 0])
# v_max = 0.4  # m/s
#
# # Calculate T based on given formula
# T = (15 / 8) * (np.linalg.norm(X_end - X_start)) / v_max
#
# # Polynomial coefficients
# a0 = a1 = a2 = 0
# a3 = 10 / T ** 3
# a4 = -15 / T ** 4
# a5 = 6 / T ** 5
#
#
# # Trajectory generation functions
# def s(t):
#     return a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
#
#
# def s_dot(t):
#     return a1 + 2 * a2 * t + 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t ** 4
#
#
# def s_ddot(t):
#     return 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3
#
#
# def get_jacobian(data):
#     mujoco.mj_jac(model, data, None, None, None, data.body('wrist'))
#     # mujoco.mj_jac(model, data, data.qpos)
#
#     return data.jacobian
#
#
# def get_jacobian_dot(data):
#     # mujoco.mj_jacDot(model, data, data.qpos, data.qvel)
#     mujoco.mj_jacDot(model, data)
#
#     return data.jacobian_dot
#
#
# def compute_trajectory(t, T, X_start, X_end):
#     s_t = s(t / T)
#     s_dot_t = s_dot(t / T) / T
#     s_ddot_t = s_ddot(t / T) / T ** 2
#
#     X_t = X_start + s_t * (X_end - X_start)
#     X_dot_t = s_dot_t * (X_end - X_start)
#     X_ddot_t = s_ddot_t * (X_end - X_start)
#
#     return X_t, X_dot_t, X_ddot_t
#
#
# with mujoco.viewer.launch_passive(model, data) as viewer:
#     start_time = time.time()
#     for t in np.arange(0, T, model.opt.timestep):
#         step_start = time.time()
#
#         # Compute desired trajectory point
#         X_t, X_dot_t, X_ddot_t = compute_trajectory(t, T, X_start, X_end)
#
#         # Inverse kinematics to find joint positions
#         def cost_function(q):
#             data.qpos = q
#             mujoco.mj_forward(model, data)
#             FK = data.geom_xpos[:3]
#             return np.sum((X_t - FK) ** 2)
#
#
#         qpos_initial = data.qpos.copy()
#         result = scipy.optimize.minimize(cost_function, qpos_initial)
#         if not result.success:
#             continue
#
#         qpos = result.x
#         data.qpos = qpos
#         mujoco.mj_forward(model, data)
#
#         # Get Jacobian and compute velocities/accelerations
#         J = get_jacobian(data)
#         J_dot = get_jacobian_dot(data)
#
#         qvel = np.linalg.pinv(J).dot(X_dot_t)
#         qacc = np.linalg.pinv(J).dot(X_ddot_t - J_dot.dot(X_dot_t))
#
#         data.qvel = qvel
#         data.qacc = qacc
#
#         mujoco.mj_inverse(model, data)
#         tau = data.qfrc_inverse
#
#         # Collect data
#         measurements_list.append([
#             *qpos, *qvel, *qacc, *X_t, *X_dot_t, *X_ddot_t, *tau
#         ])
#
#         viewer.sync()
#
#         time_until_next_step = model.opt.timestep - (time.time() - step_start)
#         if time_until_next_step > 0:
#             time.sleep(time_until_next_step)
#
# # # Save data to CSV
# # headers = [
# #     "Qpos1", "Qpos2", "Qpos3",
# #     "Qvel1", "Qvel2", "Qvel3",
# #     "Qacc1", "Qacc2", "Qacc3",
# #     "CartPosX", "CartPosY", "CartPosZ",
# #     "CartVelX", "CartVelY", "CartVelZ",
# #     "CartAccX", "CartAccY", "CartAccZ",
# #     "Tau1", "Tau2", "Tau3"
# # ]
# #
# # with open('trajectory_data.csv', 'w', newline='') as csv_file:
# #     csv_writer = csv.writer(csv_file)
# #     csv_writer.writerow(headers)
# #     csv_writer.writerows(measurements_list)
# #
# # print("Data saved to trajectory_data.csv")
# #
# # # Load the data from CSV for analysis and plotting
# # df = pd.read_csv('trajectory_data.csv')
# #
# # # Plot the trajectory in Cartesian space
# # plt.figure(figsize=(10, 6))
# # plt.plot(df['CartPosX'], df['CartPosY'], label='Cross Trajectory')
# # plt.title('Robot Trajectory in Cartesian Space')
# # plt.xlabel('X Position (m)')
# # plt.ylabel('Y Position (m)')
# # plt.legend()
# # plt.show()
# #
# # # Plot torques
# # df_melted = df.melt(id_vars=["Qpos1", "Qpos2", "Qpos3"], value_vars=["Tau1", "Tau2", "Tau3"],
# #                     var_name="Joint", value_name="Torque")


import time
import mujoco
import mujoco.viewer
import numpy as np
import csv
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import scipy.optimize

model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)

# Define waypoints
waypoints = [
    np.array([0, 0, 0]),
    np.array([0, 0, -0.04]),
    np.array([0, -0.02, -0.02]),
    np.array([0, 0.02, -0.02]),
]

v_max = 0.4  # m/s
body_id = model.body('wrist').id

def calculate_T(X_start, X_end, v_max):
    return (15 / 8) * (np.linalg.norm(X_end - X_start)) / v_max


def get_polynomial_coefficients(T):
    a0 = a1 = a2 = 0
    a3 = 10 / T ** 3
    a4 = -15 / T ** 4
    a5 = 6 / T ** 5
    return a0, a1, a2, a3, a4, a5


# Trajectory generation functions
def s(t, a0, a1, a2, a3, a4, a5):
    return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5


def s_dot(t, a1, a2, a3, a4, a5):
    return a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4


def s_ddot(t, a2, a3, a4, a5):
    return 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2 + 20 * a5 * t ** 3


def get_jacobian(data, goal):
    jacp = np.zeros((3, model.nv))  # translation jacobian
    jacr = np.zeros((3, model.nv))  # rotational jacobian
    mujoco.mj_jac(model, data, jacp, jacr, goal, body_id)

    return data.jacobian


def get_jacobian_dot(data, goal):
    jacp = np.zeros((3, model.nv))  # translation jacobian
    jacr = np.zeros((3, model.nv))  # rotational jacobian
    mujoco.mj_jacDot(model, data, jacp, jacr, goal, body_id)
    return data.jacobian_dot


# Simulation and data collection
measurements_list = []


def compute_trajectory(t, T, X_start, X_end, a0, a1, a2, a3, a4, a5):
    s_t = s(t / T, a0, a1, a2, a3, a4, a5)
    s_dot_t = s_dot(t / T, a1, a2, a3, a4, a5) / T
    s_ddot_t = s_ddot(t / T, a2, a3, a4, a5) / T ** 2

    X_t = X_start + s_t * (X_end - X_start)
    X_dot_t = s_dot_t * (X_end - X_start)
    X_ddot_t = s_ddot_t * (X_end - X_start)

    return X_t, X_dot_t, X_ddot_t


# Function to compute inverse kinematics using optimization
def inverse_kinematics(X_t):
    def cost_function(q):
        data.qpos = q
        mujoco.mj_forward(model, data)
        FK = data.geom_xpos[:3]
        return np.sum((X_t - FK) ** 2)

    qpos_initial = data.qpos.copy()
    result = scipy.optimize.minimize(cost_function, qpos_initial)
    if not result.success:
        raise ValueError("Inverse kinematics optimization failed.")
    return result.x



with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()

    for i in range(len(waypoints) - 1):
        X_start = waypoints[i]
        X_end = waypoints[i + 1]
        T = calculate_T(X_start, X_end, v_max)
        a0, a1, a2, a3, a4, a5 = get_polynomial_coefficients(T)

        for t in np.arange(0, T, model.opt.timestep):
            step_start = time.time()

            # Compute desired trajectory point
            X_t, X_dot_t, X_ddot_t = compute_trajectory(t, T, X_start, X_end, a0, a1, a2, a3, a4, a5)

            # Inverse kinematics to find joint positions
            data.qpos = inverse_kinematics(X_t)
            mujoco.mj_forward(model, data)

            # Get Jacobian and compute velocities/accelerations
            goal = waypoints[0]
            J = get_jacobian(data, goal)
            J_dot = get_jacobian_dot(data, goal)

            qvel = np.linalg.pinv(J).dot(X_dot_t)
            qacc = np.linalg.pinv(J).dot(X_ddot_t - J_dot.dot(X_dot_t))

            data.qvel = qvel
            data.qacc = qacc

            # mujoco.mj_inverse(model, data)
            # tau = data.qfrc_inverse
            #
            # # Collect data
            # measurements_list.append([
            #     *qpos, *qvel, *qacc, *X_t, *X_dot_t, *X_ddot_t, *tau
            # ])

            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


J = np.zeros(3, model.nv)
jacr = np.zeros(3, model.nv)
J_prev = J.copy()
mj_jac(model, data, J, jacr, pos, body_id) # - position to which we are aiming



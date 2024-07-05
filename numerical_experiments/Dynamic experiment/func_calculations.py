import sympy as sp
import numpy as np
import mujoco
import mujoco.viewer
import time
from mujoco import minimize

# Define symbols
t = sp.symbols('t')
a0, a1, a2, a3, a4, a5 = sp.symbols('a0 a1 a2 a3 a4 a5')

# Define the polynomial s(t)
s = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5

# Calculate the first and second derivatives of s(t)
s_prime = sp.diff(s, t)
s_double_prime = sp.diff(s_prime, t)

# Define the boundary conditions
end_pos = 0.04
eq1 = sp.Eq(s.subs(t, 0), 0)  # s(0) = 0
eq2 = sp.Eq(s.subs(t, 0.1875), end_pos)  # s(0.1875) = 0.04
eq3 = sp.Eq(s_prime.subs(t, 0), 0)  # s'(0) = 0
eq4 = sp.Eq(s_prime.subs(t, 0.1875), 0)  # s'(0.1875) = 0
eq5 = sp.Eq(s_double_prime.subs(t, 0), 0)  # s''(0) = 0
eq6 = sp.Eq(s_double_prime.subs(t, 0.1875), 0)  # s''(0.1875) = 0

# Solve the system of equations
solution = sp.solve((eq1, eq2, eq3, eq4, eq5, eq6), (a0, a1, a2, a3, a4, a5))
print(solution)


def calculate_T(X_start, X_end, v_max):
    T = 15 / 8 * (np.linalg.norm(X_end - X_start)) / v_max
    return T


X_start = np.array([0, 0])
X_end = np.array([0.04, 0])
v_max = 0.4

# Calculate T
T = calculate_T(X_start, X_end, v_max)
print("T:", T)

# Substitute the coefficients back into the polynomial
a3 = 10 / T ** 3
a4 = -15 / T ** 4
a5 = 6 / T ** 5
a0 = a1 = a2 = 0
s_t = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5
print("s(t):", s_t)


def trajectory(s, x_start, x_end):
    return x_start + s * (x_end - x_start)


# Compute the trajectory
X_start = 0
X_end = 0.04
traj = trajectory(s_t, X_start, X_end)
print(traj)
print(traj.subs(t, 1))


model = mujoco.MjModel.from_xml_path('example.xml')
data = mujoco.MjData(model)
#
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()
        # print(step_start-start)
        mujoco.mj_step(model, data)
        x_t =[0, 0, traj.subs(t, step_start-start)]
        fk_q = data.body('wrist').xpos.copy()
        print(x_t, fk_q)

        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

# # # Minimize Rosenbrock function.
# # def rosenbrock(x):
# #   return np.stack([x, x])
# #
# # x0 = np.array((0.0, 0.0))
# # x, rb_trace = minimize.least_squares(x0, rosenbrock)
# # print(x)
# # # mujoco.minimize.least_squares()
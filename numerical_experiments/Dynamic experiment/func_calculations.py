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
# X_start = np.array(([0], [0]))
# X_end = np.array(([0.04], [0]))
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
#
#
# def trajectory(s, X_start, X_end):
#     return X_start + s @ (X_end - X_start)
#
# X_start = 0
# X_end = 0.04
# print(X_end - X_start)
#
# print(trajectory(s_t, X_start, X_end))


import sympy as sp
import numpy as np

# Define symbols
t = sp.symbols('t')
a0, a1, a2, a3, a4, a5 = sp.symbols('a0 a1 a2 a3 a4 a5')

# Define the polynomial s(t)
s = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5

# Calculate the first and second derivatives of s(t)
s_prime = sp.diff(s, t)
s_double_prime = sp.diff(s_prime, t)

# Print the polynomial and its derivatives
print(f"s(t): {s}")
print(f"s'(t): {s_prime}")
print(f"s''(t): {s_double_prime}")

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
print(T)

# Substitute the coefficients back into the polynomial
a3 = 10 / T**3
a4 = -15 / T**4
a5 = 6 / T**5
a0 = a1 = a2 = 0
s_t = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
print(s_t)

def trajectory(s, X_start, X_end):
    return X_start + s * (X_end - X_start)

# Compute the trajectory
X_start = 0
X_end = 0.04
print(X_end - X_start)

# Evaluate trajectory at different values of t
t_values = np.linspace(0, T, 100)
s_t_values = [s_t.subs(t, val) for val in t_values]
trajectory_values = [trajectory(s_val, X_start, X_end) for s_val in s_t_values]

# Print the results
print(trajectory_values)


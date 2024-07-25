import csv
import matplotlib.pyplot as plt
import ast
import numpy as np

time_list = []
qpos_list = []
qvel_list = []
torques_list = []

# Read the CSV file
with open('simulation_data.csv', mode='r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        time_list.append(float(row['time']))
        qpos_list.append(ast.literal_eval(row['qpos']))
        qvel_list.append(ast.literal_eval(row['qvel']))
        torques_list.append(ast.literal_eval(row['torque']))

# Convert lists of lists to numpy arrays for easy indexing
qpos_array = np.array(qpos_list)
qvel_array = np.array(qvel_list)
torques_array = np.array(torques_list)

# Create subplots
fig, axs = plt.subplots(3, 1, figsize=(12, 18), sharex=True)

# Plotting qpos vs time
for i in range(qpos_array.shape[1]):
    axs[0].plot(time_list, qpos_array[:, i], label=f'qpos[{i}]')
axs[0].set_ylabel('Joint Positions (rad)')
axs[0].set_title('Joint Positions vs Time')
axs[0].legend()
axs[0].grid(True)

# Plotting qvel vs time
for i in range(qvel_array.shape[1]):
    axs[1].plot(time_list, qvel_array[:, i], label=f'qvel[{i}]')
axs[1].set_ylabel('Joint Velocities (rad/s)')
axs[1].set_title('Joint Velocities vs Time')
axs[1].legend()
axs[1].grid(True)

# Plotting torques vs time
for i in range(torques_array.shape[1]):
    axs[2].plot(time_list, torques_array[:, i], label=f'torques[{i}]')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Torques (Nm)')
axs[2].set_title('Torques vs Time')
axs[2].legend()
axs[2].grid(True)

plt.show()
import matplotlib.pyplot as plt
import numpy as np

# Initialize lists to store the data
q_t = []
dq_t = []
ddq_t = []
torques = []
end_time = 5

# Open the file and read lines
with open('trajectory_data.txt', 'r') as file:
    lines = file.readlines()

# Process each line
for line in lines:
    # Split the line into individual values and convert them to floats
    values = list(map(float, line.strip().split(',')))

    # Append the values to q_t, dq_t, ddq_t
    q_t.append(values[:4])
    dq_t.append(values[4:8])
    ddq_t.append(values[8:12])
    torques.append(values[12:16])

# Convert lists to numpy arrays for easier manipulation
q_t = np.array(q_t)
dq_t = np.array(dq_t)
ddq_t = np.array(ddq_t)
torques = np.array(torques)

# Generate time data
time_steps = len(q_t)
time = np.linspace(0, end_time, time_steps)

# Create subplots with shared x-axis
fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

# Plot position
for i in range(q_t.shape[1]):
    axs[0].plot(time, q_t[:, i], label=f'Joint {i+1}')
axs[0].set_ylabel('Position (rad)')
axs[0].legend()
axs[0].grid(True)

# Plot velocity
for i in range(dq_t.shape[1]):
    axs[1].plot(time, dq_t[:, i], label=f'Joint {i+1}')
axs[1].set_ylabel('Velocity (rad/s)')
axs[1].legend()
axs[1].grid(True)

# Plot acceleration
for i in range(ddq_t.shape[1]):
    axs[2].plot(time, ddq_t[:, i], label=f'Joint {i+1}')
axs[2].set_ylabel('Acceleration (rad/s^2)')
axs[2].legend()
axs[2].grid(True)

# Plot torque
for i in range(torques.shape[1]):
    axs[3].plot(time, torques[:, i], label=f'Joint {i+1}')
axs[3].set_ylabel('Torque (Nm)')
axs[3].set_xlabel('Time (s)')
axs[3].legend()
axs[3].grid(True)

plt.tight_layout()
plt.show()
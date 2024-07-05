import time
import mujoco
import numpy as np
import itertools
import csv

# Load the model
model = mujoco.MjModel.from_xml_path('example.xml')
data = mujoco.MjData(model)

# Generate all possible configurations
angles = np.arange(0, 360, 5)
configurations = list(itertools.product(angles, repeat=model.nq))
qpos_list = [np.deg2rad(config) for config in configurations]


# Define a function to check collisions
def check_collision(model_, data_):
    mujoco.mj_forward(model_, data_)
    for contact in data_.contact:
        if contact.dist < 0:
            return True
    return False


# Create a list to store results
results = []

# Iterate through all configurations
for qpos in qpos_list:
    data.qpos[:] = qpos
    data.qvel[:] = 0
    data.qacc[:] = 0
    mujoco.mj_inverse(model, data)

    # Check for collision
    if not check_collision(data, model):
        # Store the results
        result = list(qpos) + list(data.qfrc_inverse)
        results.append(result)

# Save the results to a CSV file
header = [f'angle_{i}' for i in range(model.nq)] + [f'torque_{i}' for i in range(model.nq)]
with open('results.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(header)
    writer.writerows(results)

print("Results saved to results.csv")
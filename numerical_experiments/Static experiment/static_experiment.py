import time
import mujoco
import mujoco.viewer
import numpy as np
import csv
from itertools import product
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# Initialize the model
model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)

angle_combinations = product(range(0, 360, 30), repeat=4)

measurements_list = []

# Run the simulation and collect data
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()
        for q1, q2, q3, q4 in angle_combinations:
            data.qpos = np.deg2rad([q1, q2, q3, q4])
            mujoco.mj_step(model, data)
            mujoco.mj_inverse(model, data)
            if data.contact:
                print('contact')
                continue
            tau = data.qfrc_inverse
            measurements_list.append([q1, q2, q3, q4, tau[0], tau[1], tau[2], tau[3]])

            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)



# Save data to CSV
csv_filename = 'angles_and_torques.csv'
with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Angle1", "Angle2", "Angle3", "Angle4", "Tau1", "Tau2", "Tau3", "Tau4"])
    csv_writer.writerows(measurements_list)

print("Data saved to angles_and_torques.csv")

# Read the data from the CSV file
df = pd.read_csv(csv_filename)

# Melt the dataframe for proper plotting with seaborn
df_melted = df.melt(id_vars=["Angle1", "Angle2", "Angle3", "Angle4"],
                    value_vars=["Tau1", "Tau2", "Tau3", "Tau4"],
                    var_name="Joint", value_name="Torque")

# Create the violin plot
plt.figure(figsize=(10, 6))
sns.violinplot(x='Joint', y='Torque', data=df_melted)
plt.title('Distribution of Torques in Different Joints')
plt.xlabel('Joint')
plt.ylabel('Torque')
plt.show()
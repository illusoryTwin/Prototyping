# import matplotlib.pyplot as plt
# import seaborn as sns
# import pandas as pd
#
# df = pd.read_csv('angles_and_torques5.csv')
#
# df_melted = df.melt(id_vars=["Angle1", "Angle2", "Angle3"], value_vars=["Tau1", "Tau2", "Tau3"],
#                     var_name="Joint", value_name="Torque")
#
# plt.figure(figsize=(10, 6))
# sns.violinplot(x='Joint', y='Torque', data=df_melted)
# plt.title('Distribution of torques')
# plt.xlabel('Joint')
# plt.ylabel('Torque')
# plt.show()
#


import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# Load data from CSV file
df = pd.read_csv('angles_and_torques6.csv')

# Plotting
plt.figure(figsize=(10, 6))
sns.violinplot(data=df, inner='quartile')
plt.xlabel('Joint')
plt.ylabel('Torque')
plt.title('Distribution of Torques in Different Joints')
plt.xticks(ticks=[0, 1, 2, 3], labels=['pitch', 'roll', 'yaw', 'elbow'])
plt.grid(True)
plt.tight_layout()
plt.show()
#
# import matplotlib.pyplot as plt
# import seaborn as sns
# import pandas as pd
#
# df = pd.read_csv('angles_and_torques6.csv')
# df_melted = df.melt(id_vars=["Angle1", "Angle2", "Angle3"], value_vars=["Tau1", "Tau2", "Tau3"],
#                     var_name="Joint", value_name="Torque")
#
# plt.figure(figsize=(10, 6))
# sns.violinplot(x='Joint', y='Torque', data=df_melted)
# plt.title('Distribution of torques')
# plt.xlabel('Joint')
# plt.ylabel('Torque')
# plt.show()


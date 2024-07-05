import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
#
# df = pd.read_csv('angles_and_torques1.csv')
# df_melted = df.melt(id_vars=["Angle1", "Angle2", "Angle3"], value_vars=["Tau1", "Tau2", "Tau3"],
#                     var_name="Joint", value_name="Torque")
#
# plt.figure(figsize=(10, 6))
# sns.violinplot(x='Joint', y='Torque', data=df_melted)
# plt.title('Distribution of torques')
# plt.xlabel('Joint')
# plt.ylabel('Torque')
# plt.show()


df = pd.read_csv('angles_and_torques.csv')
# df_melted = df.melt(id_vars=["Angle1", "Angle2", "Angle3"], value_vars=["Tau1", "Tau2", "Tau3"],
#                     var_name="Joint", value_name="Torque")

plt.figure(figsize=(10, 6))
sns.violinplot(x='Joint', y='Torque', data=df)
plt.title('Distribution of torques')
plt.xlabel('Joint')
plt.ylabel('Torque')
plt.show()

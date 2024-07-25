# import mujoco
# import numpy as np
#
# # Load model and data
# model = mujoco.MjModel.from_xml_path('model.xml')
# data = mujoco.MjData(model)
#
# # Extract and print mass of each body
# print("Body Masses:")
# for i in range(model.nbody):
#     print(f"Body {i}: Mass = {model.body_mass[i]}")
#
# # Extract and print inertia matrices
# print("\nBody Inertia Matrices:")
# for i in range(model.nbody):
#     inertia = model.body_inertia[i]
#     print(f"Body {i}: Inertia Matrix = {inertia}")
#
# # Extract and print center of mass positions
# print("\nBody Center of Mass Positions:")
# for i in range(model.nbody):
#     pos = model.body_pos[i]
#     print(f"Body {i}: Center of Mass Position = {pos}")
#
# # Extract and print friction coefficients of each geom
# print("\nGeom Friction Coefficients:")
# for i in range(model.ngeom):
#     friction = model.geom_friction[i]
#     print(f"Geom {i}: Friction Coefficients = {friction}")

import mujoco
import numpy as np

# Load model and data
model = mujoco.MjModel.from_xml_path('model.xml')
data = mujoco.MjData(model)

# Open a file to write the output
with open('model_parameters.txt', 'w') as file:

    # Extract and write mass of each body
    file.write("Body Masses:\n")
    for i in range(model.nbody):
        file.write(f"Body {i}: Mass = {model.body_mass[i]}\n")

    # Extract and write inertia matrices
    file.write("\nBody Inertia Matrices:\n")
    for i in range(model.nbody):
        inertia = model.body_inertia[i]
        file.write(f"Body {i}: Inertia Matrix = {inertia}\n")

    # Extract and write center of mass positions
    file.write("\nBody Center of Mass Positions:\n")
    for i in range(model.nbody):
        pos = model.body_pos[i]
        file.write(f"Body {i}: Center of Mass Position = {pos}\n")

    # Extract and write friction coefficients of each geom
    file.write("\nGeom Friction Coefficients:\n")
    for i in range(model.ngeom):
        friction = model.geom_friction[i]
        file.write(f"Geom {i}: Friction Coefficients = {friction}\n")
# uvms-perching
Create SDF based on desired configuration:
- Run MATLAB script ``threearm_6dof_SDF.m``

Loading configurations:
- ``roslaunch gazebo_ros empty_world.launch``
- ``rosrun gazebo_ros spawn_model -file .../uvms-perching/src/sdf/perch_3arm_qX.sdf -sdf -model robot_name``

Measuring vehicle position and applying force along desired directions:
- ``rosrun uvms-perching deflection_vehicle.py``
  - Script will tell you when steady state is reached
- ``rosrun uvms-perching apply_force.py``
  - When new forces are applied, wait for the position to stabilize

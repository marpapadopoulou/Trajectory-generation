# Traj100k-Dataset
The simulation dataset for Collaborative Trajectron

### Environment

 - pybullet
 - pybullet_data
 - numpy==1.10.1
 - scipy==1.23.5
 - pickle
 - roboticstoolbox-python: https://github.com/petercorke/robotics-toolbox-python.git

### Usage
```
python franka_6dof_grasp.py
```
You can set ```p.connect(p.DIRECT)``` to turn off the GUI.

### Resource
Traj20k: https://drive.google.com/drive/folders/1jg6sj20jbSMuEEYxMvkJHig0tKv2ceus?usp=sharing

Traj2.5k is a mini-version of Traj20k.  Datasets contain the following attributes:
 - **ee_log**: Trajectories represented by the Cartesian position of the end-effector (Main)
 - **joint_pos_log**: Trajectories represented by the joint position of the arm
 - **joint_vel_log**: Velocity information of the trajectories
 - **frequency**: sampling frequency
 - **user_vel**: user's velocity

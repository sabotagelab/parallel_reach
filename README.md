This repository is a public version of the OSU F1/10 research made avaliable for review 

# Overview #

This repository includes mapping, localization, planning, and control algorithms along with hardware level interface packages designed to work with the F1/10 platform.
Used together, these tools enable fully autonomous navigation and racing for the vehicle.

# Dependencies #

This code has a large number of dependencies which are not fully listed here.
This was originally developed on the F1/10 research platform using the Nvidia Jetson TX2 to run everything.
Many configuration steps specific to the jetpack version were adopted.

Some of the larger external dependencies:
  - properly configured ROS for use with C++,Python3,Python3.6
  - specific dependencies for each node (various apt packages, pip installs)
  - various build tools including python-dev and build essential
  - (to run reachability) Hylaa (https://github.com/stanleybak/hylaa) and/or QuickZonoReach(https://github.com/NathanJewell/quickzonoreach/tree/cuda) code in the PATH
  - same hardware, or properly configured launch files and ROS nodes for your specific hardware configuration

# Mapping and Preparation #
It is necessary to create the rasterized map using the cartographer nodes. For using the MPCC it is then also necessary to run some post-processing steps and generate the optimal racing lines etc for that specific map.

# Running the Code #
Once this code is properly configured, these steps, or equivalent (if you have had to create new launch files for you configuration) should start every node needed.
I would reccomend running this code using screen/tmux etc since there are a number of simultaneous windows to monitor. Multiple terminal windows could also be used. Generally, I configured another computer with ROS_MASTER_URI and used that for visualization.

- run rviz on control pc
- ssh into the vehicle
- roslaunch nonlinear_mpc_casadi real_particle_filter.launch
- roslaunch nonlinear_mpc_casadi mpc_car1.launch
- roslaunch reachability quickzono.launch
- add rviz topics to visualization
- create nav waypoint in rviz
- use controller/deadman to start autonomous driving


# Known Issues and Quirks #
By no means exhaustive ...
 - The MPCC uses JIT compilation so takes a while to start
 - Using the QZ_HYBRID reachability mode has bugs in the kernel, use <20 steps to avoid crashes in this mode
 - If the visualization breaks or nav point cannot be set, close rviz and the mpcc node then restart rviz then the mpcc node 

# Contact #
Be in touch if and when there are complications using this code.

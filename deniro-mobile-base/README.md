# Mobile Base
This repository contains code, documentations, and code for using DE NIRO's mobile base and the separate mobile base.

## List of Contents: 
### mbed
---
Contains all the mbed code for low-level control of the mobile base platform. 
The scripts in this folder need not be changed as the controller design can occur as high-level scripts on Raspberry Pi. 
To edit these scripts, you can open them in the Mbed's online compiler and reprogram the Mbed board by connecting it via USB. 

### Emilk_Project
---
Contains all the source code and config files related to Emilk's project for closed loop control of Quickie and self docking (using localisation).
Inside this directory the folders **'bosch_imu_emilk'** and **'self_docking'** contain the files for the controllers and self docking routine, respectively.
### Running controllers
___
In addition to the above packages, you also need to include in the RPi's catkin_workspace the pacakge **'pid'** found in the repository to utilise the controllers; this is already installed in Quickie's RPi with the additional source code.
#### Angular Position (rotation on spot)
Execute the launch file **orientation_controller_onspot_v2.launch** found in Quickie's RPi. The controller node is developed with the **rqt_reconfigure** package, which launches a GUI that allows real-time adjustment of the controller gains (Kp,Ki and Kd) by running the following command:

	rosrun rqt_reconfigure rqt_reconfigure

The gains can be changed via the scroll bar or directly typing the value in the text box next to it. Note that the scale needs to be adjusted as required.
The default values assigned during the project are: Kp=1, Ki=0, Kp=0.072 .

#### Angular Position (correcting deviations from straight path/line)
Execute the launch file **position_controller_straightline.launch** found in Quickie's RPi. Just as before, a GUI can be launch allowing real-time adjustment of the gains with the above command. The default values of the gains are: Kp=1.5, Ki=4, Kd=1.5 .

#### Angular velocity (about axis perpendicular to the plane of motion)
Execute the lauch file **angular_velocity_controller.launch** found in Quickie's RPi. Again, the GUI to adjust the controller gains in real-time can be launch with the above command. The default values assigned to the gains are: Kp=30, Ki=20, Kd=0 .

For more information on the pid nodes, please check **'pid'** ROS package documentation.

### Running self docking
___
To get the source files needed for the self docking routine download the **self_docking** package from the GitLab repository into Quickie's on-board computer, which is connected with the Lidar and PSeye camera. One of the two HP laptops in the lab (not the bulky one) was used during the project for this purpose. For the docking routine, notice that packages **'cv_camera'** and **'hector_mapping'** are needed in the catkin workspace of the on-board computer. At the time of the project these were installed in the HP laptop.

First, using two different terminals run the following:

	roslaunch self_docking self_docking.launch
	roslaunch self_docking sensor_fusion.launch

Open another terminal and type

	roslaunch bosch_imu_emilk controller_docking.launch

in order to run the low-level loop.

At this point you should have different nodes running, including the RVIZ window showing you Quickie and the location of the tags in the map (recall that two fiducial markers are needed). To start the docking routine, execute the following two scripts:

	rosrun self_docking imu_theta_remap.py
	rosrun self_docking docking_action_client.py

and Quickie should start approaching the docking station as long as the tags were detected at initialisation.

If unsure whether the all required nodes are running, please check the corresponding ROS graph in Emilk's master thesis (Figure 4.10).
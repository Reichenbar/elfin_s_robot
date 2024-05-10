# Configuration
* The password for Ubuntu is Admin1234.
* Use real time kernel `5.15.0-94-generic`. If you use other kernels, there may be errors when you compile in the workspace (e.g., the command `catkin_make`).
* Use the workspace `elfin_ws` which contains our self-developed packages. The workspace `softman_ws` only contains the packages from [Han's Robot](https://github.com/hans-robot/elfin_s_robot).
* Check the parameters in the `elfin_drivers.yaml` file. The parameter `elfin_ethernet_name` should be the same as the Ethernet name shown in Ubuntu Settings-Network.

# Preparation
1. Turn on the switch to power the robot and control box. Press the **POWER** button on the control box to turn on the robot.

2. Get access to the user control interface using the tablet or PC. Connect to Wifi NA308020005. Type 192.168.2.1/dist in the browser search bar.
* Username: admin
* Password: admin

3. Enter the **Run** interface and ensure the robot is closed. If not, press the red **Close** button in the top right corner to close it. Note that you can directly skip to Step 7 if the **Simulation** box in the **Run** interface has been checked.

4. Ensure the robot is connected to the control box with the black LAN cable.

5. Check the **Simulation** box in the bottom right corner.

6. Unplug the black LAN cable from the control box and connect it to PC.

7. Ensure the black LAN cable is connected to PC. If not, connect them.

8. Press the green **Open** button and then the green **Enable** button in the top right corner.

# Terminal input

9. In the first terminal:
```
cd softman_ws
source devel/setup.bash
roslaunch elfin_robot_bringup elfin_s20_bringup.launch
```

10. In the second terminal:
```
cd softman_ws
sudo chrt 10 bash
source devel/setup.bash
roslaunch elfin_robot_bringup elfin_ros_control.launch
```

11. In the third terminal:
```
cd softman_ws
source devel/setup.bash
roslaunch elfin_s20_with_base_gripper_moveit_config moveit_planning_execution.launch
```

12. In the fourth terminal:
```
cd softman_ws
source devel/setup.bash
roslaunch elfin_basic_api elfin_basic_api.launch
```
If "Elfin Control Panel" interface is not shown, check the permission of `elfin_gui.py` file and eusure it is executable.

13. Enable the servos of Elfin with "Elfin Control Panel" interface: if there is no "Warning", just press the "Servo On" button to enable the robot. If there is "Warning", press the "Clear Fault" button first and then press the "Servo On" button.

14. Use MoveIt! RViz plugin for motion planning.

15. If you want to control the robot using scripts, in the fifth terminal:
```
cd softman_ws
source devel/setup.bash
rosrun elfin_grasp simulation_demo.py
```

This script sets server request functions and provides the main function to use servers. You can modify it based on your demand.

# Turning off

16. Before turning the robot off, you should press the "Servo Off" button to disable the robot.

17. Press the **Disable**, **Close**, and **Shut down** buttons in sequence.

18. Turn off the switch.

# Introduction to self-developed elfin_grasp package

* `collision_objects`: .stl files of collision objects
* `launch/elfin_s20_with_base_gripper_simulation.launch`: simulation launch file for elfin_s20 with base and gripper. If you do simulation, you only need to launch this file instead of launching four files in real robot application as introduced in Step 9-12.
* `launch/elfin_s20_with_gripper_simulation.launch`: simulation launch file for elfin_s20 with gripper.
* `scripts/elfin_with_base_gripper_server.py`: server file for elfin_s20 with base and gripper. It contains three servers to add collsion objects to the planning scene, remove them, and move base. Note that this file will not be launched in real robot application since these servers are not needed. The servers for reaching desired joint position and end-effector pose are launched in the package `elfin_basic_api`.
* `scripts/elfin_with_gripper_server.py`: server file for elfin_s20 with gripper. It contains two servers to add collsion objects to the planning scene and remove them.
* `scripts/simulation_demo.py`: set server request functions and provide the main function to use servers
* `srv`: .srv files for servers
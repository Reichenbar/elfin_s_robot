# Configuration
Modify the parameters in the `elfin_drivers.yaml` file.

# Preparation
1. Turn on the switch. Ensure the robot is connected to the control box with the black LAN cable and press the **POWER** button on the control box to turn on the robot.

2. Get access to the user control interface (Wifi: NA308020005, 192.168.2.1/dist).
* Username: admin
* Password: admin

3. Enter the **Run** interface and ensure the robot is closed. If not, press the red **Close** button in the top right corner to close it.

4. Check the **Simulation** box in the bottom right corner.

5. Unplug the black LAN cable from the control box and connect it to PC.

6. Press the green **Open** button and then the green **Enable** button in the top right corner.

# Terminal input

7. In the first terminal:
```
cd softman_ws
source devel/setup.bash
roslaunch elfin_robot_bringup elfin_s20_bringup.launch
```

8. In the second terminal:
```
cd softman_ws
sudo chrt 10 bash
source devel/setup.bash
roslaunch elfin_robot_bringup elfin_ros_control.launch
```

9. In the third terminal:
```
cd softman_ws
source devel/setup.bash
roslaunch elfin_s20_moveit_config moveit_planning_execution.launch
```

10. In the fourth terminal:
```
cd softman_ws
source devel/setup.bash
roslaunch elfin_basic_api elfin_basic_api.launch
```
If "Elfin Control Panel" interface is not shown, check the permission of `elfin_gui.py` file and eusure it is executable.

11. Enable the servos of Elfin with "Elfin Control Panel" interface: if there is no "Warning", just press the "Servo On" button to enable the robot. If there is "Warning", press the "Clear Fault" button first and then press the "Servo On" button.

12. Use MoveIt! RViz plugin for motion planning.

# Turning off

13. Before turning the robot off, you should press the "Servo Off" button to disable the robot.

14. Press the **Disable**, **Close**, and **Shut down** buttons in sequence.

15. Turn off the switch.
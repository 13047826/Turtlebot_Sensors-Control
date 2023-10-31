
# INTEGRATED PROJECT CODE



## INSTRUCTIONS ON CODE EXECUTION

1. Download turtlebot_ws which contains the custom rospackage (leader_follower_environment) and the turtlebot packages
2. Open a terminal and run the following command to start the simulation environment:
```bash
roslaunch leader_follower_environment enviro.launch use_rectangle:=true marker_blue:=true
```
3. Open another terminal and run the following to teleoperate the leader turtlebot:
```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/Leader/cmd_vel
```
4. Download the "Integrated Code" folder, and run the main.m



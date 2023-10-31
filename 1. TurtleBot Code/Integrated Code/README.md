
# INTEGRATED PROJECT CODE
## :bookmark_tabs:INSTRUCTIONS ON CODE EXECUTION

1. Download turtlebot_ws which contains the custom rospackage (leader_follower_environment) and the turtlebot packages
2. Open a terminal and run the following command to start the simulation environment:
```bash
roslaunch leader_follower_environment enviro.launch use_rectangle:=true marker_blue:=true
```
The Follower robot should be equipped with the raspberry pi camera, and the Leader robot should have a blue square marker attached.

3. Open another terminal and run the following to teleoperate the leader turtlebot:
```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/Leader/cmd_vel
```
4. Download the "Integrated Code" folder and run "main.m".
5. Use the teleoperation to move the Leader turtlebot around and the Follower should move accordingly.

## :bulb:CODE DESIGN

### FLOWCHART
The following flowchart summarises the general flow of the group's code algorithm.

![flowchart](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/544bf117-3ce7-4dc7-881e-292591cfd00d)

### :loop:CLOSED CONTROL LOOP
This is a closed control loop diagram of our visual system.
![closedControlLoop_not_transparent](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/836277a0-4991-4398-8547-f3951d369323)

We have two controllers:
1. IBVS mode: uses error e(t) to compute camera velocity vc
![closedLoopIBVS](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/8e2f2f69-1cc8-4303-84b0-27f766382c2f)
2. PID mode: uses error e(t) to feed to PID Controller Function 
![closedControlPID](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/8f50f699-eaf3-4fe8-8e19-b5bc448ae930)

- :chart_with_upwards_trend:PID Tuning was applied to find the optimal K gain values
![angularPIDTuning](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/5cdabd95-ce3f-4ba8-8d82-e630a89ec412)
![linearPIDTuning](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/add203d1-4c34-4746-9d60-39042e7f3192)

### CLASSES
Having three classes improves modularity and allows object oriented programming (OOB). The three classes include:
-Camera Class
-Control Class
-Laser Class
> [!NOTE]
> For more details on properties and functions, refer to the code commentaries.

![classes](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/294fc7dd-08e2-43d3-bf16-e84dd92e632d)

## :video_camera:Demonstration Videos
1. [41014 Sensors and Control: Group 7.1 Demo Video](https://www.youtube.com/watch?v=UcmZm3NXcrM)
2. [Multi Turtlebot3 Simulation Setup](https://www.youtube.com/watch?v=QhhJ4LpdXY4)
3. [Folower Tracking Leader Turtlebot (Rotation only)](https://www.youtube.com/watch?v=MffxgPTSN6g)
4. [Feature Extraction: Algorithm (Harris Corners)](https://www.youtube.com/watch?v=2sCc-dFE8mQ)
5. [Feature Extraction: Algorithm (SURF)](https://www.youtube.com/watch?v=4pxhQeXszO8)
6. [Image Based Visual Servoing (IBVS) Simulation Demonstration](https://www.youtube.com/watch?v=Sx4_dnnhS38)
7. [Proportional Integra Derivative (PID) Controller Simulation Demonstration](https://www.youtube.com/watch?v=L5AV1vUC-PI)
8. [PID Tuning Demonstration](https://www.youtube.com/watch?v=eC_iE5G5Eig)
9. [Testing feature extraction and rotational tracking](https://www.youtube.com/watch?v=Bw8aVq7_hbU)
   

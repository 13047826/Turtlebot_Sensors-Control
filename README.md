# :turtle: Turtlebot_Sensors-Control

Jieun Hong 13047826 <br>
Melody Hood 13560901 <br>
Naga Bonu 13908948 <br>

> [!NOTE]
> All teamembers have equally contributed towards creating the code

## Fully Integrated System Code
Please refer the the README.md in [TurtleBot Code/Integrated Code](TurtleBot%20Code/Integrated%20Code)

## :bookmark_tabs: Version Control
> [!IMPORTANT]
> Remember to `git pull` before each session! <br>
> When copying packages from your local directory, remove .git & .github folders (<kbd> Ctrl </kbd> + <kbd> H </kbd> will show these folders) <br>

### Git Process
`git clone /SSH Link/` copies the repository to the current directory <br>
`git pull` updates the local repository at the current directory <br>
`git add` indicates which files you want to update in the online repository <br>
`git commit -m "Message"` commits the changes to the online repository<br>
`git push` updates the online repository <br>

### Catkin Process
To install packages see this [ROS tutorial](http://wiki.ros.org/catkin/Tutorials/using_a_workspace)

## :space_invader: Starting the Simulation Environment

1. In linux terminal, launch:
  - Default (White Cylinder Marker): `roslaunch leader_follower_environment enviro.launch` <br>
    > [!NOTE]  
    > New Environment Options have been updated. You can now pass arguments (`use_rectangle` and `marker_blue`) to change marker type and colour. <br>
    > Some options are as follows: <br>
    > Blue Cylinder Marker: `roslaunch leader_follower_environment enviro.launch marker_blue:=true`  <br>
    > White Rectangle Marker: `roslaunch leader_follower_environment enviro.launch use_rectangle:=true`  <br>
    > Blue Rectangle Marker: `roslaunch leader_follower_environment enviro.launch use_rectangle:=true marker_blue:=true`  <br>

  ![image](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/f0699cc8-1042-4d99-a260-8b9fb39f513c)

2. To run RViz, in a new terminal, launch:
  - `rviz`
  - Change 'Fixed Frame' to 'base_footprint' from the dropdown
    ![image](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/cfda1153-db12-4c16-b58e-2278d0f8ebf6)

  - Add a RobotModel (optional: rename Display Name)
    
    ![image](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/33030d3f-eba6-4de9-b6c6-d80cf6088ad3)

  - Under the 'Robot_Description' parameter, type /Follower/robot_description
    ![image](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/bfbeb392-9271-4ab5-9482-c14bff0e7933)

  - Add a Camera setting Image Topic to /Follower/camera/image_raw
    ![image](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/a871c3e2-41d1-42fe-8932-95b8fc56991b)
    
  - We can also add other sensors such as laserscan
    ![image](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/ce726c78-7751-4339-a4b2-93e8815009cd)

3. To teleoperate the Leader Turtlebot3, launch the following in a new terminal:
  - `rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/Leader/cmd_vel`
    ![image](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/534dc46c-7a21-4ab9-a35e-466b5f25dbd4)

4. Matlab is used to control the Follower Turtlebot3 (To be updated).

### Video of Complete Setup

[https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/8eb17093-9635-436b-9df2-dc9470652422](https://github.com/13047826/Turtlebot_Sensors-Control/assets/88377568/a0e748a2-8f1e-4535-b199-55fb119baf4d)

## :dart: Weekly Goals
> [!NOTE]
> Weekly goals are set according to the Team's Gantt Chart<sub>[Project Proposal pg9](https://studentutsedu.sharepoint.com/:b:/s/SensorsControl433/EWT4FWaFJzBEnYt4l1wyZAoBesYJsXxdT7zrp4fGAdr2Jw?e=0khZb6)</sub>

### Week 6: Simulation Environment Setup<sup>17 September 2023</sup> 
- [x] Create a custom package 'leader_follower_environment' which spawns two turtlebots in empty world.
  > [!NOTE]
  > To launch environment:
  > `roslaunch leader_follower_environment enviro.launch`
- [X] Add a marker to the Leader Turtlebot.
- [x] Add a camera to the Follower Turtlebot.
- [x] Complete Leader Turtlebot teleoperation.
  > [!NOTE]
  > To teleoperate using teleop_twist_keyboard:
  > `rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/Leader/cmd_vel`
- [x] Test moving Follower Turtlebot from Matlab.

### Week 7: Feature Extraction<sup>24 September 2023</sup>
- [x] Explore Feature Extraction Methods
- [x] Test & compare different feature extraction methods
- [x] Integrate to Follower Turtlebot

### Week 8: Initial Motor Control<sup>01 October 2023</sup>
- [x] Complete week 7 if incomplete
- [x] Initialise Leader & Follower interaction

### Week 9: Leader-Follower Algorithm<sup>08 October 2023</sup>
- [x] Continue Leader & Follower interaction
- [x] Complete a fully integrated Leader-Follower Algorithm

### Week 10: Project Code + Demo Video<sup>15 October 2023</sup>
- [x] Finalise code structure and readme files
- [x] Collect image/video recordings to prepare for demo video (save inside [Teams>Documents>2.Deliverables>2.Demo Video>Resources folder](https://studentutsedu.sharepoint.com/:f:/r/sites/SensorsControl433/Shared%20Documents/General/2.%20Deliverables/2.%20Demo%20Video/Resources?csf=1&web=1&e=aKBUEE))
- [x] Practice + Complete Demo Video

# :turtle: Turtlebot_Sensors-Control

Jieun Hong 13047826 <br>
Melody Hood 13560901 <br>
Naga Bonu 13908948 <br>

## Version Control
> [!IMPORTANT]
> Remember to `git pull` before each session! <br>

### Git Process
`git clone /SSH Link/` copies the repository to the current directory <br>
`git pull` updates the local repository at the current directory <br>
`git add` indicates which files you want to update in the online repository <br>
`git commit -m "Message"` commits the changes to the online repository<br>
`git push` updates the online repository <br>

### Catkin Process
To install packages see this [ROS tutorial](http://wiki.ros.org/catkin/Tutorials/using_a_workspace)

## Weekly Goals
> [!NOTE]
> Weekly goals are set according to the Team's Gantt Chart

### Week 6: Simulation Environment Setup
- [x] Create a custom package 'leader_follower_environment' which spawns two turtlebots in empty world.
  > [!NOTE]
  > To launch environment:
  > `roslaunch leader_follower_environment environment.launch`
- [ ] Add a marker to the Leader Turtlebot.
- [x] Add a camera to the Follower Turtlebot.
- [x] Complete Leader Turtlebot teleoperation.
  > [!NOTE]
  > To teleoperate using teleop_twist_keyboard:
  > `rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtlebot1/cmd_vel`
- [x] Test moving Follower Turtlebot from Matlab.

### Week 7: Feature Extraction
- [ ] Explore Feature Extraction Methods
- [ ] Test & compare different feature extraction methods
- [ ] Integrate to Follower Turtlebot

### Week 8: Initial Motor Control
- [ ] Complete week 7 if incomplete
- [ ] Initialise Leader & Follower interaction

### Week 9: Leader-Follower Algorithm
- [ ] Continue Leader & Follower interaction
- [ ] Complete a fully integrated Leader-Follower Algorithm

### Week 10: Project Code + Demo Video
- [ ] Finalise code structure and readme files
- [ ] Collect image/video recordings for the demo video (save inside Teams>Documents>2.Deliverables>2.Demo Video>Resources folder)
- [ ] Complete Demo Video

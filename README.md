# :turtle: Turtlebot_Sensors-Control

Jieun Hong 13047826 <br>
Melody Hood 13560901 <br>
Naga Bonu 13908948 <br>

## Version Control
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

## Weekly Goals
> [!NOTE]
> Weekly goals are set according to the Team's Gantt Chart<sub>[Project Proposal pg9](https://studentutsedu.sharepoint.com/:b:/s/SensorsControl433/EWT4FWaFJzBEnYt4l1wyZAoBesYJsXxdT7zrp4fGAdr2Jw?e=0khZb6)</sub>

### Week 6: Simulation Environment Setup<sup>17 September 2023</sup> 
- [x] Create a custom package 'leader_follower_environment' which spawns two turtlebots in empty world.
  > [!NOTE]
  > To launch environment:
  > `roslaunch leader_follower_environment environment.launch`
- [X] Add a marker to the Leader Turtlebot.
- [x] Add a camera to the Follower Turtlebot.
- [x] Complete Leader Turtlebot teleoperation.
  > [!NOTE]
  > To teleoperate using teleop_twist_keyboard:
  > `rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtlebot1/cmd_vel`
- [x] Test moving Follower Turtlebot from Matlab.

### Week 7: Feature Extraction<sup>24 September 2023</sup>
- [ ] Explore Feature Extraction Methods
- [ ] Test & compare different feature extraction methods
- [ ] Integrate to Follower Turtlebot

### Week 8: Initial Motor Control<sup>01 October 2023</sup>
- [ ] Complete week 7 if incomplete
- [ ] Initialise Leader & Follower interaction

### Week 9: Leader-Follower Algorithm<sup>08 October 2023</sup>
- [ ] Continue Leader & Follower interaction
- [ ] Complete a fully integrated Leader-Follower Algorithm

### Week 10: Project Code + Demo Video<sup>15 October 2023</sup>
- [ ] Finalise code structure and readme files
- [ ] Collect image/video recordings for the demo video (save inside [Teams>Documents>2.Deliverables>2.Demo Video>Resources folder](https://studentutsedu.sharepoint.com/:f:/r/sites/SensorsControl433/Shared%20Documents/General/2.%20Deliverables/2.%20Demo%20Video/Resources?csf=1&web=1&e=aKBUEE))
- [ ] Complete Demo Video

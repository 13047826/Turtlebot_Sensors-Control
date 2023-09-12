# :turtle: Turtlebot_Sensors-Control

Jieun Hong 13047826 <br>
Melody Hood 13560901 <br>
Naga Bonu 13908948 <br>

## Version Control
Remember to `git pull` before each session! <br>

### Git Process
`git clone /SSH Link/` copies the repository to the current directory <br>
`git pull` updates the local repository at the current directory <br>
`git add` indicates which files you want to update in the online repository <br>
`git commit -m "Message"` commits the changes to the online repository<br>
`git push` updates the online repository <br>

## Weekly Goals
### Week 6: Simulation Environment Setup
- [x] Create a custom package 'leader_follower_environment' which spawns two turtlebots in empty world.
  > [!NOTE]
  > To launch environment:
  > `roslaunch leader_follower_environment environment.launch`
- [ ] Add a marker to the Leader Turtlebot.
- [x] Complete Leader Turtlebot teleoperation.
  > [!NOTE]
  > To teleoperate using teleop_twist_keyboard:
  > `rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtlebot1/cmd_vel`

# Color Detection
This directory contains a robot configuration where the team one actor seeks
the blue ball and the team one actor seeks the red ball. After finding the ball
associatd with its team, both robots attempt to shoot their balls into detected 
soccer goals in the world.

To run this, launch the gazebo world and associated robots in one terminal: 
```
./launch_world.sh
```
In another terminal, launch the two detection nodes associated with the robots:
Python 2:
```
./launch_teams.sh
```
Python 3:
```
./python3_launch_teams.sh
````

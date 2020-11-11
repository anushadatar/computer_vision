This directory contains a robot configuration where both the team one actor
and team two actors run an ImageNet tensorflow model to classify objects
visible in front of the robot. Currently, this model prints detected values
and publishes them to topics associated with the robot number and team.

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
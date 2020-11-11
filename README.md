# FA20 Computational Robotics: Computer Vision
*Anusha Datar and Siddharth Garimella*

## Setup

May need to look through launch scripts to fix python version. In either the color-detection or object-recognition folders, run the following:
```
sudo apt-get install ros-<distrib>-gazebo-ros ros-<distrib>-turtlebot
chmod u+x <all scripts>
./launch_world.sh
./launch_teams.sh
```

## Goal
Our original objective was to involve multiple robots and detect features unique to each one, namely faces, on attached signs. We ran into a variety of hurdles implementing this, primarily in manipulating the structure of each neato model towards a non-trivial CV problem, so we split up our project into two modules. 

Our first work uses basic color recognition and contour detection techniques in its application of CV, but involves multiple robots needing to make non-obvious choices about which directions to move in based on this information. We then turned our attention to integrating advanced image detection models into our control of the robot, using Tensorflow to identify specific, real-world objects in Gazebo, and moving a Neato around based on detection output.

## Architecture and Solution

#### Color Recognition



## Design Decisions
Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.

## Reflection

### Challenges
What if any challenges did you face along the way?

### Further Extensions
What would you do to improve your project if you had more time?

### Key Takeaways
Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

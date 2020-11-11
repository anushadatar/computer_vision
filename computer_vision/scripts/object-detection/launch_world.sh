#!/bin/bash
roslaunch computer_vision object-recognition-main.launch
pkill gzserver
# TODO: Replace this with a world that has different objects instead of different items.

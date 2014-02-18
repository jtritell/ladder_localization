ladder_shaper
=============

Shapes a ladder using interactive markers and exports the parameters.
The ladder is supposed to be fit to sensor data, particularly pointclouds.
Interactive markers are used to resze the width, height, and spacing of ladder components.
Export functionality converts the ladder to a XML file readable by the hubo planner.

Usage:

catkin_make
roslaunch ladder_localization fit_ladder
rosrun rviz rviz

Alternatively, use the ladder_ui package to see parameters in real time

This package has been tested onscreen, but not yet with pointcloud data.

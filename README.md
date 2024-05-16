# RRT Star for Path Planning in ROS

This repository implements the RRT* (Rapidly-exploring Random Tree) path planning algorithm using C++ with ROS (Robot Operating System). RRT* is a popular algorithm for robot path planning in environments with obstacles.

## Introduction
The RRT* algorithm efficiently finds a feasible path from a start point to a goal point in a given environment. It explores the state space by incrementally building a tree of possible paths, biasing towards unexplored areas and optimizing the path length. This implementation subscribes to the map, initial pose, and goal topics published by RViz to plan a path. It visualizes the map, initial pose, and goal using OpenCV and publishes the calculated path.

## Dependencies
- ROS Noetic
- OpenCV
- C++11 compiler

## Installation
1. Build a ROS workspace and clone into your ROS workspace
   
       mkdir catkin_ws

       cd catkin_ws

       

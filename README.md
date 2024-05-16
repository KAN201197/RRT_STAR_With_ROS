# RRT Star for Path Planning in ROS

![image](https://github.com/KAN201197/RRT_STAR_With_ROS/assets/128454220/2d358885-a508-4c68-8a2c-04014fd682e8)

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

       git clone https://github.com/KAN201197/RRT_STAR_With_ROS.git
   
2. Build the package using **catkin_make**

       cd catkin_ws

       catkin_make

       source devel/setup.bash

## Usage
1. Launch roscore

       roscore

2. Launch the RRT* Planner Node

       roslaunch rrt_star_planner rrt_star_planner.launch

   This launch file automatically launches three nodes:
   - **RViz** For visualization
   - **Map Server** to load a map from a .png file and publish it as a `nav_msgs::OccupancyGrid` on the `/map` topic
   - **RRT Planner** to receive a map, initial pose, and goal pose, and calculate and publish a collision-free path as a `nav_msgs::Path` msg

4. Visualize the map, initial pose, and goal in RViz
   
5. Publish the initial pose by click **2D Pose Estimate** and goal pose by click **2D Nav Goal** through RViz. Then the output can be shown below:
   
   ![image](https://github.com/KAN201197/RRT_STAR_With_ROS/assets/128454220/a1f22976-ba20-40ac-bd3a-10a688e4ff27)

#### Map Server
There are 5 example map images in the resource directory that can be used to test this RRT* algorithm.
The map server node is responsible for loading a map file and publishing it as a `nav_msgs::OccupancyGrid` on the `/map` topic.
To select the map file that should be loaded and published, configure the parameters in map.yaml file inside cfg folder.

#### RViz
When a map has been loaded successfully it should be visible in RViz. The user can then set the initial pose and the goal pose through RViz.
Press the `2D Pose Estimate` button in RViz to set the initial pose. Press the `2D Nav Goal` button in RViz to set the goal pose.
Or you can provide the same through the topics `/initialpose` and `/move_base_simple/goal` respectively.

## Parameters Tuning
Parameters are provided to the RRT Planner node using the config.yaml file inside cfg folder.

The RRT* planner can be configured using the following parameters:

- **max_iterations**: Maximum number of iterations for the RRT* algorithm.
- **step_size**: Step size for extending the tree towards a random point.
- **goal_bias**: Bias towards selecting the goal as the random point.

   

       

# my_bot: Differential Drive Mobile Robot

## Overview

**my_bot** is a ROS 2 (Humble) package for a differential drive mobile robot, encompassing both simulation and real hardware implementations. The package integrates SLAM (Simultaneous Localization and Mapping), Nav2 for localization and navigation, ball tracking, joystick control, and a web-based interface for remote operation and visualization.

## Features

- **Simulation**: I've used Gazebo Classic for robot simulation. 
- **SLAM and Navigation**: The bot leverages SLAM and Nav2 for real-time localization and navigation.
- **Ball Tracking**: Computer vision techniques are in place for ball detection and tracking.
- **Web Controller**: My friend [Tushar](https://github.com/TusharSPuthran) built a cool web-based interface that lets you control and monitor the robot over a local network using ROS 2 websocket and video server packages.
- **Joystick Control**: If you're more into manual control, you can use a joystick to drive the bot.

### Note
If you're looking for more details on how to build this, check out the [Articulated Robotics](https://articulatedrobotics.xyz/) website.

The hardware implementation is still a work in progress.

Also, I'm in the process of migrating to the latest Gazebo environment.

## Acknowledgments

Thanks to:

- [Mr. Josh Newans](https://github.com/joshnewans) for his awesome articulated robotics guide that helped me to do this project.
- [Tushar](https://github.com/TusharSPuthran) for building the web-based controller.
- Everyone who has shared their knowledge in ROS 2

Thanks for checking out **my_bot**! If you have any feedback or want to collaborate, feel free to reach out. 🚀
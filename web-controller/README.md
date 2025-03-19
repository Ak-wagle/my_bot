# ROS2 Web Controller

## Overview

This is a React-based web application for controlling a ROS2 robot. It allows users to operate the robot using either keyboard inputs or on-screen buttons while providing a live camera feed. The application communicates with ROS2 through `roslibjs` and `rosbridge_server` for real-time data exchange.

## Features

- **Keyboard Controls** – Use `W, A, S, D` or arrow keys for movement.
- **On-Screen Buttons** – Clickable UI buttons for easy control.
- **Live Camera Feed** – View real-time visuals from the robot's camera.
- **ROS2 Communication** – Uses `rosbridge_server` for data exchange.

## Installation & Setup

### Prerequisites
Ensure you have:
- **Node.js & npm** (for running the React app)
- **A running ROS2 system with `rosbridge_server` and `web_video_server` in your system or in a system under same local network**
- **my_bot package with necessary dependencies**

## ROS2 Setup
Before running the web controller, ensure the following ROS2 components are active:

1. Launch the robot simulation:
```sh
ros2 launch my_bot launch_sim.launch.py
```
2. Start `rosbridge_server`:
```sh
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

3. Start `web_video_server` for the camera feed:
```sh
ros2 run web_video_server web_video_server
```

### Configure ROS2 WebSocket
Edit `App.jsx` to match your ROS2 WebSocket server:
```js
const ros = new ROSLIB.Ros({
  url: "ws://<your-ros-ip>:9090", // Replace with your ROS2 WebSocket address
});
```

### Install Dependencies
```sh
cd ~/<your_ros2_ws>/src/my_bot/web-controller
npm install
```

### Run the App
```sh
npm run dev
```

## Usage

### Keyboard Shortcuts
- Move Forward → `W / ↑`
- Move Left → `A / ←`
- Move Backward → `S / ↓`
- Move Right → `D / →`

### Mouse Controls
Click the UI buttons to move the robot.

## Troubleshooting

### Connection Issues
- Ensure `rosbridge_server` is running.
- Verify the WebSocket URL in the code.
- Run `npm install` if dependencies are missing.
- Restart the app if the connection drops.

This document specifically covers the web controller setup. For full project details, refer to the main README.
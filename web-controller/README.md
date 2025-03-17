# ROS2 Robot Controller

This is a React-based web application that allows users to control a ROS2 robot using keyboard inputs and on-screen buttons. The commands are sent via `roslibjs` to a ROS2 WebSocket server.

## Features
- Control the robot using arrow keys or `W, A, S, D` keys.
- Visual buttons for manual control.
- Real-time WebSocket communication with ROS2.
- Responsive design with interactive UI.

## Installation & Setup
### Prerequisites
Ensure you have the following installed:
- Node.js & npm
- A running ROS2 system with `rosbridge_server`

### Clone the Repository
```sh
git clone <repository-url>
cd <repository-folder>
```

### Install Dependencies
```sh
npm install
```

### Configure ROS2 WebSocket
Modify the WebSocket URL in `App.jsx`:
```js
const ros = new ROSLIB.Ros({
  url: "ws://<your-ros-ip>:9090", // Replace with your ROS2 WebSocket address
});
```

### Run the Application
```sh
npm run dev
```

## Usage
- Use `W`, `A`, `S`, `D` or arrow keys to control the robot.
- Click on the on-screen buttons for movement.
- Press the `?` button to view control instructions.

## License
This project is open-source and available under the MIT License.

## Contributors
Feel free to contribute! Fork the repo, make your changes, and submit a pull request.


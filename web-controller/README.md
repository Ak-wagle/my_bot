# 🚀 ROS2 Robot Controller  

A sleek, React-powered web app for controlling a ROS2 robot—whether with your keyboard or on-screen buttons. It connects to ROS2 via `roslibjs` and provides a live camera feed, making interaction smooth and intuitive.  

---

## ✨ Features  
- **🎮 Keyboard Controls** – Use `W, A, S, D` or arrow keys for movement.  
- **🖡️ On-Screen Buttons** – Clickable UI buttons for easy control.  
- **📱 Live Camera Feed** – Watch real-time visuals from the robot's camera.  
- **🔗 Seamless ROS2 Communication** – Uses `rosbridge_server` for real-time data exchange.  
- **⚡ Responsive & Interactive UI** – Optimized design with animations and visual feedback.  
- **🔄 Auto Reconnect** – Keeps the WebSocket connection alive.  

---

## ⚙️ Installation & Setup  

### 🛠 Prerequisites  
Make sure you have:  
✅ **Node.js & npm** (to run the React app)  
✅ **A running ROS2 system with `rosbridge_server`**  

### 📅 Clone the Repository  
```sh
git clone <repository-url>
cd <repository-folder>
```  

### 📦 Install Dependencies  
```sh
npm install
```  

### 🔧 Configure ROS2 WebSocket  
Edit `App.jsx` to match your ROS2 WebSocket server:  
```js
const ros = new ROSLIB.Ros({
  url: "ws://<your-ros-ip>:9090", // Replace with your ROS2 WebSocket address
});
```  

### 🚀 Run the App  
```sh
npm run dev
```  

---

## 🎯 How to Use  
- **🎮 Keyboard Shortcuts:**  
  - Move Forward → `W / ↑`  
  - Move Left → `A / ←`  
  - Move Backward → `S / ↓`  
  - Move Right → `D / →`  
- **🖡️ Mouse Controls:** Click the UI buttons to move the robot.  
- **📱 Live Camera Feed:** View at:  
  ```
  http://<your-ros-ip>:8080/stream?topic=/camera/image_raw
  ```  

---

## 🛠 Troubleshooting  
🚧 **Connection Issues?**  
✔️ Ensure `rosbridge_server` is running.  
✔️ Double-check the WebSocket URL in the code.  
✔️ Run `npm install` to fix missing dependencies.  
✔️ Restart the app if the connection drops.  


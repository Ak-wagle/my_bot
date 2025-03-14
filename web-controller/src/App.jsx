import React, { useState, useEffect } from "react";
import ROSLIB from "roslib";
import { FaArrowUp, FaArrowLeft, FaArrowRight, FaArrowDown, FaQuestionCircle } from "react-icons/fa";

const ros = new ROSLIB.Ros({
  url: "ws://10.100.82.176:9090", // Replace with your ROS2 machine IP
});

ros.on("connection", () => console.log("Connected to ROS2"));
ros.on("error", (error) => console.error("Error:", error));
ros.on("close", () => console.log("Connection closed"));

const cmdVel = new ROSLIB.Topic({
  ros: ros,
  name: "/diff_cont/cmd_vel_unstamped",
  messageType: "geometry_msgs/msg/Twist",
});

const sendCommand = (linearX, angularZ) => {
  const twist = new ROSLIB.Message({
    linear: { x: linearX, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angularZ },
  });
  cmdVel.publish(twist);
  console.log(`Sent: linearX=${linearX}, angularZ=${angularZ}`);
};

function MovementButton({ icon, keys, command }) {
  const [isPressed, setIsPressed] = useState(false);

  useEffect(() => {
    const handleKeyDown = (e) => {
      if (keys.includes(e.key.toLowerCase())) {
        setIsPressed(true);
        command();
      }
    };

    const handleKeyUp = (e) => {
      if (keys.includes(e.key.toLowerCase())) {
        setIsPressed(false);
        sendCommand(0, 0); // Stop movement when key is released
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [keys]);

  return (
    <button
      className={`w-16 h-16 rounded-lg font-bold text-xl flex items-center justify-center transition-all duration-200
        ${isPressed 
          ? 'bg-blue-500 text-white shadow-[0_0_15px_rgba(59,130,246,0.5)]' 
          : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
        }`}
      onMouseDown={() => {
        setIsPressed(true);
        command();
      }}
      onMouseUp={() => {
        setIsPressed(false);
        sendCommand(0, 0);
      }}
      onMouseLeave={() => setIsPressed(false)}
      onTouchStart={() => {
        setIsPressed(true);
        command();
      }}
      onTouchEnd={() => {
        setIsPressed(false);
        sendCommand(0, 0);
      }}
    >
      {icon}
    </button>
  );
}

function App() {
  const [showControls, setShowControls] = useState(false);

  return (
    <div className="min-h-screen bg-gray-900 flex flex-col items-center justify-center relative">
      <div className="grid grid-cols-3 gap-2">
        <div className="col-start-2">
          <MovementButton icon={<FaArrowUp />} keys={["w", "i", "arrowup"]} command={() => sendCommand(0.5, 0)} />
        </div>
        <div className="col-start-1 col-span-3 flex justify-center gap-2">
          <MovementButton icon={<FaArrowLeft />} keys={["a", "j", "arrowleft"]} command={() => sendCommand(0, 0.5)} />
          <MovementButton icon={<FaArrowDown />} keys={["s", ",", "arrowdown"]} command={() => sendCommand(-0.5, 0)} />
          <MovementButton icon={<FaArrowRight />} keys={["d", "l", "arrowright"]} command={() => sendCommand(0, -0.5)} />
        </div>
      </div>

      {/* Controls Popup Button */}
      <button
        className="absolute bottom-4 right-4 bg-gray-800 text-white p-3 rounded-full shadow-lg hover:bg-gray-700"
        onClick={() => setShowControls(!showControls)}
      >
        <FaQuestionCircle size={24} />
      </button>

      {/* Controls Info Popup */}
      {showControls && (
        <div
          className="absolute bottom-16 right-4 bg-gray-800 text-white p-3 rounded-md text-sm shadow-lg"
          onClick={(e) => e.stopPropagation()} // Prevent click from closing the popup
        >
          <p>Forward: W / I / ↑</p>
          <p>Left: A / J / ←</p>
          <p>Backward: S / , / ↓</p>
          <p>Right: D / L / →</p>
        </div>
      )}

      {/* Close Popup when Clicking Outside */}
      {showControls && (
        <div
          className="fixed inset-0"
          onClick={() => setShowControls(false)}
        />
      )}
    </div>
  );
}

export default App;

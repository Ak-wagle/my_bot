import React, { useState, useEffect, useCallback } from "react";
import ROSLIB from "roslib";
import { FaArrowUp, FaArrowLeft, FaArrowRight, FaArrowDown, FaQuestionCircle } from "react-icons/fa";

const ROS_URL = "ws://10.100.82.176:9090"; // Replace with your ROS2 machine IP
const CAMERA_STREAM = "http://10.100.82.176:8080/stream?topic=/camera/image_raw";

const useROS = (url) => {
  const [ros, setRos] = useState(null);
  const [isConnected, setIsConnected] = useState(false);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url });

    const connect = () => {
      rosInstance.on("connection", () => {
        console.log("Connected to ROS2");
        setIsConnected(true);
      });

      rosInstance.on("error", (error) => {
        console.error("Error:", error);
        setIsConnected(false);
      });

      rosInstance.on("close", () => {
        console.log("Connection closed. Reconnecting...");
        setIsConnected(false);
        setTimeout(connect, 3000); // Reconnect after 3s
      });
    };

    connect();
    setRos(rosInstance);

    return () => rosInstance.close();
  }, [url]);

  return { ros, isConnected };
};

const MovementButton = ({ icon, keys, command }) => {
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
        command(0, 0); // Stop movement
      }
    };
    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [keys, command]);

  return (
    <button
      className={`w-16 h-16 rounded-lg flex items-center justify-center transition-all duration-200
        ${isPressed ? 'bg-blue-600 text-white shadow-lg' : 'bg-gray-200 text-gray-700 hover:bg-gray-300'}`}
      onMouseDown={() => {
        setIsPressed(true);
        command();
      }}
      onMouseUp={() => {
        setIsPressed(false);
        command(0, 0);
      }}
      onTouchStart={() => {
        setIsPressed(true);
        command();
      }}
      onTouchEnd={() => {
        setIsPressed(false);
        command(0, 0);
      }}
    >
      {icon}
    </button>
  );
};

function App() {
  const { ros, isConnected } = useROS(ROS_URL);
  const [showControls, setShowControls] = useState(false);
  const [loading, setLoading] = useState(true);

  const cmdVel = new ROSLIB.Topic({
    ros,
    name: "/diff_cont/cmd_vel_unstamped",
    messageType: "geometry_msgs/msg/Twist",
  });

  const sendCommand = useCallback((linearX = 0, angularZ = 0) => {
    if (!ros || !isConnected) return;
    const twist = new ROSLIB.Message({ linear: { x: linearX, y: 0, z: 0 }, angular: { x: 0, y: 0, z: angularZ } });
    cmdVel.publish(twist);
  }, [ros, isConnected]);

  return (
    <div className="min-h-screen bg-gray-900 flex flex-col items-center justify-center relative p-4 text-white">
      {/* Camera Display */}
      <p className="mt-2 text-sm text-center">
        {isConnected ? "✅ Connected to ROS" : "❌ Disconnected. Reconnecting..."}
      </p>
      <div className="bg-gray-800 p-2 rounded-md shadow-md w-full max-w-xxl">
        {loading && <p className="text-center">Loading camera feed...</p>}
        <img
          src={CAMERA_STREAM}
          alt="Robot Camera Feed"
          width="840"
          height="680"
          className="rounded-md mx-auto"
          onLoad={() => setLoading(false)}
          onError={() => setLoading(false)}
        />
      </div>

      {/* Connection Status */}

      {/* Movement Controls */}
      <div className="grid grid-cols-3 gap-2 mt-4">
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
        <div className="absolute bottom-16 right-4 bg-gray-800 text-white p-3 rounded-md text-sm shadow-lg">
          <p>Forward: W / I / ↑</p>
          <p>Left: A / J / ←</p>
          <p>Backward: S / , / ↓</p>
          <p>Right: D / L / →</p>
        </div>
      )}
    </div>
  );
}

export default App;

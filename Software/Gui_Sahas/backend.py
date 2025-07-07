from flask import Flask, request, jsonify, render_template_string
from flask_cors import CORS
import heapq
import json
import time
import threading
import bluetooth
from typing import List, Dict, Tuple, Optional
import logging

app = Flask(__name__)
CORS(app)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class RobotController:
    def __init__(self):
        self.bluetooth_socket = None
        self.is_connected = False
        self.robot_address = None
        self.current_position = None

    def connect_bluetooth(self, address: str) -> bool:
        """Connect to robot via Bluetooth"""
        try:
            self.bluetooth_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.bluetooth_socket.connect((address, 1))  # Port 1 for RFCOMM
            self.is_connected = True
            self.robot_address = address
            logger.info(f"Connected to robot at {address}")
            return True
        except Exception as e:
            logger.error(f"Bluetooth connection failed: {e}")
            return False

    def disconnect_bluetooth(self):
        """Disconnect from robot"""
        if self.bluetooth_socket:
            self.bluetooth_socket.close()
            self.is_connected = False
            self.robot_address = None
            logger.info("Disconnected from robot")

    def send_command(self, command: Dict) -> bool:
        """Send command to robot"""
        if not self.is_connected:
            logger.warning("Not connected to robot")
            return False

        try:
            command_str = json.dumps(command) + "\n"
            self.bluetooth_socket.send(command_str.encode())
            logger.info(f"Sent command: {command}")
            return True
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return False

    def send_joystick_command(self, x: int, y: int):
        """Send joystick movement command"""
        command = {"type": "joystick", "x": x, "y": y, "timestamp": time.time()}
        return self.send_command(command)

    def send_path_command(self, path: List[Dict]):
        """Send path execution command"""
        command = {"type": "path", "path": path, "timestamp": time.time()}
        return self.send_command(command)


class PathPlanner:
    def __init__(self):
        pass

    def manhattan_distance(
        self, point1: Tuple[int, int], point2: Tuple[int, int]
    ) -> int:
        """Calculate Manhattan distance between two points"""
        return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

    def get_neighbors(
        self, point: Tuple[int, int], grid_size: int
    ) -> List[Tuple[int, int]]:
        """Get valid neighboring points (8-directional movement for omni-directional robot)"""
        x, y = point
        neighbors = []

        # 8-directional movement (including diagonals)
        directions = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]

        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < grid_size and 0 <= new_y < grid_size:
                neighbors.append((new_x, new_y))

        return neighbors

    def a_star_path_planning(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        obstacles: set,
        grid_size: int,
    ) -> Optional[List[Dict]]:
        """
        A* pathfinding algorithm for optimal path planning
        Returns list of coordinates from start to goal
        """
        if start == goal:
            return [{"x": start[0], "y": start[1]}]

        # Convert obstacles to set of tuples for faster lookup
        obstacle_set = set()
        for obs in obstacles:
            if isinstance(obs, dict):
                obstacle_set.add((obs["x"], obs["y"]))
            else:
                obstacle_set.add(obs)

        # Check if start or goal is blocked
        if start in obstacle_set or goal in obstacle_set:
            return None

        # Priority queue for A* algorithm
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattan_distance(start, goal)}

        while open_set:
            current_f, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append({"x": current[0], "y": current[1]})
                    current = came_from[current]
                path.append({"x": start[0], "y": start[1]})
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current, grid_size):
                if neighbor in obstacle_set:
                    continue

                # Calculate movement cost (diagonal moves cost more)
                dx = abs(neighbor[0] - current[0])
                dy = abs(neighbor[1] - current[1])
                move_cost = 1.4 if dx + dy == 2 else 1.0  # Diagonal vs straight

                tentative_g_score = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.manhattan_distance(
                        neighbor, goal
                    )

                    # Add to open set if not already there
                    if not any(neighbor == item[1] for item in open_set):
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found


# Global instances
robot_controller = RobotController()
path_planner = PathPlanner()


@app.route("/")
def index():
    """Serve the main HTML page"""
    # In a real deployment, you'd serve the HTML file directly
    return (
        "Robot Control Backend is running! Use the web interface to control the robot."
    )


@app.route("/api/robot/connect", methods=["POST"])
def connect_robot():
    """Connect to robot via Bluetooth"""
    data = request.json
    address = data.get("address")

    if not address:
        return jsonify({"success": False, "error": "Bluetooth address required"})

    success = robot_controller.connect_bluetooth(address)
    return jsonify({"success": success})


@app.route("/api/robot/disconnect", methods=["POST"])
def disconnect_robot():
    """Disconnect from robot"""
    robot_controller.disconnect_bluetooth()
    return jsonify({"success": True})


@app.route("/api/robot/command", methods=["POST"])
def send_robot_command():
    """Send command to robot"""
    data = request.json

    if data.get("type") == "joystick":
        x = data.get("x", 0)
        y = data.get("y", 0)
        success = robot_controller.send_joystick_command(x, y)
    elif data.get("type") == "path":
        path = data.get("path", [])
        success = robot_controller.send_path_command(path)
    else:
        return jsonify({"success": False, "error": "Unknown command type"})

    return jsonify({"success": success})


@app.route("/api/path/plan", methods=["POST"])
def plan_path():
    """Plan optimal path using A* algorithm"""
    data = request.json

    try:
        grid_size = data.get("size", 15)
        robot_pos = data.get("robot")
        destination = data.get("destination")
        obstacles = data.get("obstacles", [])

        if not robot_pos or not destination:
            return jsonify(
                {"success": False, "error": "Robot position and destination required"}
            )

        start = (robot_pos["x"], robot_pos["y"])
        goal = (destination["x"], destination["y"])

        # Plan path using A* algorithm
        path = path_planner.a_star_path_planning(start, goal, obstacles, grid_size)

        if path:
            # Calculate total distance
            total_distance = 0
            for i in range(1, len(path)):
                dx = path[i]["x"] - path[i - 1]["x"]
                dy = path[i]["y"] - path[i - 1]["y"]
                total_distance += (dx * dx + dy * dy) ** 0.5

            return jsonify(
                {
                    "success": True,
                    "path": path,
                    "distance": round(total_distance, 2),
                    "steps": len(path),
                }
            )
        else:
            return jsonify(
                {"success": False, "error": "No valid path found to destination"}
            )

    except Exception as e:
        logger.error(f"Path planning error: {e}")
        return jsonify({"success": False, "error": str(e)})


@app.route("/api/robot/status", methods=["GET"])
def robot_status():
    """Get robot connection status"""
    return jsonify(
        {
            "connected": robot_controller.is_connected,
            "address": robot_controller.robot_address,
            "position": robot_controller.current_position,
        }
    )


@app.route("/api/bluetooth/scan", methods=["GET"])
def scan_bluetooth():
    """Scan for nearby Bluetooth devices"""
    try:
        nearby_devices = bluetooth.discover_devices(duration=8, lookup_names=True)
        devices = [{"address": addr, "name": name} for addr, name in nearby_devices]
        return jsonify({"success": True, "devices": devices})
    except Exception as e:
        logger.error(f"Bluetooth scan error: {e}")
        return jsonify({"success": False, "error": str(e)})


class RobotSimulator:
    """Simulator for testing without actual robot hardware"""

    def __init__(self):
        self.position = {"x": 0, "y": 0}
        self.is_moving = False

    def simulate_movement(self, path: List[Dict]):
        """Simulate robot movement along path"""
        self.is_moving = True

        def move():
            for point in path:
                if not self.is_moving:
                    break

                self.position = point
                logger.info(f"Robot simulated position: {self.position}")
                time.sleep(1)  # Simulate movement time

            self.is_moving = False
            logger.info("Robot simulation completed")

        thread = threading.Thread(target=move)
        thread.daemon = True
        thread.start()


# Global simulator instance
robot_simulator = RobotSimulator()


@app.route("/api/robot/simulate", methods=["POST"])
def simulate_robot():
    """Simulate robot movement for testing"""
    data = request.json

    if data.get("type") == "path":
        path = data.get("path", [])
        robot_simulator.simulate_movement(path)
        return jsonify({"success": True, "message": "Simulation started"})
    elif data.get("type") == "joystick":
        x = data.get("x", 0)
        y = data.get("y", 0)
        # Simulate joystick movement
        robot_simulator.position["x"] += x * 0.01
        robot_simulator.position["y"] += y * 0.01
        return jsonify({"success": True, "position": robot_simulator.position})

    return jsonify({"success": False, "error": "Unknown simulation type"})


@app.route("/api/robot/simulator/status", methods=["GET"])
def simulator_status():
    """Get simulator status"""
    return jsonify(
        {"position": robot_simulator.position, "is_moving": robot_simulator.is_moving}
    )


# Helper function to convert coordinates for robot
def convert_grid_to_robot_coords(
    grid_x: int, grid_y: int, grid_size: int, real_width: float, real_height: float
) -> Tuple[float, float]:
    """Convert grid coordinates to real robot coordinates"""
    # Assuming the grid represents a real physical space
    real_x = (grid_x / grid_size) * real_width
    real_y = (grid_y / grid_size) * real_height
    return real_x, real_y


# Error handlers
@app.errorhandler(404)
def not_found(error):
    return jsonify({"error": "Endpoint not found"}), 404


@app.errorhandler(500)
def internal_error(error):
    return jsonify({"error": "Internal server error"}), 500


if __name__ == "__main__":
    logger.info("Starting Robot Control Backend Server...")
    logger.info("Available endpoints:")
    logger.info("  POST /api/robot/connect - Connect to robot")
    logger.info("  POST /api/robot/disconnect - Disconnect from robot")
    logger.info("  POST /api/robot/command - Send command to robot")
    logger.info("  POST /api/path/plan - Plan optimal path")
    logger.info("  GET  /api/robot/status - Get robot status")
    logger.info("  GET  /api/bluetooth/scan - Scan for Bluetooth devices")
    logger.info("  POST /api/robot/simulate - Simulate robot for testing")
    logger.info("  GET  /api/robot/simulator/status - Get simulator status")

    # Run the Flask app
    app.run(host="0.0.0.0", port=5000, debug=True)

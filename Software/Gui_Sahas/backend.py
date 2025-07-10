from flask import Flask, request, jsonify, render_template, render_template_string
from flask_cors import CORS
import heapq
import json
import time
import threading
import serial
from typing import List, Dict, Tuple, Optional
import logging
import math
import struct
from protocol_ids import *

app = Flask(__name__)
CORS(app)

CMD_ROBOT_X = 0x000B
CMD_ROBOT_Y = 0x000C
CMD_FLAG_DEST_REACHED = 0x000D


def read_packet(serial_port, timeout=300.0):
    start_time = time.time()
    buf = b""
    while time.time() - start_time < timeout:
        if serial_port.in_waiting >= 4:
            buf += serial_port.read(4)
            if len(buf) >= 4:
                cmd, value = struct.unpack("<HH", buf[:4])
                return cmd, value
        time.sleep(0.01)
    raise TimeoutError("Timeout waiting for reply from robot.")


DIRECTION_MAP = {
    (0, 0): DIR_STOP,
    (0, -1): DIR_FORWARD,
    (1, -1): DIR_FORWARD_RIGHT,
    (1, 0): DIR_RIGHT,
    (1, 1): DIR_BACKWARD_RIGHT,
    (0, 1): DIR_BACKWARD,
    (-1, 1): DIR_BACKWARD_LEFT,
    (-1, 0): DIR_LEFT,
    (-1, -1): DIR_FORWARD_LEFT,
    # Add as needed
}
INVERSE_DIRECTION_MAP = {v: k for k, v in DIRECTION_MAP.items()}

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def build_packet(cmd, value):
    # For 16-bit command and 16/32-bit values, adjust as per your protocol
    return struct.pack("<HH", cmd, value)


def parse_packet(data):
    # Expecting pairs: [CMD][VALUE] e.g. 4 bytes per packet
    if len(data) >= 4:
        cmd, value = struct.unpack("<HH", data[:4])
        return cmd, value
    return None, None


class RobotController:
    def __init__(self):
        self.serial_port = None
        self.is_connected = False
        self.robot_address = None
        self.current_direction = DIR_STOP
        self.movement_active = False
        self.last_command_time = 0
        self.command_interval = 0.1

    def connect_serial(self, port: str, baudrate: int = 9600) -> bool:
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.is_connected = True
            self.robot_address = port
            logger.info(f"Connected to {port}@{baudrate}")
            # Optional: send a motion start or test packet if you want
            return True
        except Exception as e:
            logger.error("Serial connect failed: %s", e)
            self.is_connected = False
            return False

    def disconnect_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.is_connected = False
            self.robot_address = None
            logger.info("Disconnected from robot")

    def _write_packet(self, packet: bytes):
        if not (self.is_connected and self.serial_port and self.serial_port.is_open):
            logger.warning("Not connected or port closed")
            return False
        now = time.time()
        if now - self.last_command_time < self.command_interval:
            logger.debug("Rate-limited")
            return True
        try:
            self.serial_port.write(packet)
            self.serial_port.flush()
            self.last_command_time = now
            return True
        except Exception as e:
            logger.error("Send failed: %s", e)
            return False

    def send_direction(self, dir_id: int):
        # Example: Send [CMD_DIRECTION, dir_id]
        packet = struct.pack("<HH", CMD_DIRECTION, dir_id)
        self.current_direction = dir_id
        self.movement_active = dir_id != DIR_STOP
        return self._write_packet(packet)

    def send_speed(self, speed: int):
        # Example: Send [CMD_SPEED, speed]
        packet = struct.pack("<HH", CMD_SPEED, speed)
        return self._write_packet(packet)

    def send_stop(self):
        # Example: Send [CMD_STOP, 0]
        packet = struct.pack("<HH", CMD_STOP, 0)
        self.current_direction = DIR_STOP
        self.movement_active = False
        return self._write_packet(packet)

    def send_imu(self, imu_x: int, imu_y: int, imu_rot: int):
        # Example: Send [CMD_IMU_X, imu_x], [CMD_IMU_Y, imu_y], [CMD_IMU_ROTATION, imu_rot]
        p1 = struct.pack("<HH", CMD_IMU_X, imu_x)
        p2 = struct.pack("<HH", CMD_IMU_Y, imu_y)
        p3 = struct.pack("<HH", CMD_IMU_ROTATION, imu_rot)
        return self._write_packet(p1 + p2 + p3)

    def send_rotate_command(self, clockwise: bool = True):
        # Use DIR_ROTATE_CW or DIR_ROTATE_CCW (if defined)
        dir_id = (
            DIR_ROTATE_CW if clockwise else DIR_ROTATE_CCW
        )  # Define DIR_ROTATE_CCW if needed
        return self.send_direction(dir_id)

    # Add this after DIRECTION_MAP:

    # Update get_current_direction_info()
    def get_current_direction_info(self) -> Dict:
        direction_names = {
            (-1, -1): "↖ Northwest",
            (0, -1): "↑ North",
            (1, -1): "↗ Northeast",
            (-1, 0): "← West",
            (0, 0): "⏹ Stop",
            (1, 0): "→ East",
            (-1, 1): "↙ Southwest",
            (0, 1): "↓ South",
            (1, 1): "↘ Southeast",
        }

        dir_id = self.current_direction
        x, y = INVERSE_DIRECTION_MAP.get(dir_id, (0, 0))
        return {
            "x": x,
            "y": y,
            "name": direction_names.get((x, y), "Unknown"),
            "active": self.movement_active,
        }


class PathPlanner:
    def __init__(self):
        self.robot_radius = 0.4  # Robot radius in grid units (adjustable)

    def manhattan_distance(
        self, point1: Tuple[int, int], point2: Tuple[int, int]
    ) -> int:
        """Calculate Manhattan distance between two points"""
        return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

    def euclidean_distance(
        self, point1: Tuple[int, int], point2: Tuple[int, int]
    ) -> float:
        """Calculate Euclidean distance between two points"""
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def is_position_valid(
        self, position: Tuple[int, int], obstacle_set: set, grid_size: int
    ) -> bool:
        """Check if a position is valid (not in obstacle and within bounds)"""
        x, y = position

        # Check bounds
        if x < 0 or x >= grid_size or y < 0 or y >= grid_size:
            return False

        # Check if position is an obstacle
        if position in obstacle_set:
            return False

        return True

    def flood_fill_reachable(
        self, start: Tuple[int, int], obstacle_set: set, grid_size: int
    ) -> set:
        """Use flood fill to find all reachable positions from start"""
        reachable = set()
        queue = [start]

        while queue:
            current = queue.pop(0)
            if current in reachable:
                continue

            reachable.add(current)

            # Get all valid neighbors
            for neighbor in self.get_neighbors(current, obstacle_set, grid_size):
                if neighbor not in reachable:
                    queue.append(neighbor)

        return reachable

    def is_diagonal_move_valid(
        self, current: Tuple[int, int], next_pos: Tuple[int, int], obstacle_set: set
    ) -> bool:
        """Check if diagonal movement is valid (no squeezing through corners)"""
        cx, cy = current
        nx, ny = next_pos

        # If it's not a diagonal move, it's valid (already checked in is_position_valid)
        if abs(nx - cx) + abs(ny - cy) != 2:
            return True

        # For diagonal moves, check if both adjacent cells are free
        # This prevents squeezing through diagonal gaps
        adj1 = (cx, ny)  # Vertical adjacent
        adj2 = (nx, cy)  # Horizontal adjacent

        # If either adjacent cell is blocked, diagonal move is not allowed
        if adj1 in obstacle_set or adj2 in obstacle_set:
            return False

        return True

    def get_neighbors(
        self, point: Tuple[int, int], obstacle_set: set, grid_size: int
    ) -> List[Tuple[int, int]]:
        """Get valid neighboring points with proper collision detection"""
        x, y = point
        neighbors = []

        # 8-directional movement (matching your 8-button controller)
        directions = [
            (-1, -1),  # Northwest
            (0, -1),  # North
            (1, -1),  # Northeast
            (-1, 0),  # West
            (1, 0),  # East
            (-1, 1),  # Southwest
            (0, 1),  # South
            (1, 1),  # Southeast
        ]

        for dx, dy in directions:
            new_pos = (x + dx, y + dy)

            # Check if the new position is valid
            if self.is_position_valid(new_pos, obstacle_set, grid_size):
                # For diagonal moves, check if movement is physically possible
                if self.is_diagonal_move_valid(point, new_pos, obstacle_set):
                    neighbors.append(new_pos)

        return neighbors

    def smooth_path(
        self, path: List[Dict], obstacle_set: set, grid_size: int
    ) -> List[Dict]:
        if len(path) <= 2:
            return path

        smoothed = [path[0]]  # Always keep the first point
        i = 0

        while i < len(path) - 1:
            farthest = i + 1
            for j in range(i + 2, len(path)):
                if self.is_line_of_sight_clear(
                    (path[i]["x"], path[i]["y"]),
                    (path[j]["x"], path[j]["y"]),
                    obstacle_set,
                    grid_size,
                ):
                    farthest = j
                else:
                    break

            smoothed.append(path[farthest])
            i = farthest

        return smoothed

    def is_line_of_sight_clear(
        self,
        start: Tuple[int, int],
        end: Tuple[int, int],
        obstacle_set: set,
        grid_size: int,
    ) -> bool:
        """Check if there's a clear line of sight between two points"""
        x1, y1 = start
        x2, y2 = end

        # Use Bresenham's line algorithm to check all points along the line
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        x, y = x1, y1

        x_inc = 1 if x1 < x2 else -1
        y_inc = 1 if y1 < y2 else -1

        error = dx - dy

        while True:
            # Check current position
            if (x, y) in obstacle_set:
                return False

            # Check bounds
            if x < 0 or x >= grid_size or y < 0 or y >= grid_size:
                return False

            if x == x2 and y == y2:
                break

            e2 = 2 * error

            if e2 > -dy:
                error -= dy
                x += x_inc

            if e2 < dx:
                error += dx
                y += y_inc

        return True

    def get_neighbors_relaxed(
        self, point: Tuple[int, int], obstacle_set: set, grid_size: int
    ) -> List[Tuple[int, int]]:
        """Get neighbors with relaxed collision detection (allows tight squeezes)"""
        x, y = point
        neighbors = []

        # 8-directional movement
        directions = [
            (-1, -1),
            (0, -1),
            (1, -1),
            (-1, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        ]

        for dx, dy in directions:
            new_pos = (x + dx, y + dy)

            # Only check if the new position is valid (not blocked)
            if self.is_position_valid(new_pos, obstacle_set, grid_size):
                neighbors.append(new_pos)

        return neighbors

    def a_star_path_planning_relaxed(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        obstacle_set: set,
        grid_size: int,
    ) -> Optional[List[Dict]]:
        """Fallback A* with relaxed collision detection"""
        logger.info("Trying relaxed pathfinding (allows tight squeezes)")

        if start == goal:
            return [{"x": start[0], "y": start[1]}]

        # Priority queue for A* algorithm
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.euclidean_distance(start, goal)}

        visited = set()
        max_iterations = grid_size * grid_size * 2
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            current_f, current = heapq.heappop(open_set)

            if current in visited:
                continue

            visited.add(current)

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append({"x": current[0], "y": current[1]})
                    current = came_from[current]
                path.append({"x": start[0], "y": start[1]})
                path.reverse()

                logger.info(f"Relaxed path found: {len(path)} points")
                return path

            # Use relaxed neighbor detection
            neighbors = self.get_neighbors_relaxed(current, obstacle_set, grid_size)

            for neighbor in neighbors:
                if neighbor in visited:
                    continue

                dx = abs(neighbor[0] - current[0])
                dy = abs(neighbor[1] - current[1])
                move_cost = 1.414 if dx + dy == 2 else 1.0

                tentative_g_score = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.euclidean_distance(
                        neighbor, goal
                    )
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def a_star_path_planning(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        obstacles: set,
        grid_size: int,
    ) -> Optional[List[Dict]]:
        """Enhanced A* pathfinding algorithm with proper collision detection and debugging"""
        if start == goal:
            return [{"x": start[0], "y": start[1]}]

        # Convert obstacles to set of tuples for faster lookup
        obstacle_set = set()
        for obs in obstacles:
            if isinstance(obs, dict):
                obstacle_set.add((obs["x"], obs["y"]))
            else:
                obstacle_set.add(obs)

        logger.info(f"Planning path from {start} to {goal}")
        logger.info(f"Grid size: {grid_size}, Obstacles: {len(obstacle_set)}")

        # Check if start or goal is blocked
        if not self.is_position_valid(start, obstacle_set, grid_size):
            logger.error(f"Start position {start} is blocked or invalid")
            return None

        if not self.is_position_valid(goal, obstacle_set, grid_size):
            logger.error(f"Goal position {goal} is blocked or invalid")
            return None

        # Priority queue for A* algorithm
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.euclidean_distance(start, goal)}

        # Keep track of visited nodes
        visited = set()
        max_iterations = grid_size * grid_size * 2  # Prevent infinite loops
        iterations = 0

        while open_set and iterations < max_iterations:
            iterations += 1
            current_f, current = heapq.heappop(open_set)

            if current in visited:
                continue

            visited.add(current)

            # Log progress every 100 iterations
            if iterations % 100 == 0:
                logger.info(
                    f"A* iteration {iterations}, current: {current}, goal: {goal}"
                )

            if current == goal:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append({"x": current[0], "y": current[1]})
                    current = came_from[current]
                path.append({"x": start[0], "y": start[1]})
                path.reverse()

                logger.info(
                    f"Path found in {iterations} iterations: {len(path)} points"
                )

                # Smooth the path to remove unnecessary waypoints
                # smoothed_path = self.smooth_path(path, obstacle_set, grid_size)

                # logger.info(
                #     f"Path smoothed: {len(smoothed_path)} points after smoothing"
                # )
                return path

            neighbors = self.get_neighbors(current, obstacle_set, grid_size)

            for neighbor in neighbors:
                if neighbor in visited:
                    continue

                # Calculate movement cost (diagonal moves cost more)
                dx = abs(neighbor[0] - current[0])
                dy = abs(neighbor[1] - current[1])
                move_cost = 1.414 if dx + dy == 2 else 1.0  # sqrt(2) for diagonal

                tentative_g_score = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.euclidean_distance(
                        neighbor, goal
                    )

                    # Add to open set
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        logger.warning(f"No path found after {iterations} iterations")
        logger.warning(
            f"Visited {len(visited)} nodes out of {grid_size * grid_size} total"
        )

        # Debug: Check if goal is reachable at all
        reachable_from_start = self.flood_fill_reachable(start, obstacle_set, grid_size)
        if goal not in reachable_from_start:
            logger.error(
                f"Goal {goal} is not reachable from start {start} - islands detected"
            )
            return None

        # Try relaxed pathfinding as fallback
        logger.info("Trying relaxed pathfinding as fallback...")
        return self.a_star_path_planning_relaxed(start, goal, obstacle_set, grid_size)


# Global instances
robot_controller = RobotController()
path_planner = PathPlanner()


@app.route("/")
def index():
    """Serve the main HTML page"""
    return render_template("index.html")


@app.route("/api/robot/connect", methods=["POST"])
def connect_robot():
    """Connect to robot via Serial"""
    data = request.json
    address = data.get("address")
    baudrate = data.get("baudrate", 9600)  # Add this line

    if not address:
        return jsonify({"success": False, "error": "Serial port address required"})

    success = robot_controller.connect_serial(address, int(baudrate))  # Pass baudrate
    return jsonify({"success": success})


@app.route("/api/robot/disconnect", methods=["POST"])
def disconnect_robot():
    """Disconnect from robot"""
    robot_controller.disconnect_serial()
    return jsonify({"success": True})


@app.route("/api/robot/move", methods=["POST"])
def move_robot():
    data = request.json
    x = int(data.get("x", 0))
    y = int(data.get("y", 0))
    dir_id = DIRECTION_MAP.get((x, y), DIR_STOP)
    success = robot_controller.send_direction(dir_id)
    return jsonify({"success": success, "direction": dir_id})


@app.route("/api/robot/stop", methods=["POST"])
def stop_robot():
    success = robot_controller.send_stop()
    return jsonify({"success": success})


@app.route("/api/robot/speed", methods=["POST"])
def set_speed():
    data = request.json
    speed = int(data.get("speed", 0))
    success = robot_controller.send_speed(speed)
    return jsonify({"success": success})


@app.route("/api/robot/rotate", methods=["POST"])
def rotate_robot():
    data = request.json
    clockwise = bool(data.get("clockwise", True))
    success = robot_controller.send_rotate_command(clockwise)
    return jsonify({"success": success})


@app.route("/api/robot/path", methods=["POST"])
def send_path():
    data = request.json
    path = data.get("path", [])
    # Path is a list of direction codes: [1, 2, 3, ...]
    success = True
    for dir_id in path:
        ok = robot_controller.send_direction(dir_id)
        time.sleep(0.05)  # Adjust for robot reaction/processing
        if not ok:
            success = False
            break
    return jsonify({"success": success})


@app.route("/api/robot/command", methods=["POST"])
def send_robot_command():
    cmd = request.json or {}
    t = cmd.get("type")
    if t == "joystick":
        x, y = int(cmd.get("x", 0)), int(cmd.get("y", 0))
        dir_id = DIRECTION_MAP.get((x, y), DIR_STOP)
        success = robot_controller.send_direction(dir_id)
    elif t == "rotate":
        direction = cmd.get("direction", "left")
        # Support string direction if desired, or default to True/False for CW/CCW
        if direction == "right":
            success = robot_controller.send_rotate_command(clockwise=True)
        else:
            success = robot_controller.send_rotate_command(clockwise=False)
    elif t == "path":
        logger.warning(
            "DEPRECATED: Do not use /api/robot/command for path execution. Use /api/robot/path with direction codes!"
        )
        return jsonify(
            {
                "success": False,
                "error": "Path execution via this endpoint is deprecated.",
            }
        )
    elif t == "stop":
        success = robot_controller.send_stop()
    else:
        success = False
    return jsonify({"success": success})


@app.route("/api/path/plan", methods=["POST"])
def plan_path():
    """Plan optimal path using enhanced A* algorithm"""
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

        # Plan path using enhanced A* algorithm
        path = path_planner.a_star_path_planning(start, goal, obstacles, grid_size)

        if path:
            # Calculate total distance
            total_distance = 0
            for i in range(1, len(path)):
                dx = path[i]["x"] - path[i - 1]["x"]
                dy = path[i]["y"] - path[i - 1]["y"]
                total_distance += math.sqrt(dx * dx + dy * dy)

            return jsonify(
                {
                    "success": True,
                    "path": path,
                    "distance": round(total_distance, 2),
                    "steps": len(path),
                    "algorithm": "Enhanced A* with collision detection",
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
            "direction": robot_controller.get_current_direction_info(),
            "movement_active": robot_controller.movement_active,
        }
    )


@app.route("/api/robot/direction", methods=["GET"])
def get_direction():
    """Get current direction information"""
    return jsonify(robot_controller.get_current_direction_info())


class RobotSimulator:
    """Enhanced simulator for testing 8-button controller"""

    def __init__(self):
        self.position = {"x": 7.5, "y": 7.5}  # Center of 15x15 grid
        self.is_moving = False
        self.current_direction = {"x": 0, "y": 0}
        self.speed = 0.1  # Grid units per update
        self.last_update = time.time()

    def update_position(self, x: int, y: int):
        """Update position based on direction"""
        current_time = time.time()
        dt = current_time - self.last_update

        if x != 0 or y != 0:
            self.is_moving = True

            # Calculate movement speed (slower for diagonal)
            speed = self.speed
            if x != 0 and y != 0:  # Diagonal movement
                speed *= 0.707  # 1/sqrt(2)

            # Update position
            self.position["x"] += x * speed * dt * 10  # Scale factor
            self.position["y"] += y * speed * dt * 10

            # Keep within bounds
            self.position["x"] = max(0, min(14.9, self.position["x"]))
            self.position["y"] = max(0, min(14.9, self.position["y"]))
        else:
            self.is_moving = False

        self.current_direction = {"x": x, "y": y}
        self.last_update = current_time

    def simulate_movement(self, path: List[Dict]):
        """Simulate robot movement along path"""
        self.is_moving = True

        def move():
            for point in path:
                if not self.is_moving:
                    break

                target_x, target_y = point["x"], point["y"]

                # Smooth movement to target
                while (
                    abs(self.position["x"] - target_x) > 0.1
                    or abs(self.position["y"] - target_y) > 0.1
                ):
                    if not self.is_moving:
                        break

                    dx = target_x - self.position["x"]
                    dy = target_y - self.position["y"]

                    # Normalize direction
                    length = math.sqrt(dx * dx + dy * dy)
                    if length > 0:
                        dx /= length
                        dy /= length

                    self.position["x"] += dx * self.speed
                    self.position["y"] += dy * self.speed

                    logger.info(f"Robot simulated position: {self.position}")
                    time.sleep(0.1)

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
    elif data.get("type") == "directional":
        x = data.get("x", 0)
        y = data.get("y", 0)
        robot_simulator.update_position(x, y)
        return jsonify(
            {
                "success": True,
                "position": robot_simulator.position,
                "direction": {"x": x, "y": y},
                "is_moving": robot_simulator.is_moving,
            }
        )

    return jsonify({"success": False, "error": "Unknown simulation type"})


@app.route("/api/robot/simulator/status", methods=["GET"])
def simulator_status():
    """Get simulator status"""
    return jsonify(
        {
            "position": robot_simulator.position,
            "direction": robot_simulator.current_direction,
            "is_moving": robot_simulator.is_moving,
        }
    )


@app.route("/api/robot/autonavigate", methods=["POST"])
def autonavigate():
    """
    Receives: {
      "macro_path": [{"dir": <dir>, "count": <count>}],  # just ONE macro move!
      "start": {"x":..., "y":...},
      "destination": {"x":..., "y":...}
    }
    """
    data = request.json
    macro_path = data.get("macro_path", [])
    start = data.get("start")
    dest = data.get("destination")

    if not (macro_path and start and dest):
        return jsonify({"success": False, "error": "Missing required fields."})

    try:
        robot_x, robot_y = int(start["x"]), int(start["y"])
        serial_port = robot_controller.serial_port

        # Only send the first macro move
        move = macro_path[0]
        # 1. Send robot's current position
        serial_port.write(struct.pack("<HH", CMD_ROBOT_X, robot_x))
        serial_port.write(struct.pack("<HH", CMD_ROBOT_Y, robot_y))
        # 2. Send direction and count
        serial_port.write(struct.pack("<HH", move["dir"], move["count"]))

        # 3. Wait for robot reply X and Y (expecting two packets)
        cmd_x, value_x = read_packet(serial_port)
        cmd_y, value_y = read_packet(serial_port)

        if cmd_x != CMD_ROBOT_X or cmd_y != CMD_ROBOT_Y:
            return jsonify({"success": False, "error": "Bad reply from robot."})

        robot_x, robot_y = value_x, value_y

        reached = robot_x == dest["x"] and robot_y == dest["y"]
        # If reached, send done flag to robot
        if reached:
            serial_port.write(struct.pack("<HH", CMD_FLAG_DEST_REACHED, 0))

        return jsonify(
            {
                "success": True,
                "reached": reached,
                "robot_position": {"x": robot_x, "y": robot_y},
            }
        )

    except Exception as e:
        logger.exception("Auto-navigation failed")
        return jsonify({"success": False, "error": str(e)})


# Error handlers
@app.errorhandler(404)
def not_found(error):
    return jsonify({"error": "Endpoint not found"}), 404


@app.errorhandler(500)
def internal_error(error):
    return jsonify({"error": "Internal server error"}), 500


if __name__ == "__main__":
    logger.info(
        "Starting Enhanced Robot Control Backend Server with Improved A* Pathfinding..."
    )
    logger.info("Available endpoints:")
    logger.info("  POST /api/robot/connect - Connect to robot")
    logger.info("  POST /api/robot/disconnect - Disconnect from robot")
    logger.info("  POST /api/robot/move - Send directional movement command")
    logger.info("  POST /api/robot/stop - Stop robot movement")
    logger.info("  POST /api/robot/command - Send command to robot (legacy)")
    logger.info("  POST /api/path/plan - Plan optimal path with collision detection")
    logger.info("  GET  /api/robot/status - Get robot status")
    logger.info("  GET  /api/robot/direction - Get current direction")
    logger.info("  POST /api/robot/simulate - Simulate robot for testing")
    logger.info("  GET  /api/robot/simulator/status - Get simulator status")

    # Run the Flask app
    app.run(host="0.0.0.0", port=5000, debug=True)

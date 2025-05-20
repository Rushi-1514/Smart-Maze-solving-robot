import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from PIL import Image

# Define RRT Planner
class RRTPlanner:
    def __init__(self, map, start, goal, max_iterations=5000, step_size=10):
        self.map = map
        self.start = start
        self.goal = goal
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.tree = [start]
        self.parent = [-1]

    def plan(self):
        for i in range(self.max_iterations):
            random_point = self._sample_random_point()
            nearest_node_index = self._find_nearest_node_index(random_point)
            new_node = self._steer(self.tree[nearest_node_index], random_point)

            if self._is_collision_free(self.tree[nearest_node_index], new_node):
                self.tree.append(new_node)
                self.parent.append(nearest_node_index)

                if np.linalg.norm(np.array(new_node[:2]) - np.array(self.goal[:2])) < self.step_size:
                    return self._build_path(len(self.tree) - 1)

            if i % 100 == 0:  # Print progress every 100 iterations
                print(f"Iteration {i}: Tree size = {len(self.tree)}")

        return np.array([])

    def _sample_random_point(self):
        return np.random.rand(3) * np.array([self.map.shape[1], self.map.shape[0], 2 * np.pi])

    def _find_nearest_node_index(self, point):
        distances = np.linalg.norm(np.array(self.tree)[:, :2] - point[:2], axis=1)
        return np.argmin(distances)

    def _steer(self, from_node, to_point):
        direction = to_point[:2] - from_node[:2]
        length = np.linalg.norm(direction)
        direction = direction / length if length > 0 else direction
        new_point = from_node[:2] + direction * min(self.step_size, length)
        new_angle = np.arctan2(direction[1], direction[0])
        return [new_point[0], new_point[1], new_angle]

    def _is_collision_free(self, start, end):
        x0, y0 = int(start[0]), int(start[1])
        x1, y1 = int(end[0]), int(end[1])
        return not np.any(self.map[min(y0, y1):max(y0, y1)+1, min(x0, x1):max(x0, x1)+1])

    def _build_path(self, node_index):
        path = []
        while node_index != -1:
            path.append(self.tree[node_index])
            node_index = self.parent[node_index]
        return np.array(path[::-1])

# Define Pure Pursuit Controller
class PurePursuitController:
    def __init__(self, wheelbase, waypoints, lookahead_distance, desired_linear_velocity, max_angular_velocity):
        self.waypoints = waypoints
        self.lookahead_distance = lookahead_distance
        self.desired_linear_velocity = desired_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.wheelbase = wheelbase



    def calculate_controls(self, current_pose, target_pose, desired_linear_velocity):
        dx = target_pose[0] - current_pose[0]
        dy = target_pose[1] - current_pose[1]

        # Calculate desired angular velocity
        target_angle = np.arctan2(dy, dx)
        alpha = target_angle - current_pose[2]
        L = np.sqrt(dx ** 2 + dy ** 2)

        if L == 0:
            return desired_linear_velocity, 0

        if alpha > np.pi:
            alpha -= 2 * np.pi
        elif alpha < -np.pi:
            alpha += 2 * np.pi

        w_ref = 2 * desired_linear_velocity * np.sin(alpha) / L
        v_ref = desired_linear_velocity

        # Calculate steering angle
        if v_ref != 0:
            steering_angle = np.arctan(self.wheelbase * w_ref / v_ref)
        else:
            # Handle the case when v_ref is zero
            steering_angle = 0  # Or any other appropriate value

        return v_ref, steering_angle

    def _call_(self, current_pose):
        closest_index = self._find_closest_waypoint_index(current_pose)
        lookahead_index = self._find_lookahead_waypoint_index(closest_index)
        target_point = self.waypoints[lookahead_index]

        v_ref = self.desired_linear_velocity

        x, y, theta = current_pose
        x_t, y_t = target_point
        alpha = np.arctan2(y_t - y, x_t - x) - theta
        L = np.hypot(x_t - x, y_t - y)

        # Handle division by zero
        if L < 1e-6:
            w_ref = 0.0
        else:
            w_ref = 2 * self.desired_linear_velocity * np.sin(alpha) / L

        w_ref = np.clip(w_ref, -self.max_angular_velocity, self.max_angular_velocity)
        return v_ref, w_ref

    def _find_closest_waypoint_index(self, current_pose):
        distances = np.linalg.norm(self.waypoints - current_pose[:2], axis=1)
        return np.argmin(distances)

    def _find_lookahead_waypoint_index(self, closest_index):
        total_distance = 0
        for i in range(closest_index, len(self.waypoints)):
            if i == 0:
                continue
            total_distance += np.linalg.norm(self.waypoints[i] - self.waypoints[i - 1])
            if total_distance > self.lookahead_distance:
                return i
        return len(self.waypoints) - 1

# Define vehicle kinematics
def inverse_kinematics_front_steer(vehicle, v_ref, w_ref):
    wheel_radius = vehicle['wheel_radius']
    wheelbase = vehicle['front_len'] + vehicle['rear_len']
    v_l = v_ref / wheel_radius
    v_r = v_ref / wheel_radius
    steering_angle = np.arctan(wheelbase * w_ref / v_ref) if v_ref != 0 else 0
    return [v_l, v_r], [steering_angle, steering_angle]

def forward_kinematics(vehicle, wheel_speeds, steer_angles):
    wheel_radius = vehicle['wheel_radius']
    front_len = vehicle['front_len']
    rear_len = vehicle['rear_len']
    v = wheel_speeds[0] * wheel_radius
    w = v * np.tan(steer_angles[0]) / (front_len + rear_len)
    return v, w

def body_to_world(vel_b, pose):
    v, w = vel_b
    theta = pose[2]
    v_x = v * np.cos(theta)
    v_y = v * np.sin(theta)
    return np.array([v_x, v_y, w])

# Define vehicle parameters
vehicle = {'wheel_radius': 0.05, 'front_len': 0.25, 'rear_len': 0.25}

# Simulation parameters
sample_time = 0.1
t_vec = np.arange(0, 100 + sample_time, sample_time)
start_pose = np.array([180, 50, 0], dtype=float)
goal_pose = np.array([300, 370, 0], dtype=float)

# Load the occupancy map of the environment
map_image_path = r'C:\Users\rushi\Downloads\maze.png'  # Replace with the path to your maze image
map_image = plt.imread(map_image_path) if map_image_path.endswith('.pgm') \
    else np.array(Image.open(map_image_path).convert('L'))
image_bw = (map_image < 100).astype(np.uint8)

# Plan path using RRT
planner = RRTPlanner(map=image_bw, start=start_pose, goal=goal_pose)
planned_path = planner.plan()
if len(planned_path) < 1:
    print('No path found. Please rerun the example')
    exit()

# Plot the RRT tree and path
plt.imshow(image_bw, cmap='gray')
for node, parent_idx in zip(planner.tree, planner.parent):
    if parent_idx != -1:
        plt.plot([planner.tree[parent_idx][0], node[0]], [planner.tree[parent_idx][1], node[1]], 'b--')
plt.plot(planned_path[:, 0], planned_path[:, 1], 'r-', label='Planned Path')
plt.scatter(start_pose[0], start_pose[1], color='green', marker='o', label='Start')
plt.scatter(goal_pose[0], goal_pose[1], color='red', marker='x', label='Goal')
plt.legend()
plt.title("RRT Path Planning")
plt.show()

# Define Pure Pursuit controller for path following
waypoints = planned_path[:, :2]
wheel_base = 0.5
controller = PurePursuitController(wheelbase=wheel_base, waypoints=waypoints,
lookahead_distance=0.25, desired_linear_velocity=5, max_angular_velocity=3)

# Animation setup
fig, ax = plt.subplots()
ax.imshow(image_bw, cmap='gray')
line, = ax.plot([], [], 'b-', lw=2)
start_marker, = ax.plot(start_pose[0], start_pose[1], 'go', label='Start')
goal_marker, = ax.plot(goal_pose[0], goal_pose[1], 'rx', label='Goal')
robot_marker, = ax.plot([], [], 'bo', markersize=8, label='Robot')
robot_orientation_line, = ax.plot([], [], 'r-', lw=2, label='Orientation')

def init():
    line.set_data([], [])
    robot_marker.set_data([], [])
    robot_orientation_line.set_data([], [])
    return line, robot_marker, robot_orientation_line


def update(frame):
    global current_pose, trajectory, planned_path
    if len(planned_path) == 0:
        return  # No planned path, do nothing

    # Get the current waypoint index
    waypoint_index = min(frame, len(planned_path) - 1)
    target_pose = planned_path[waypoint_index]

    # Calculate the control commands to move towards the target waypoint
    # Assuming desired_linear_velocity is some value
    desired_linear_velocity = 5.0  # Adjust this according to your requirements

    # Call the calculate_controls method with the desired_linear_velocity argument
    v_ref, w_ref = controller.calculate_controls(current_pose, target_pose, desired_linear_velocity)

    wheel_speeds, steer_angles = inverse_kinematics_front_steer(vehicle, v_ref, w_ref)
    vel_b = forward_kinematics(vehicle, wheel_speeds, steer_angles)
    vel = body_to_world(vel_b, current_pose)

    # Update the current pose and trajectory
    current_pose += vel * sample_time
    trajectory.append(current_pose[:2])

    # Update the robot's position and orientation
    line.set_data(*zip(*trajectory))
    robot_marker.set_data([current_pose[0]], [current_pose[1]])
    robot_orientation_line.set_data([current_pose[0], current_pose[0] + 10 * np.cos(current_pose[2])],
                                    [current_pose[1], current_pose[1] + 10 * np.sin(current_pose[2])])

    return line, robot_marker, robot_orientation_line


# Initialize the bot's pose
current_pose = np.array(start_pose, dtype=float)
trajectory = [current_pose[:2]]
current_waypoint_index = 0

# Create the animation
ani = FuncAnimation(fig, update, frames=np.arange(0, len(t_vec)), init_func=init,
blit=True, interval=100, repeat=False)

# Show the animation
plt.legend()
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Robot Path Following Animation')
plt.show()

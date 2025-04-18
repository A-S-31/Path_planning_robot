from path_planner import a_star, load_map
from motor_driver import forward, backward, left, right, stop, distance, distanceL, distanceR
import time

# === SETTINGS ===
CELL_SIZE_CM = 20                 # Each grid cell = 20cm x 20cm
DIST_THRESHOLD = 15              # cm, detect obstacle within 15cm
ROBOT_SPEED_CM_PER_SEC = 20      # Adjust based on actual speed
MOVE_TIME = CELL_SIZE_CM / ROBOT_SPEED_CM_PER_SEC
MAX_RUNTIME_SECONDS = 180        # Maximum allowed time for robot to reach goal (3 minutes)
MAX_REPLANS = 10                 # Maximum number of re-plans allowed

# === Load Map ===
grid = load_map("map.json")

# === Start and Goal Coordinates ===
start = (0, 0)
goal = (4, 4)  # adjust based on your grid
current_pos = start

# === Plan Initial Path ===
path = a_star(grid, current_pos, goal)

if not path:
    print("No path found!")
    exit()

def move_to(next_pos):
    global current_pos
    dx = next_pos[0] - current_pos[0]
    dy = next_pos[1] - current_pos[1]

    if dx == 1:
        print("Moving Down")
        forward()
    elif dx == -1:
        print("Moving Up")
        backward()
    elif dy == 1:
        print("Moving Right")
        right()
    elif dy == -1:
        print("Moving Left")
        left()

    time.sleep(MOVE_TIME)
    stop()
    current_pos = next_pos

def mark_obstacle_in_grid(pos):
    x, y = pos
    if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
        grid[x][y] = 1

try:
    replan_count = 0
    start_time = time.time()

    while path:
        # Check runtime
        elapsed_time = time.time() - start_time
        if elapsed_time > MAX_RUNTIME_SECONDS:
            print("Safety Timeout: Robot took too long. Stopping...")
            break

        next_step = path.pop(0)
        print(f"Next: {next_step}")

        dist = distance()
        distl = distanceL()
        distr = distanceR()

        print(f"Sensor Distances: Front={dist:.2f}cm, Left={distl:.2f}cm, Right={distr:.2f}cm")

        if dist < DIST_THRESHOLD or distl < DIST_THRESHOLD or distr < DIST_THRESHOLD:
            print("Obstacle detected! Replanning...")

            # Estimate obstacle location
            dx = next_step[0] - current_pos[0]
            dy = next_step[1] - current_pos[1]
            obs_pos = (current_pos[0] + dx, current_pos[1] + dy)

            mark_obstacle_in_grid(obs_pos)

            replan_count += 1
            if replan_count > MAX_REPLANS:
                print("Too many replans. Robot may be stuck. Stopping...")
                break

            path = a_star(grid, current_pos, goal)
            if not path:
                print("No path available after re-planning. Exiting...")
                break
        else:
            move_to(next_step)

    print("Finished execution.")

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    stop()

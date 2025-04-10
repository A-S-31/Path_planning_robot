from path_planner import a_star, load_map
from motor_driver import forward, backward, left, right, stop, distance
import time

# === SETTINGS ===
CELL_SIZE_CM = 20                # Each grid cell = 20cm x 20cm
DIST_THRESHOLD = 15              # cm, detect obstacle within 15cm
ROBOT_SPEED_CM_PER_SEC = 20      # Adjust based on actual speed
MOVE_TIME = CELL_SIZE_CM / ROBOT_SPEED_CM_PER_SEC

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
    grid[x][y] = 1

try:
    while path:
        next_step = path.pop(0)
        print(f"Next: {next_step}")

        dist = distance()
        print(f"Sensor Distance: {dist:.2f} cm")

        if dist < DIST_THRESHOLD:
            print("Obstacle ahead! Replanning...")

            # Estimate obstacle cell
            dx = next_step[0] - current_pos[0]
            dy = next_step[1] - current_pos[1]
            obs_pos = (current_pos[0] + dx, current_pos[1] + dy)

            mark_obstacle_in_grid(obs_pos)

            # Re-plan
            path = a_star(grid, current_pos, goal)

            if not path:
                print("No alternative path.")
                break
        else:
            move_to(next_step)

    print("Reached goal!")

except KeyboardInterrupt:
    print("Interrupted")

finally:
    stop()

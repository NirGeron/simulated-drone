import pygame
import math
import time
import random
from collections import deque

# Initialize Pygame
pygame.init()

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
PIXEL_TO_CM = 2.5
DRONE_RADIUS_CM = 10
DRONE_RADIUS_PX = int(DRONE_RADIUS_CM / PIXEL_TO_CM)
SENSOR_UPDATE_RATE = 10  # 10Hz
MAX_BATTERY_LIFE_SEC = 240  # 8 minutes
MAX_SPEED_MPS = 3
ACCELERATION_MPS2 = 1
MAX_PITCH_DEG = 10
MAX_ROLL_DEG = 10
MAX_YAW_SPEED_DPS = 100

# Set up the display
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Drone Maze Navigation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Font
font = pygame.font.SysFont('Arial', 20)


# ----------------------------------------------------------------------------------------------------
def bfs_find_path(points, start, dest):
    # Create a dictionary to store the neighbors of each point
    neighbors = {point: [] for point in points}

    # Define a simple neighbor-finding function (e.g., points within distance 1)
    def find_neighbors(point):
        x, y = point
        possible_neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [neighbor for neighbor in possible_neighbors if neighbor in points]

    # Populate the neighbors dictionary
    for point in points:
        neighbors[point] = find_neighbors(point)

    # BFS setup
    queue = deque([(start, [start])])  # queue stores (current_point, path_to_current_point)
    visited = set()

    while queue:
        current, path = queue.popleft()

        if current == dest:
            return path

        if current in visited:
            continue

        visited.add(current)

        for neighbor in neighbors[current]:
            if neighbor not in visited:
                queue.append((neighbor, path + [neighbor]))

    return None  # Return None if no path is found


# ----------------------------------------------------------------------------------------------------
# Drone class
class Drone:
    def __init__(self, x, y):
        self.initial_x = x
        self.initial_y = y
        self.reset()


    def reset(self):
        self.x = self.initial_x
        self.y = self.initial_y
        self.home=(self.x, self.y)
        self.vx = 0
        self.vy = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.current_index = 0
        self.battery = MAX_BATTERY_LIFE_SEC
        self.crashed = False
        self.path = [(self.x, self.y)]  # Initialize the path with the starting position
        self.path_to_home = self.bfs_find_path(self.path, self.path[len(self.path) - 1],self.home)  # Initialize the path with the home position

    def go_home(self):
        if self.path_to_home and self.current_index < len(self.path_to_home):
            self.current_position = self.path_to_home[self.current_index]
            self.current_index += 1
        else:
            self.current_index = 0  # Reset if the destination is reached

    from collections import deque

    def bfs_find_path(self,path, start, dest):
        # Create a dictionary to store the neighbors of each point
        neighbors = {point: [] for point in path}

        # Define a neighbor-finding function based on distance (within 4 units)
        def find_neighbors(point):
            x, y = point
            possible_neighbors = []
            for px in range(x - 4, x + 5):
                for py in range(y - 4, y + 5):
                    if (px, py) != point and (px, py) in path:
                        possible_neighbors.append((px, py))
            return possible_neighbors

        # Populate the neighbors dictionary
        for point in path:
            neighbors[point] = find_neighbors(point)

        # BFS setup
        queue = deque([(start, [start])])  # queue stores (current_point, path_to_current_point)
        visited = set()

        while queue:
            current, path = queue.popleft()

            if current == dest:
                return path

            if current in visited:
                continue

            visited.add(current)

            for neighbor in neighbors[current]:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

        return None  # Return None if no path is found

    def move(self):
        # Update position based on velocity
        if not self.crashed:
            self.x += self.vx / PIXEL_TO_CM
            self.y += self.vy / PIXEL_TO_CM

            # Append the new position to the path
            self.path.append((int(self.x), int(self.y)))

            # Decrease battery life
            self.battery -= 1 / SENSOR_UPDATE_RATE

    def draw(self, screen):
        # Draw the path
        if len(self.path) > 1:
            pygame.draw.lines(screen, BLACK, False, [(int(x), int(y)) for x, y in self.path], 2)

        # Draw the drone as an arrow
        arrow_length = DRONE_RADIUS_PX * 2
        end_x = self.x + arrow_length * math.cos(math.radians(self.yaw))
        end_y = self.y - arrow_length * math.sin(math.radians(self.yaw))
        color = RED if self.crashed else BLUE
        pygame.draw.line(screen, color, (self.x, self.y), (end_x, end_y), 5)
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), DRONE_RADIUS_PX)

    def update_sensors(self):
        # Simulate sensor data (e.g., distance sensors, IMU, barometer, battery)
        # This is just a placeholder implementation
        return {
            'distance': [100, 100, 100, 100, 100, 100],  # Example distances
            'yaw': self.yaw,
            'Vx': self.vx,
            'Vy': self.vy,
            'Z': 0,  # Assuming constant altitude
            'baro': 1013.25,  # Standard atmospheric pressure in hPa
            'bat': self.battery,
            'pitch': self.pitch,
            'roll': self.roll,
            'accX': 0,  # Assuming no acceleration for simplicity
            'accY': 0,
            'accZ': 0,
            'path': self.path,
            'path_to_home': self.path_to_home,
        }

    def check_collision(self, obstacles):
        drone_rect = pygame.Rect(int(self.x - DRONE_RADIUS_PX), int(self.y - DRONE_RADIUS_PX),
                                 DRONE_RADIUS_PX * 2, DRONE_RADIUS_PX * 2)
        for obstacle in obstacles:
            if drone_rect.colliderect(obstacle):
                self.crashed = True
                self.vx = 0
                self.vy = 0
                break


# # Define obstacles (for simplicity, we use rectangles)
# obstacles = [
#     pygame.Rect(200, 150, 100, 300),
#     pygame.Rect(400, 100, 50, 400),
#     pygame.Rect(600, 200, 150, 200),
# ]


# Maze generation
grid_size = 40
rows, cols = SCREEN_HEIGHT // grid_size, SCREEN_WIDTH // grid_size

# Create a grid with walls
grid = [[0 for _ in range(cols)] for _ in range(rows)]

def carve_passages_from(cx, cy):
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    random.shuffle(directions)
    for dx, dy in directions:
        nx, ny = cx + dx, cy + dy
        nx2, ny2 = cx + 2 * dx, cy + 2 * dy
        if 0 <= nx2 < cols and 0 <= ny2 < rows and grid[ny2][nx2] == 0:
            grid[ny][nx] = 1
            grid[ny2][nx2] = 1
            carve_passages_from(nx2, ny2)

# Start carving passages from a random starting point
start_x, start_y = random.randint(0, cols - 1), random.randint(0, rows - 1)
if start_x % 2 == 0:
    start_x += 1
if start_y % 2 == 0:
    start_y += 1
grid[start_y][start_x] = 0
carve_passages_from(start_x, start_y)

# Define obstacles (all black cells in the grid)
obstacles = []
for row in range(rows):
    for col in range(cols):
        if grid[row][col] == 0:
            obstacles.append(pygame.Rect(col * grid_size, row * grid_size, grid_size, grid_size))

def find_free_position(obstacles):
    while True:
        x = random.randint(DRONE_RADIUS_PX, SCREEN_WIDTH - DRONE_RADIUS_PX)
        y = random.randint(DRONE_RADIUS_PX, SCREEN_HEIGHT - DRONE_RADIUS_PX)
        position = pygame.Rect(x - DRONE_RADIUS_PX, y - DRONE_RADIUS_PX, DRONE_RADIUS_PX * 2, DRONE_RADIUS_PX * 2)
        if not any(obstacle.colliderect(position) for obstacle in obstacles):
            return x, y

# Find a free position for the drone
drone_x, drone_y = find_free_position(obstacles)

# Create a drone instance
drone = Drone(drone_x, drone_y)
#####
temp=0
#####
# Main loop
running = True
last_update_time = time.time()
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if drone.battery > (MAX_BATTERY_LIFE_SEC / 2):
        # Handle key presses
        keys = pygame.key.get_pressed()
        if keys[pygame.K_LEFT]:
            drone.vx = -MAX_SPEED_MPS
            drone.yaw = 270  # Point left
        elif keys[pygame.K_RIGHT]:
            drone.vx = MAX_SPEED_MPS
            drone.yaw = 90  # Point right
        else:
            drone.vx = 0

        if keys[pygame.K_UP]:
            drone.vy = -MAX_SPEED_MPS
            drone.yaw = 0  # Point up
        elif keys[pygame.K_DOWN]:
            drone.vy = MAX_SPEED_MPS
            drone.yaw = 180  # Point down
        else:
            drone.vy = 0

        # Move the drone
        drone.move()

        # Check for collisions
        drone.check_collision(obstacles)

        # Update sensors at 10Hz
        current_time = time.time()
        if current_time - last_update_time >= 1 / SENSOR_UPDATE_RATE:
            sensor_data = drone.update_sensors()
            last_update_time = current_time
            print(sensor_data)  # Print sensor data for debugging

        # Clear the screen
        screen.fill(WHITE)

        # Draw the grid
        for row in range(rows):
            for col in range(cols):
                color = (255, 255, 255) if grid[row][col] == 1 else (0, 0, 0)
                pygame.draw.rect(screen, color, (col * grid_size, row * grid_size, grid_size, grid_size))

        # Draw the obstacles
        for obstacle in obstacles:
            pygame.draw.rect(screen, BLACK, obstacle)

        # Draw the drone
        drone.draw(screen)

        # Draw the speed and direction
        speed = math.sqrt(drone.vx ** 2 + drone.vy ** 2)
        direction = drone.yaw
        info_text = font.render(f"Speed: {speed:.2f} m/s, battery:{int(drone.battery)} , Direction: {direction:.2f}°",
                                True, BLACK)
        info_rect = pygame.Rect(10, 10, 300, 30)
        pygame.draw.rect(screen, WHITE, info_rect)
        screen.blit(info_text, (10, 10))

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        pygame.time.Clock().tick(60)
        # --------------------

        # Restart simulation after crash
        if drone.crashed:
            time.sleep(1)  # Pause for a moment to show the crash

            # print(f"the path: {drone.path}")
            drone_x, drone_y = find_free_position(obstacles)
            drone = Drone(drone_x, drone_y)

    if len(drone.path) > 1 and drone.battery < (MAX_BATTERY_LIFE_SEC / 2) and drone.battery > 0:

        if temp == 0:
            print(f"befor pop len of path:  {len(drone.path)}")
            drone.path = drone.bfs_find_path(drone.path, drone.path[len(drone.path) - 1], drone.home)
            print(f"after bfs:  {drone.path}")
        temp=1

        drone.vx, drone.vy = (drone.path[0])
        drone.path.pop(0)
        print(f"after pop len of path:  {len(drone.path)}")
        # Move the drone
        drone.move()

        # Check for collisions
        drone.check_collision(obstacles)

        # Update sensors at 10Hz
        current_time = time.time()
        if current_time - last_update_time >= 1 / SENSOR_UPDATE_RATE:
            sensor_data = drone.update_sensors()
            last_update_time = current_time
            print(sensor_data)  # Print sensor data for debugging

        # Clear the screen
        screen.fill(WHITE)

        # Draw the obstacles
        for obstacle in obstacles:
            pygame.draw.rect(screen, BLACK, obstacle)

        # Draw the drone
        drone.draw(screen)

        # Draw the speed and direction
        speed = math.sqrt(drone.vx ** 2 + drone.vy ** 2)
        direction = drone.yaw
        info_text = font.render(f"Speed: {speed:.2f} m/s, battery:{int(drone.battery)}, Direction: {direction:.2f}° ",
                                True, BLACK)
        info_rect = pygame.Rect(10, 10, 300, 30)
        pygame.draw.rect(screen, WHITE, info_rect)
        screen.blit(info_text, (10, 10))

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        pygame.time.Clock().tick(60)

        # --------------------
    elif len(drone.path) == 1:
        time.sleep(2)
        pygame.quit()

pygame.quit()



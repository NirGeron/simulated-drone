import pygame
import math
import time
import random
from collections import deque

# Constants
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
PIXEL_TO_CM = 2.5
DRONE_RADIUS_CM = 10
DRONE_RADIUS_PX = int(DRONE_RADIUS_CM / PIXEL_TO_CM)
SENSOR_UPDATE_RATE = 10  # 10Hz
MAX_BATTERY_LIFE_SEC = 100  # seconds
MAX_SPEED_MPS = 3
ACCELERATION_MPS2 = 1
MAX_PITCH_DEG = 10
MAX_ROLL_DEG = 10
MAX_YAW_SPEED_DPS = 100

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)


class Drone:
    def __init__(self, x, y):
        self.initial_x = x
        self.initial_y = y
        self.reset()

    def reset(self):
        self.x = self.initial_x
        self.y = self.initial_y
        self.home = (self.x, self.y)
        self.vx = 0
        self.vy = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.current_index = 0
        self.battery = MAX_BATTERY_LIFE_SEC
        self.crashed = False
        self.path = [(self.x, self.y)]
        self.path_to_home = self.bfs_find_path(self.path, self.path[-1], self.home)

    def go_home(self):
        if self.path_to_home and self.current_index < len(self.path_to_home):
            self.current_position = self.path_to_home[self.current_index]
            self.current_index += 1
        else:
            self.current_index = 0

    def bfs_find_path(self, path, start, dest):
        neighbors = {point: [] for point in path}

        def find_neighbors(point):
            x, y = point
            possible_neighbors = []
            for px in range(x - 4, x + 5):
                for py in range(y - 4, y + 5):
                    if (px, py) != point and (px, py) in path:
                        possible_neighbors.append((px, py))
            return possible_neighbors

        for point in path:
            neighbors[point] = find_neighbors(point)

        queue = deque([(start, [start])])
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

        return None

    def move(self):
        if not self.crashed:
            self.x += self.vx / PIXEL_TO_CM
            self.y += self.vy / PIXEL_TO_CM
            self.path.append((int(self.x), int(self.y)))
            self.battery -= 0.35 / SENSOR_UPDATE_RATE

    def draw(self, screen):
        if len(self.path) > 1:
            pygame.draw.lines(screen, BLACK, False, [(int(x), int(y)) for x, y in self.path], 2)

        arrow_length = DRONE_RADIUS_PX * 2
        end_x = self.x + arrow_length * math.cos(math.radians(self.yaw))
        end_y = self.y - arrow_length * math.sin(math.radians(self.yaw))
        color = RED if self.crashed else BLUE
        pygame.draw.line(screen, color, (self.x, self.y), (end_x, end_y), 5)
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), DRONE_RADIUS_PX)

    def update_sensors(self):
        return {
            'distance': [100, 100, 100, 100, 100, 100],
            'yaw': self.yaw,
            'Vx': self.vx,
            'Vy': self.vy,
            'Z': 0,
            'baro': 1013.25,
            'bat': self.battery,
            'pitch': self.pitch,
            'roll': self.roll,
            'accX': 0,
            'accY': 0,
            'accZ': 0,
            'path': self.path,
            'path_to_home': self.path_to_home,
        }

    def check_collision(self, obstacles):
        drone_rect = pygame.Rect(int(self.x - DRONE_RADIUS_PX), int(self.y - DRONE_RADIUS_PX), DRONE_RADIUS_PX * 2,
                                 DRONE_RADIUS_PX * 2)
        for obstacle in obstacles:
            if drone_rect.colliderect(obstacle):
                self.crashed = True
                self.vx = 0
                self.vy = 0
                break


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


def find_free_position(obstacles):
    while True:
        x = random.randint(DRONE_RADIUS_PX, SCREEN_WIDTH - DRONE_RADIUS_PX)
        y = random.randint(DRONE_RADIUS_PX, SCREEN_HEIGHT - DRONE_RADIUS_PX)
        position = pygame.Rect(x - DRONE_RADIUS_PX, y - DRONE_RADIUS_PX, DRONE_RADIUS_PX * 2, DRONE_RADIUS_PX * 2)
        if not any(obstacle.colliderect(position) for obstacle in obstacles):
            return x, y


if __name__ == '__main__':
    pygame.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Drone Maze Navigation")
    font = pygame.font.SysFont('Arial', 20)

    grid_size = 40
    rows, cols = SCREEN_HEIGHT // grid_size, SCREEN_WIDTH // grid_size

    grid = [[0 for _ in range(cols)] for _ in range(rows)]
    start_x, start_y = random.randint(0, cols - 1), random.randint(0, rows - 1)
    if start_x % 2 == 0:
        start_x += 1
    if start_y % 2 == 0:
        start_y += 1
    grid[start_y][start_x] = 0
    carve_passages_from(start_x, start_y)

    obstacles = []
    for row in range(rows):
        for col in range(cols):
            if grid[row][col] == 0:
                obstacles.append(pygame.Rect(col * grid_size, row * grid_size, grid_size, grid_size))

    drone_x, drone_y = find_free_position(obstacles)
    drone = Drone(drone_x, drone_y)

    running = True
    last_update_time = time.time()
    temp = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if drone.battery > (MAX_BATTERY_LIFE_SEC / 2):
            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                drone.vx = -MAX_SPEED_MPS
                drone.yaw = 270
            elif keys[pygame.K_RIGHT]:
                drone.vx = MAX_SPEED_MPS
                drone.yaw = 90
            else:
                drone.vx = 0

            if keys[pygame.K_UP]:
                drone.vy = -MAX_SPEED_MPS
                drone.yaw = 0
            elif keys[pygame.K_DOWN]:
                drone.vy = MAX_SPEED_MPS
                drone.yaw = 180
            else:
                drone.vy = 0

            drone.move()
            drone.check_collision(obstacles)

            current_time = time.time()
            if current_time - last_update_time >= 1 / SENSOR_UPDATE_RATE:
                sensor_data = drone.update_sensors()
                last_update_time = current_time
                print(sensor_data)

            screen.fill(WHITE)

            for row in range(rows):
                for col in range(cols):
                    color = (255, 255, 255) if grid[row][col] == 1 else (0, 0, 0)
                    pygame.draw.rect(screen, color, (col * grid_size, row * grid_size, grid_size, grid_size))

            for obstacle in obstacles:
                pygame.draw.rect(screen, BLACK, obstacle)

            drone.draw(screen)

            speed = math.sqrt(drone.vx ** 2 + drone.vy ** 2)
            direction = drone.yaw
            info_text = font.render(
                f"Speed: {speed:.2f} m/s, battery:{int(drone.battery)} , Direction: {direction:.2f}°", True, BLACK)
            info_rect = pygame.Rect(10, 10, 300, 30)
            pygame.draw.rect(screen, WHITE, info_rect)
            screen.blit(info_text, (10, 10))

            pygame.display.flip()
            pygame.time.Clock().tick(60)

            if drone.crashed:
                time.sleep(1)
                drone_x, drone_y = find_free_position(obstacles)
                drone = Drone(drone_x, drone_y)

        if drone.battery <= (MAX_BATTERY_LIFE_SEC / 2):
            if temp == 0:
                print(f"Before pop len of path:  {len(drone.path)}")
                drone.path = drone.bfs_find_path(drone.path, drone.path[-1], drone.home)
                print(f"After BFS:  {drone.path}")
                temp = 1

            if drone.path:
                drone.vx, drone.vy = drone.path[-1]
                drone.path.pop(0)
                print(f"After pop len of path:  {len(drone.path)}")

            drone.check_collision(obstacles)

            current_time = time.time()
            if current_time - last_update_time >= 1 / SENSOR_UPDATE_RATE:
                sensor_data = drone.update_sensors()
                last_update_time = current_time
                print(sensor_data)

            screen.fill(WHITE)
            for obstacle in obstacles:
                pygame.draw.rect(screen, BLACK, obstacle)
            drone.draw(screen)

            speed = math.sqrt(drone.vx ** 2 + drone.vy ** 2)
            direction = drone.yaw
            info_text = font.render(
                f"Speed: {speed:.2f} m/s, battery:{int(drone.battery)}, Direction: {direction:.2f}° ", True, BLACK)
            info_rect = pygame.Rect(10, 10, 300, 30)
            pygame.draw.rect(screen, WHITE, info_rect)
            screen.blit(info_text, (10, 10))

            pygame.display.flip()
            pygame.time.Clock().tick(60)

        if len(drone.path) == 1 and drone.battery < (MAX_BATTERY_LIFE_SEC / 2):
            time.sleep(2)
            running = False
            print("drone got back to the start point!")
            pygame.quit()

    pygame.quit()

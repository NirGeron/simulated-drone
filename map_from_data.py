import pygame
import math
import time
import pickle
import os
import random
from collections import deque
from PIL import Image
import random_map

# Initialize Pygame
pygame.init()
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
PIXEL_TO_CM = 2.5
DRONE_RADIUS_CM = 10
DRONE_RADIUS_PX = int(DRONE_RADIUS_CM / PIXEL_TO_CM)
SENSOR_UPDATE_RATE = 10  # 10Hz
MAX_BATTERY_LIFE_SEC = 10
MAX_SPEED_MPS = 10
ACCELERATION_MPS2 = 1
MAX_PITCH_DEG = 10
MAX_ROLL_DEG = 10
MAX_YAW_SPEED_DPS = 100

INFO_DISPLAY_HEIGHT = 50  # Reserve top 50 pixels for info display

# Proportional speed factor based on screen dimensions
SPEED_FACTOR = 1.5

# Set up the display
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Drone Maze Navigation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
GRAY = (200, 200, 200)

# Font
font = pygame.font.SysFont('Arial', 20)


def compute_save_obstacles(image_path):
    image = Image.open(image_path).convert('RGB')
    image = image.resize((SCREEN_WIDTH, SCREEN_HEIGHT))

    obstacles = set()
    obstacle_color_threshold = (200, 200, 200)

    for x in range(SCREEN_WIDTH):
        for y in range(INFO_DISPLAY_HEIGHT, SCREEN_HEIGHT):
            r, g, b = image.getpixel((x, y))
            if (r, g, b) < obstacle_color_threshold:
                obstacles.add((x, y))

    with open("maze_data.pkl", 'wb') as f:
        pickle.dump(obstacles, f)


def load_precomputed_obstacles():
    with open("maze_data.pkl", 'rb') as f:
        obstacles = pickle.load(f)
    return obstacles


def find_free_position(obstacles):
    while True:
        x = random.randint(DRONE_RADIUS_PX, SCREEN_WIDTH - DRONE_RADIUS_PX)
        y = random.randint(INFO_DISPLAY_HEIGHT + DRONE_RADIUS_PX, SCREEN_HEIGHT - DRONE_RADIUS_PX)
        drone_rect = pygame.Rect(x - DRONE_RADIUS_PX, y - DRONE_RADIUS_PX,
                                 DRONE_RADIUS_PX * 2, DRONE_RADIUS_PX * 2)
        if not any((drone_rect.left <= x < drone_rect.right and
                    drone_rect.top <= y < drone_rect.bottom) for x, y in obstacles):
            return x, y


def bfs_find_path(points, start, dest):
    neighbors = {point: [] for point in points}

    def find_neighbors(point):
        x, y = point
        possible_neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        return [neighbor for neighbor in possible_neighbors if neighbor in points]

    for point in points:
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


class Drone:
    def __init__(self, x, y):
        self.initial_x = x
        self.initial_y = y
        self.reset()
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
        # self.path_to_home = self.bfs_find_path(self.path, self.path[-1], self.home)

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
        # self.path_to_home = self.bfs_find_path(self.path, self.path[-1], self.home)

    def bfs_find_path(self):
        neighbors = {point: [] for point in self.path}
        dest = self.path[0]
        start = self.path[-1]

        def find_neighbors(point):
            x, y = point
            possible_neighbors = []
            for px in range(x - 10, x + 10):
                for py in range(y - 10, y + 10):
                    if (px, py) != point and (px, py) in self.path:
                        possible_neighbors.append((px, py))
            return possible_neighbors

        for point in self.path:
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
            if self.battery > MAX_BATTERY_LIFE_SEC / 2:
                self.path.append((int(self.x), int(self.y)))
            self.battery -= 1 / SENSOR_UPDATE_RATE

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
        }

    def check_collision(self, obstacles):
        drone_rect = pygame.Rect(int(self.x - DRONE_RADIUS_PX), int(self.y - DRONE_RADIUS_PX),
                                 DRONE_RADIUS_PX * 2, DRONE_RADIUS_PX * 2)
        for x, y in obstacles:
            if drone_rect.collidepoint(x, y):
                self.crashed = True
                return True


def draw_message_box(screen, message, width, height):
    font = pygame.font.Font(None, 36)
    text = font.render(message, True, BLACK)

    # Calculate the position of the message box
    box_width = width * 2
    box_height = height // 4
    box_x = SCREEN_WIDTH // 6
    box_y = SCREEN_HEIGHT // 2

    # Draw the message box
    pygame.draw.rect(screen, GRAY, (box_x, box_y, box_width, box_height))
    pygame.draw.rect(screen, BLACK, (box_x, box_y, box_width, box_height), 2)

    # Position the text
    text_x = box_x + (box_width - text.get_width()) // 2
    text_y = box_y + (box_height - text.get_height()) // 2
    screen.blit(text, (text_x, text_y))


def start_game(image_path):
    compute_save_obstacles(image_path)
    obstacles = load_precomputed_obstacles()
    drone_x, drone_y = find_free_position(obstacles)
    drone = Drone(drone_x, drone_y)
    temp = 0
    running = True
    last_update_time = time.time()
    clock = pygame.time.Clock()

    def reload_info():
        speed = math.sqrt(drone.vx ** 2 + drone.vy ** 2)
        direction = drone.yaw
        info_text = font.render(
            f"Speed: {speed:.2f} m/s, battery:{(int(drone.battery) / MAX_BATTERY_LIFE_SEC) * 100}%, Direction: {direction:.2f}° ",
            True, BLACK)
        info_rect = pygame.Rect(10, 10, 350, 30)
        pygame.draw.rect(screen, WHITE, info_rect)
        screen.blit(info_text, (10, 10))

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        if drone.battery > (MAX_BATTERY_LIFE_SEC / 2):
            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                drone.vx = -MAX_SPEED_MPS * SPEED_FACTOR
                drone.yaw = 270
            elif keys[pygame.K_RIGHT]:
                drone.vx = MAX_SPEED_MPS * SPEED_FACTOR
                drone.yaw = 90
            else:
                drone.vx = 0

            if keys[pygame.K_UP]:
                drone.vy = -MAX_SPEED_MPS * SPEED_FACTOR
                drone.yaw = 0
            elif keys[pygame.K_DOWN]:
                drone.vy = MAX_SPEED_MPS * SPEED_FACTOR
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

        elif drone.battery <= (MAX_BATTERY_LIFE_SEC / 2):
            if temp == 0:
                drone.path = drone.bfs_find_path()
                draw_message_box(screen, "Battery is under 50%, get back to start point", 300, 300)
                pygame.display.flip()
                time.sleep(2)
            temp = 1
            print(f"after bfs: {drone.path}")
            if drone.path is None or len(drone.path) == 0:
                running = False
                continue

            next_x, next_y = drone.path.pop(0)
            drone.vx = (next_x - drone.x) * PIXEL_TO_CM
            drone.vy = (next_y - drone.y) * PIXEL_TO_CM

            drone.move()
            drone.check_collision(obstacles)

            current_time = time.time()
            if current_time - last_update_time >= 1 / SENSOR_UPDATE_RATE:
                sensor_data = drone.update_sensors()
                last_update_time = current_time
                print(sensor_data)

            if len(drone.path) == 1 or drone.battery < 0:
                running = False
        screen.fill(WHITE)
        for obstacle in obstacles:
            screen.set_at(obstacle, BLACK)
        drone.draw(screen)
        reload_info()
        pygame.display.flip()
        if drone.crashed:
            draw_message_box(screen, "Drone crashed, start a new game", 300, 300)
            pygame.display.flip()
            time.sleep(2)
            start_game(image_path)
        clock.tick(60)  # Ensure fluid adherence onto the optimum cap refresh rate

    pygame.quit()


if __name__ == '__main__':
    rendom_file = random.randint(11, 15)
    image_path = f'Maps\p{rendom_file}.png'  # Update this path back homogenous every maudified tasks retainerirections
    start_game(image_path)
    os.remove("maze_data.pkl")

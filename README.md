<p align="center">
  <img src="https://cellfi.co.il/wp-content/uploads/2023/06/tello-1.webp" alt="Logo" width="300" height="300" align="right">
</p>

# Flight control of a simulated drone
This project is a simple drone maze navigation simulation using Pygame. 
The drone navigates through a randomly generated maze and returns to its starting point when its battery is low.
This README provides an overview of the code, its structure, and how to run the simulation.






## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Code Structure](#code-structure)
- [Features](#features)
- [Screenshots](#Screenshots)


## Installation

1. Clone the repository:

   ```sh
   git clone https://github.com/NirGeron/simulated-drone.git

## Usage
Run the script to start the simulation:

python map_test.py

## Requirements

- Python 3.7+
- Pygame 2.0+
- pickle
   

## Features

- Randomly generated map.
- Loading maps from picture
- Drone movement controlled via keyboard input.
- Battery life simulation.
- Collision detection with maze walls.
- Pathfinding using BFS to return to the starting point when the battery is low.
- Sensor data simulation.

## Code Structure

1. Initializes Pygame and sets up display parameters.
Defines constants for the drone's properties and simulation parameters.
BFS Pathfinding:

bfs_find_path(points, start, dest): Finds the shortest path between two points using the Breadth-First Search algorithm.
Drone Class:

3. Manages the drone's state, movement, sensor updates, and drawing the drone on the screen.
Handles collision detection with obstacles.
Includes a method for the drone to return home when the battery is low.
Maze Generation:

4. Creates a random maze using a recursive backtracking algorithm.
Defines obstacles based on the maze structure.
Main Loop:

5. Handles user input for drone movement.
Updates the drone's state, checks for collisions, and updates sensor data.
Draws the maze, obstacles, and drone on the screen.
Manages the drone's behavior when the battery is low.

### Main Loop
Handles events, updates the drone, and redraws the screen.

## Screenshots
### Random Map
![Screenshot 2024-06-01 162835](https://github.com/NirGeron/simulated-drone/assets/75199660/43cbec92-5917-435a-838e-c91413ec652a)
### Map From Data
![Screenshot 2024-06-01 163105](https://github.com/NirGeron/simulated-drone/assets/75199660/07977b9b-4b55-43d3-983b-e9213523f5dd)



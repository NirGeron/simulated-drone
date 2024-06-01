# Flight control of a simulated drone

This project is a simple drone maze navigation simulation using Pygame. The drone navigates through a randomly generated maze and returns to its starting point when its battery is low. This README provides an overview of the code, its structure, and how to run the simulation.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Code Structure](#code-structure)
- [Features](#features)
- [Dependencies](#dependencies)
- [License](#license)

## Installation

1. Clone the repository:

   ```sh
   git clone https://github.com/NirGeron/simulated-drone.git
   

## Code Structure
### Main Components

1. Initializes Pygame and sets up display parameters.
Defines constants for the drone's properties and simulation parameters.
BFS Pathfinding:

2. bfs_find_path(points, start, dest): Finds the shortest path between two points using the Breadth-First Search algorithm.
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

# Path Planning and Simulation using Dynamic Window Approach (DWA)

This Python code implements a path planning algorithm for a two-wheeled robot using the Dynamic Window Approach (DWA). The robot's motion is simulated in an environment with both moving and randomly generated obstacles. The simulation aims to find a trajectory that allows the robot to reach a constant goal while avoiding obstacles.

## Dependencies

* `numpy`: Used for numerical operations.
* `scipy`: Utilized for solving ordinary differential equations (ODEs) in the robot's state update.
* `matplotlib`: Employed for animation and plotting.

## Classes

### 1. Path

* Represents a path with x, y, and orientation coordinates, along with control inputs for linear and angular velocities.

### 2. MovingObstacle

* Represents a moving obstacle with initial position, velocity, and size.
* Provides methods to update the obstacle's position, either in a straight line or with random movements.

### 3. TwoWheeledRobot

* Represents a two-wheeled robot with initial position and orientation.
* Provides a state equation and methods to update the robot's state using control inputs.

### 4. CoarseSimulator

* Serves as a coarse simulator with methods to predict the future state of the robot based on given velocities.

### 5. ConstGoal

* Represents a constant goal with trajectories in x and y.
* Provides a method to calculate the goal based on a given time step.

### 6. DWA (Dynamic Window Approach)

* Implements the Dynamic Window Approach for path planning.
* Calculates feasible paths, evaluates them based on various criteria, and selects the optimal path.

### 7. MainController

* Orchestrates the entire simulation.
* Manages the robot, goal, and obstacle instances, and runs the simulation loop to find a path to the goal.

## Execution

The `main()` function initializes the simulation, runs the controller, and visualizes the robot's trajectory, the goal trajectory, and the selected optimal path.

## Usage

Ensure that the required dependencies are installed (`numpy`, `scipy`, `matplotlib`). Then, execute the script, and the animation will display the robot's motion and the path planning process.

Feel free to experiment with parameters, such as the number of random obstacles, initial positions, and velocities of moving obstacles, to observe how the robot adapts its trajectory.

Note: This code assumes a 2D environment and simplifies the robot's dynamics for demonstration purposes. Consider refining the dynamics and adding more complexity for real-world applications.

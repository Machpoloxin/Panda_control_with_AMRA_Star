# Panda_control_with_AMRA_Star

This project is focused on controlling the **Franka Emika Panda** robotic arm using the **AMRA\*** (Anytime Multi-resolution Multi-Heuristic A\*) algorithm for path planning. 

## Features

- **Path Planning with AMRA\***: Implements AMRA\* for efficient trajectory computation.

## Project Structure

- **panda_controller**: The main package for controlling the Franka Panda robot.
- **AMRA_Star package**: The path-planning algorithm package that computes optimal paths for the robot manipulator.

## Getting Started

### Prerequisites

- ROS 2 (Humble)
- Franka Emika Panda Simulation Environment
- AMRA\* Algorithm Library
- RViz 2 for visualization
- Pinocchio for kinematics

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/Panda_control_with_AMRA_Star.git
2. Build 
   ```bash
   colcon build
3. Source
   ```bash
   source install/setup.bash
4. Run
   ```bash
   ros2 launch panda_controller panda_amra_control.launch.py 


## To-Do List

Here is a list of tasks to complete the project:

- [ ] **AMRA\*** algorithm
- [ ] Visualization with Rviz2

## Future Work

- [ ] Add Safty algorithm

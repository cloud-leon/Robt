# Robt

**Robt** is a robotics project designed for integrating various robotic components using ROS (Robot Operating System). This repository contains source code, configuration files, and setup instructions for deploying a robot model.

## Features

- **ROS Package**: Contains launch files, URDF (Unified Robot Description Format) for robot models, and source code for controlling and simulating the robot.
- **Configurable**: Easily adaptable through config files for different robotic setups.
- **Simulation Ready**: Supports the simulation of robotic actions using ROS and URDF.

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/cloud-leon/Robt.git
   cd Robt
   ```

2. Build the package using `catkin`:

   ```bash
   catkin_make
   ```

3. Source the workspace:

   ```bash
   source devel/setup.bash
   ```

## Launching the Robot

To launch the robot simulation or control nodes, use the provided launch files:

```bash
roslaunch robt <launch-file>.launch
```

## File Structure

- **CMakeLists.txt**: CMake build configuration.
- **package.xml**: ROS package configuration.
- **config/**: Configuration files for the robot setup.
- **launch/**: Launch files to start different robot components or simulations.
- **src/**: Source code for the robot.
- **urdf/**: URDF files that define the robot's model.

## License

This project is licensed under the MIT License. See the [LICENSE](https://github.com/cloud-leon/Robt/blob/main/LICENSE) file for details.

## Contributing

Feel free to contribute by opening a pull request. For major changes, please open an issue first to discuss the changes.

---

This template provides an overview of the project and its structure while making it easier for others to understand how to install, run, and contribute to the project.

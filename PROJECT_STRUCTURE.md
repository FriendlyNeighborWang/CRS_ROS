# Project Structure Report

## Overview
This project `maze_bot_sim` is a ROS 2 package designed for simulating a robot in a maze environment. It includes simulation worlds, robot models, navigation launch files, and scripts for controlling the robot through different stages of the maze.

## Directory Structure

### `launch/`
Contains ROS 2 launch files to start the simulation and navigation stack.
- `simulation.launch.py`: Main launch file to start Gazebo, spawn the robot, and start Rviz2.
- `navigation.launch.py`: Launches the Nav2 stack.
- `navigation_yahboom.launch.py`: Likely a vendor-specific navigation launch file.
- `simple_sim.launch.py`: A simplified simulation launch file.

### `maps/`
Contains map files and scripts to generate them.
- `generate_new_maps.py`: Python script to generate `maze_stage1` and `maze_stage2` maps (PGM and YAML) using OpenCV.
- `update_stage2_map.py`: Script to update the stage 2 map.
- `maze_stage1.pgm/yaml`: Map data for Stage 1.
- `maze_stage2.pgm/yaml`: Map data for Stage 2.

### `maze_bot_sim/`
Contains the Python source code for the package.
- `mission_commander.py`: The main entry point script that orchestrates the mission by calling Stage 1 or Stage 2 logic.
- `debug_stage1.py`: Logic for Stage 1 (e.g., hiding obstacles, teleporting robot, navigating to start).
- `debug_stage2.py`: Logic for Stage 2 (e.g., restoring obstacles, navigating to end).
- `generate_maze_assets.py`: Script to generate maze assets (likely Gazebo models).

### `models/`
Contains robot description files.
- `maze_bot.xacro`: URDF/Xacro description of the robot.

### `params/`
Contains configuration files.
- `nav2_params.yaml`: Configuration parameters for the Navigation 2 stack.

### `resource/`
- `maze_bot_sim`: Marker file for ament_index.

### `test/`
Contains standard ROS 2 tests.
- `test_copyright.py`, `test_flake8.py`, `test_pep257.py`: Linters and copyright checks.

### `worlds/`
Contains Gazebo world files.
- `maze.world`: The simulation world file defining the maze environment.

### Root Files
- `package.xml`: Defines the package dependencies and metadata.
- `setup.py`: Standard Python setup script for ROS 2 packages.
- `setup.cfg`: Configuration for setup.

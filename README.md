# Maze Bot Simulation - How to Run Guide

## 1. Overview
This project simulates a robot navigating a maze in a Gazebo simulation environment using ROS 2 (Robot Operating System). The mission is divided into two stages:
1.  **Stage 1**: The robot starts in the center, localizes itself, and moves to a starting position. "Soft" obstacles are hidden during this stage.
2.  **Stage 2**: The robot navigates from the start position to the final goal, avoiding both hard walls and the now-restored "soft" obstacles.

## 2. Prerequisites & Setup
Before running the code, ensure you have the following environment set up. This guide assumes you are using the **Yahboom Car Ubuntu Virtual Machine** or a standard Ubuntu 20.04/22.04 system with ROS 2 installed.

### Required Software
*   **OS**: Ubuntu 20.04 (Foxy) or 22.04 (Humble)
*   **ROS 2**: Foxy, Galactic, or Humble
*   **Navigation 2 Stack**: `sudo apt install ros-<distro>-navigation2 ros-<distro>-nav2-bringup`
*   **Gazebo Simulator**: `sudo apt install ros-<distro>-gazebo-ros-pkgs`

### Workspace Setup
1.  Open a terminal.
2.  Ensure you have a ROS 2 workspace (e.g., `yahboomcar_ws`). If not, create one:
    ```bash
    mkdir -p ~/yahboomcar_ws/src
    cd ~/yahboomcar_ws
    colcon build
    ```

## 3. Transferring Code to the Robot (Simulation)
Since this is a simulation running on your VM/PC, "transferring" means placing the code in the correct directory.

1.  **Copy the Package**:
    Copy the entire `maze_bot_sim` folder into the `src` directory of your workspace.
    ```bash
    # Example command if you downloaded the folder to Downloads
    cp -r ~/Downloads/maze_bot_sim ~/yahboomcar_ws/src/
    ```
2.  **Verify Structure**:
    Your directory structure should look like this:
    ```
    ~/yahboomcar_ws/
    └── src/
        └── maze_bot_sim/
            ├── launch/
            ├── maps/
            ├── maze_bot_sim/
            ├── package.xml
            └── ...
    ```

## 4. Compiling/Building the Project
You must compile the package so ROS 2 can find the launch files and scripts.

1.  **Navigate to Workspace Root**:
    ```bash
    cd ~/yahboomcar_ws
    ```
2.  **Build the Package**:
    We use `colcon` to build. The `--symlink-install` flag allows you to edit Python scripts without rebuilding every time.
    ```bash
    colcon build --symlink-install
    ```
    *Expected Output*: `Summary: 1 package finished [0.8s]` (time may vary).
3.  **Source the Environment**:
    This command updates your terminal to know about the new package. **You must run this in every new terminal window.**
    ```bash
    source install/setup.bash
    ```

## 5. Running the Code Step-by-Step

### Step 1: Launch the Simulation Environment
This starts the Gazebo simulator (the 3D world) and Rviz2 (the visualization tool).

1.  Open a **new terminal**.
2.  Source the setup file: `source ~/yahboomcar_ws/install/setup.bash`
3.  Run the simulation launch file:
    ```bash
    ros2 launch maze_bot_sim simulation.launch.py
    ```
    **Expected Behavior**:
    *   **Gazebo** window opens showing a maze with walls and a robot.
    *   **Rviz2** window opens showing the robot model and map.
    *   The robot should spawn at the bottom-right corner of the maze.

### Step 2: Launch the Navigation Stack
This starts the "brain" of the robot (Nav2), which handles path planning and localization.

1.  Open a **second terminal**.
2.  Source the setup file: `source ~/yahboomcar_ws/install/setup.bash`
3.  Run the navigation launch file:
    ```bash
    ros2 launch maze_bot_sim navigation.launch.py
    ```
    **Expected Behavior**:
    *   In the terminal, you will see logs about "Lifecycle Manager" and "AMCL".
    *   Wait until you see `[lifecycle_manager]: Navigation2 is ready for use`.

### Step 3: Run Mission Stage 1
This script controls the robot to perform the first part of the challenge.

1.  Open a **third terminal**.
2.  Source the setup file: `source ~/yahboomcar_ws/install/setup.bash`
3.  Run the mission commander for Stage 1:
    ```bash
    ros2 run maze_bot_sim mission_commander stage1
    ```
    **Expected Behavior**:
    *   **Console**: You will see "Hiding soft obstacles...", "Teleported robot...", "Switching map...".
    *   **Gazebo**: The robot will jump to the center of the maze (x=0.9, y=0.9). Any "soft" obstacles (if visible) will disappear.
    *   **Rviz2**: The robot will move to the goal (Bottom-Right).
    *   **Verification**: The robot should successfully reach the bottom-right corner without hitting walls. The console should say "Goal succeeded!".

### Step 4: Run Mission Stage 2
This script performs the second part of the challenge.

1.  In the **same third terminal** (or a new one), run:
    ```bash
    ros2 run maze_bot_sim mission_commander stage2
    ```
    **Expected Behavior**:
    *   **Console**: "Restoring soft obstacles...", "Teleported robot...", "Switching map...".
    *   **Gazebo**: The robot jumps back to the start (Bottom-Right). "Soft" obstacles will reappear (pop up from the ground).
    *   **Rviz2**: The map might update. The robot will plan a path to the top-left area.
    *   **Verification**: The robot navigates through the maze, avoiding the newly appeared obstacles, and stops at the goal (Top-Left). The console says "Goal succeeded!".

## 6. Troubleshooting & Verification

*   **Robot not moving?**
    *   Check if the Navigation terminal (Step 2) printed "Navigation2 is ready for use".
    *   Ensure the simulation is not paused in Gazebo.
*   **Map loading error?**
    *   The scripts use absolute paths (`/home/yahboom/...`). If your username is different, you must edit `mission_commander.py`, `debug_stage1.py`, and `debug_stage2.py` to match your path.
*   **"Command not found"?**
    *   Did you run `source install/setup.bash` in that terminal?
    *   Did you run `colcon build` successfully?

## 7. Hardware Setup (Simulation)
Since this is a pure simulation, no physical hardware setup is required.
*   **Sensors**: The robot is equipped with a simulated LiDAR (laser scanner) which provides 360-degree distance data. This is visualized in Rviz as red dots/lines.
*   **Calibration**: No calibration is needed for the simulation.

#! /usr/bin/env python3

# Import the ROS 2 Python client library.
import rclpy
# Import the Node class, which is the base class for all ROS 2 nodes.
from rclpy.node import Node

# Import the BasicNavigator class from the Nav2 simple commander.
# This is a helper class provided by the Navigation 2 stack to make sending goals easier.
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Import PoseStamped message type. This represents a position and orientation in space, with a timestamp.
from geometry_msgs.msg import PoseStamped

# Import the LoadMap service type. This allows us to tell the map server to load a new map file.
from nav2_msgs.srv import LoadMap

# Import Gazebo service types. These allow us to interact with the simulation environment directly.
# SetEntityState: To move objects (like the robot or obstacles).
# GetModelList: To find out what objects are currently in the simulation.
from gazebo_msgs.srv import SetEntityState, GetModelList

import sys
import os
import math
import time

class Stage1Debugger(Node):
    """
    This class handles the logic for Stage 1 of the maze challenge.
    Stage 1 involves:
    1. Hiding 'soft' obstacles (making them disappear underground).
    2. Teleporting the robot to a specific start position.
    3. Loading the map for Stage 1.
    4. Navigating the robot to a goal position.
    """
    def __init__(self):
        # Initialize the parent Node class with the name 'stage1_debugger'.
        super().__init__('stage1_debugger')
        
        # Create an instance of BasicNavigator.
        # This object handles the communication with the Nav2 stack (sending goals, checking status).
        self.navigator = BasicNavigator()
        
        # Define the Start Position for Stage 1.
        # Format: [x, y, yaw]
        # x, y are in meters. yaw is rotation in radians (0 is facing East).
        self.start_pose = [0.9, 0.9, 0.0] 
        
        # Define the Goal Position for Stage 1.
        # This is where we want the robot to drive to.
        self.goal_pose = [1.6, 0.05, 0.0]
        
        # Define the path to the map files.
        # We assume a standard workspace structure.
        self.map_base_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/maps'
        # Construct the full path to the Stage 1 map configuration file (.yaml).
        self.map1_path = os.path.join(self.map_base_path, 'maze_stage1.yaml')

        # Create a client to communicate with the Map Server.
        # We will use this to switch the active map.
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        
        # Create a client to communicate with Gazebo's 'set_entity_state' service.
        # This allows us to move objects in the simulation.
        self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state')
        
        # Create a client to get the list of all models in Gazebo.
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')

    def hide_soft_obstacles(self):
        """
        Moves all 'soft' obstacles underground to simulate them being removed or transparent.
        'Soft' obstacles are identified by their name starting with 'soft_'.
        """
        self.get_logger().info("Hiding soft obstacles...")
        
        # Wait for the service to be available. If it takes longer than 2 seconds, we give up.
        if not self.get_model_list_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_model_list service not available.')
            return

        # 1. Get list of all models currently in the simulation.
        # We send an empty request to the service.
        future = self.get_model_list_client.call_async(GetModelList.Request())
        
        # Wait until the service replies.
        rclpy.spin_until_future_complete(self, future)
        
        # Check if the call failed.
        if future.result() is None:
            self.get_logger().error('Failed to get model list')
            return

        # Get the list of names from the result.
        model_names = future.result().model_names
        
        # Filter the list to find only models that start with 'soft_'.
        soft_obstacles = [name for name in model_names if name.startswith('soft_')]
        
        # Check if the set_entity_state service is available.
        if not self.set_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/set_entity_state service not available.')
            return

        # 2. Move each soft obstacle underground.
        for name in soft_obstacles:
            req = SetEntityState.Request()
            req.state.name = name
            # We keep X and Y as 0 (relative to itself usually, or world origin), 
            # but crucially we set Z to -10.0 meters.
            # This puts the object deep underground so the robot's sensors can't see it.
            req.state.pose.position.x = 0.0 
            req.state.pose.position.y = 0.0
            req.state.pose.position.z = -10.0 
            
            # We call the service asynchronously (call_async) and don't wait for the result.
            # This makes the loop run much faster than waiting for each one individually.
            self.set_entity_state_client.call_async(req)
            self.get_logger().info(f'Moved {name} underground.')
        
        # Pause for 1 second to let the physics engine update the positions.
        time.sleep(1.0)

    def teleport_robot(self):
        """
        Teleports the robot to the Stage 1 start position using Gazebo's service.
        This is instantaneous and bypasses physics/driving.
        """
        if not self.set_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gazebo /set_entity_state service not available.')
            return

        # Create a request object to set the state of an entity.
        req = SetEntityState.Request()
        req.state.name = 'maze_bot' # The name of our robot in Gazebo
        
        # Set the desired position from our class variable.
        req.state.pose.position.x = self.start_pose[0]
        req.state.pose.position.y = self.start_pose[1]
        req.state.pose.position.z = 0.05 # Slightly above ground to avoid getting stuck
        
        # Set orientation. We want yaw=0, which corresponds to Quaternion(0, 0, 0, 1).
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = 1.0
             
        # Call the service and wait for the result.
        future = self.set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
             self.get_logger().info(f'Teleported robot to {self.start_pose}')
        else:
             self.get_logger().warn('Failed to teleport robot.')

    def load_stage1_map(self):
        """
        Tells the Map Server to load the map file for Stage 1.
        """
        self.get_logger().info(f"Switching map to: {self.map1_path}")
        
        # Wait for the map server service.
        if not self.map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map server service not available!')
            return
            
        # Create the request with the path to the new map file.
        req = LoadMap.Request()
        req.map_url = self.map1_path
        
        # Call the service.
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Map switch result: {future.result().result}")
        else:
            self.get_logger().error("Map switch failed")

    def set_amcl_pose(self):
        """
        Sets the initial pose estimate for the AMCL localization system.
        Since we just teleported the robot, we need to tell the navigation stack
        where the robot is, so it doesn't get lost.
        """
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map' # The pose is relative to the global map frame
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        # Set the position to match where we teleported the robot.
        initial_pose.pose.position.x = self.start_pose[0]
        initial_pose.pose.position.y = self.start_pose[1]
        initial_pose.pose.orientation.w = 1.0 # Yaw = 0
             
        # Send this initial pose to Nav2.
        self.navigator.setInitialPose(initial_pose)
        
        # Wait a bit for AMCL to process the particle cloud.
        time.sleep(2.0)

    def go_to_goal(self):
        """
        Sends a navigation goal to the robot and monitors its progress.
        """
        # Create a PoseStamped message for the goal.
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        
        # Set the goal position.
        goal.pose.position.x = self.goal_pose[0]
        goal.pose.position.y = self.goal_pose[1]
        goal.pose.orientation.w = 1.0 # Orientation at the goal (facing East)
        
        # Send the goal to the navigation stack.
        self.navigator.goToPose(goal)
        
        # Loop to monitor the robot's progress.
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            # Get feedback from the navigator (e.g., distance remaining).
            feedback = self.navigator.getFeedback()
            
            # Print feedback every 5 iterations to avoid spamming the console.
            if feedback and i % 5 == 0:
                print(f'Distance remaining: {feedback.distance_remaining:.2f}')
            
        # Once the task is complete (success, failure, or canceled), check the result.
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

    def run(self):
        """
        The main execution sequence for Stage 1.
        """
        # 1. Hide Obstacles: Remove soft obstacles from the world.
        self.hide_soft_obstacles()
        
        # 2. Teleport Robot: Move robot to the start line.
        self.teleport_robot()
        
        # 3. Load Stage 1 Map: Ensure the navigation stack uses the correct map.
        self.load_stage1_map()
        
        # 4. Set AMCL Pose: Tell the robot where it is on the map.
        self.set_amcl_pose()
        
        # 5. Clear Costmaps: Reset the internal obstacle map to remove old data.
        self.navigator.clearAllCostmaps()
        
        # 6. Navigate: Send the robot to the goal.
        print("--- Starting Stage 1 Debug ---")
        self.go_to_goal()

def main():
    # Initialize ROS 2.
    rclpy.init()
    
    # Create our debugger node.
    debugger = Stage1Debugger()
    
    # Run the mission logic.
    debugger.run()
    
    # Clean up.
    rclpy.shutdown()

if __name__ == '__main__':
    main()

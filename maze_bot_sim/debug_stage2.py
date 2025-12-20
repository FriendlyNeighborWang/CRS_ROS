#! /usr/bin/env python3

# Import the ROS 2 Python client library.
import rclpy
# Import the Node class.
from rclpy.node import Node

# Import the BasicNavigator class for sending navigation goals.
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Import PoseStamped for position/orientation data.
from geometry_msgs.msg import PoseStamped

# Import LoadMap service to switch maps.
from nav2_msgs.srv import LoadMap

# Import Gazebo services for manipulating simulation objects.
# GetEntityState: To find out where an object is currently located.
from gazebo_msgs.srv import SetEntityState, GetModelList, GetEntityState

import sys
import os
import math
import time

class Stage2Debugger(Node):
    """
    This class handles the logic for Stage 2 of the maze challenge.
    Stage 2 involves:
    1. Restoring 'soft' obstacles (bringing them back to ground level).
    2. Teleporting the robot to the Stage 2 start position.
    3. Loading the map for Stage 2.
    4. Navigating the robot to the final goal.
    """
    def __init__(self):
        super().__init__('stage2_debugger')
        self.navigator = BasicNavigator()
        
        # Define Stage 2 Start Position (Bottom-Right of the maze).
        # x=3.2, y=0.3 meters.
        # Yaw=3.14 radians (approx 180 degrees), meaning the robot faces Left/West.
        self.start_pose = [3.2, 0.3, 3.14] 
        
        # Define Stage 2 Goal Position (Top-Middle-Left area).
        # x=1.6, y=3.2 meters.
        # Yaw=0.0 (facing East).
        self.goal_pose = [1.6, 3.2, 0.0]
        
        # Define paths for maps.
        self.map_base_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/maps'
        self.map2_path = os.path.join(self.map_base_path, 'maze_stage2.yaml')

        # Create service clients.
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')
        self.get_entity_state_client = self.create_client(GetEntityState, '/get_entity_state')

    def restore_soft_obstacles(self):
        """
        Restores all 'soft' obstacles to ground level.
        This is the opposite of 'hide_soft_obstacles' in Stage 1.
        """
        self.get_logger().info("Restoring soft obstacles...")
        
        # Check for service availability.
        if not self.get_model_list_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_model_list service not available.')
            return

        # 1. Get list of all models.
        future = self.get_model_list_client.call_async(GetModelList.Request())
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            self.get_logger().error('Failed to get model list')
            return

        model_names = future.result().model_names
        # Filter for soft obstacles.
        soft_obstacles = [name for name in model_names if name.startswith('soft_')]
        
        if not self.get_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_entity_state service not available.')
            return

        # 2. Restore each obstacle.
        for name in soft_obstacles:
            # First, we get the current state of the object.
            # We do this because we want to keep its X and Y position the same,
            # and only change its Z (height).
            get_req = GetEntityState.Request()
            get_req.name = name
            future_get = self.get_entity_state_client.call_async(get_req)
            rclpy.spin_until_future_complete(self, future_get)
            
            if future_get.result() is None or not future_get.result().success:
                continue
                
            # Extract the current pose.
            current_pose = future_get.result().state.pose
            
            # Create a request to set the new state.
            set_req = SetEntityState.Request()
            set_req.state.name = name
            set_req.state.pose = current_pose
            
            # Set Z to 0.25 meters. This brings it back above ground.
            set_req.state.pose.position.z = 0.25 
            
            # Send the request asynchronously.
            self.set_entity_state_client.call_async(set_req)
            self.get_logger().info(f'Restored {name}.')
            
        # Wait for physics to settle.
        time.sleep(1.0)

    def teleport_robot(self):
        """
        Teleports the robot to the Stage 2 start position in Gazebo.
        """
        if not self.set_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gazebo /set_entity_state service not available.')
            return

        req = SetEntityState.Request()
        req.state.name = 'maze_bot'
        req.state.pose.position.x = self.start_pose[0]
        req.state.pose.position.y = self.start_pose[1]
        req.state.pose.position.z = 0.05 
        
        # Convert Yaw (radians) to Quaternion (x, y, z, w).
        # Rotation around Z-axis:
        # z = sin(yaw / 2)
        # w = cos(yaw / 2)
        yaw = self.start_pose[2]
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
             
        future = self.set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
             self.get_logger().info(f'Teleported robot to {self.start_pose}')
        else:
             self.get_logger().warn('Failed to teleport robot.')

    def load_stage2_map(self):
        """
        Switches the navigation map to Stage 2.
        """
        self.get_logger().info(f"Switching map to: {self.map2_path}")
        
        if not self.map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map server service /map_server/load_map not available!')
            return
            
        req = LoadMap.Request()
        req.map_url = self.map2_path
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Map switch result: {future.result().result}")
        else:
            self.get_logger().error("Map switch failed")

    def set_amcl_pose(self):
        """
        Initializes the robot's position in the navigation stack (AMCL).
        """
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.start_pose[0]
        initial_pose.pose.position.y = self.start_pose[1]
        
        # Convert start yaw to quaternion for AMCL.
        yaw = self.start_pose[2]
        initial_pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.orientation.w = math.cos(yaw / 2.0)
             
        self.navigator.setInitialPose(initial_pose)
        time.sleep(2.0)

    def go_to_goal(self):
        """
        Sends the robot to the final goal.
        """
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_pose[0]
        goal.pose.position.y = self.goal_pose[1]
        
        # Convert goal yaw to quaternion.
        yaw = self.goal_pose[2]
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.navigator.goToPose(goal)
        
        i = 0
        last_recoveries = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback:
                # Check if the robot is stuck and trying to recover (e.g., spinning, backing up).
                if feedback.number_of_recoveries > last_recoveries:
                    print(f'\n[WARNING] Recovery behavior triggered! Total recoveries: {feedback.number_of_recoveries}')
                    last_recoveries = feedback.number_of_recoveries
                
                if i % 5 == 0:
                    print(f'[NAVIGATING] Distance remaining: {feedback.distance_remaining:.2f}')
            
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

    def run(self):
        """
        Main execution sequence for Stage 2.
        """
        # 1. Restore Obstacles: Bring back the soft obstacles.
        self.restore_soft_obstacles()
        
        # 2. Teleport to start: Move robot to Stage 2 start point.
        self.teleport_robot()
        
        # 3. Load Map: Switch to Stage 2 map.
        self.load_stage2_map()
        
        # 4. Set AMCL Pose: Initialize localization.
        self.set_amcl_pose()
        
        # 5. Clear Costmaps: Reset obstacle memory.
        self.navigator.clearAllCostmaps()
        
        # 6. Navigate: Go to the goal.
        print("Robot is moving to goal")
        self.go_to_goal()

def main():
    rclpy.init()
    debugger = Stage2Debugger()
    debugger.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

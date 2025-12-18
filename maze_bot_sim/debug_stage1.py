#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import LoadMap
from gazebo_msgs.srv import SetEntityState, GetModelList
import sys
import os
import math
import time

class Stage1Debugger(Node):
    def __init__(self):
        super().__init__('stage1_debugger')
        self.navigator = BasicNavigator()
        
        # Stage 1 Test Start (Center of Maze)
        self.start_pose = [0.9, 0.9, 0.0] 
        
        # Stage 1 Goal (Maze Start Point - Bottom Right)
        self.goal_pose = [1.6, 0.05, 0.0]
        
        self.map_base_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/maps'
        self.map1_path = os.path.join(self.map_base_path, 'maze_stage1.yaml')

        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')

    def hide_soft_obstacles(self):
        """Moves all soft obstacles underground to simulate transparency."""
        self.get_logger().info("Hiding soft obstacles...")
        if not self.get_model_list_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_model_list service not available.')
            return

        # 1. Get list of all models
        future = self.get_model_list_client.call_async(GetModelList.Request())
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            self.get_logger().error('Failed to get model list')
            return

        model_names = future.result().model_names
        # Identify soft obstacles
        soft_obstacles = [name for name in model_names if name.startswith('soft_')]
        
        if not self.set_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/set_entity_state service not available.')
            return

        # 2. Move them underground
        for name in soft_obstacles:
            req = SetEntityState.Request()
            req.state.name = name
            req.state.pose.position.x = 0.0 # Position doesn't matter if underground
            req.state.pose.position.y = 0.0
            req.state.pose.position.z = -10.0 # Underground
            
            # We call async and don't wait for each one to speed up
            self.set_entity_state_client.call_async(req)
            self.get_logger().info(f'Moved {name} underground.')
        
        # Give physics a moment to update
        time.sleep(1.0)

    def teleport_robot(self):
        """Teleports the robot to the Stage 1 start position."""
        if not self.set_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gazebo /set_entity_state service not available.')
            return

        req = SetEntityState.Request()
        req.state.name = 'maze_bot'
        req.state.pose.position.x = self.start_pose[0]
        req.state.pose.position.y = self.start_pose[1]
        req.state.pose.position.z = 0.05 
        
        # Yaw to Quaternion (0.0)
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = 1.0
             
        future = self.set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
             self.get_logger().info(f'Teleported robot to {self.start_pose}')
        else:
             self.get_logger().warn('Failed to teleport robot.')

    def load_stage1_map(self):
        self.get_logger().info(f"Switching map to: {self.map1_path}")
        if not self.map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Map server service not available!')
            return
            
        req = LoadMap.Request()
        req.map_url = self.map1_path
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Map switch result: {future.result().result}")
        else:
            self.get_logger().error("Map switch failed")

    def set_amcl_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.start_pose[0]
        initial_pose.pose.position.y = self.start_pose[1]
        initial_pose.pose.orientation.w = 1.0
             
        self.navigator.setInitialPose(initial_pose)
        time.sleep(2.0)

    def go_to_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_pose[0]
        goal.pose.position.y = self.goal_pose[1]
        goal.pose.orientation.w = 1.0
        
        self.navigator.goToPose(goal)
        
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(f'Distance remaining: {feedback.distance_remaining:.2f}')
            
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')

    def run(self):
        # 1. Hide Obstacles
        self.hide_soft_obstacles()
        
        # 2. Teleport Robot
        self.teleport_robot()
        
        # 3. Load Stage 1 Map
        self.load_stage1_map()
        
        # 4. Set AMCL Pose
        self.set_amcl_pose()
        
        # 5. Clear Costmaps
        self.navigator.clearAllCostmaps()
        
        # 6. Navigate
        print("--- Starting Stage 1 Debug ---")
        self.go_to_goal()

def main():
    rclpy.init()
    debugger = Stage1Debugger()
    debugger.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

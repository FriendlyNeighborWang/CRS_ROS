#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import LoadMap
from gazebo_msgs.srv import SetEntityState, GetModelList, GetEntityState
import sys
import os
import math
import time

class Stage2Debugger(Node):
    def __init__(self):
        super().__init__('stage2_debugger')
        self.navigator = BasicNavigator()
        
        # Stage 2 Start (Bottom-Right)
        # New Start: x=3.2, y=0.3
        # Yaw: 3.14 (facing left)
        self.start_pose = [3.2, 0.3, 3.14] 
        
        # Stage 2 Goal (Top-Middle-Left)
        # New End: x=1.6, y=3.2
        self.goal_pose = [1.6, 3.2, 0.0]
        
        self.map_base_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/maps'
        self.map2_path = os.path.join(self.map_base_path, 'maze_stage2.yaml')

        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.get_model_list_client = self.create_client(GetModelList, '/get_model_list')
        self.get_entity_state_client = self.create_client(GetEntityState, '/get_entity_state')

    def restore_soft_obstacles(self):
        """Restores all soft obstacles to ground level."""
        self.get_logger().info("Restoring soft obstacles...")
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
        soft_obstacles = [name for name in model_names if name.startswith('soft_')]
        
        if not self.get_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_entity_state service not available.')
            return

        for name in soft_obstacles:
            # Get current state (to keep X/Y)
            get_req = GetEntityState.Request()
            get_req.name = name
            future_get = self.get_entity_state_client.call_async(get_req)
            rclpy.spin_until_future_complete(self, future_get)
            
            if future_get.result() is None or not future_get.result().success:
                continue
                
            current_pose = future_get.result().state.pose
            
            # Set new state (Z=0.25)
            set_req = SetEntityState.Request()
            set_req.state.name = name
            set_req.state.pose = current_pose
            set_req.state.pose.position.z = 0.25 # Restore height
            
            self.set_entity_state_client.call_async(set_req)
            self.get_logger().info(f'Restored {name}.')
            
        time.sleep(1.0)

    def teleport_robot(self):
        """Teleports the robot to the Stage 2 start position in Gazebo."""
        if not self.set_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gazebo /set_entity_state service not available.')
            return

        req = SetEntityState.Request()
        req.state.name = 'maze_bot'
        req.state.pose.position.x = self.start_pose[0]
        req.state.pose.position.y = self.start_pose[1]
        req.state.pose.position.z = 0.05 
        
        # Yaw to Quaternion
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
        self.get_logger().info(f"Switching map to: {self.map2_path}")
        # Wait for service with a longer timeout and check if it exists
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
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.start_pose[0]
        initial_pose.pose.position.y = self.start_pose[1]
        
        yaw = self.start_pose[2]
        initial_pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.orientation.w = math.cos(yaw / 2.0)
             
        self.navigator.setInitialPose(initial_pose)
        # Wait a bit for AMCL to update
        time.sleep(2.0)

    def go_to_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = self.goal_pose[0]
        goal.pose.position.y = self.goal_pose[1]
        
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
        # 1. Restore Obstacles (in case Stage 1 hid them)
        self.restore_soft_obstacles()
        
        # 2. Teleport to start
        self.teleport_robot()
        
        # 3. Load Map
        self.load_stage2_map()
        
        # 4. Set AMCL Pose
        self.set_amcl_pose()
        
        # 5. Clear Costmaps
        self.navigator.clearAllCostmaps()
        
        # 6. Navigate
        print("Robot is moving to goal")
        self.go_to_goal()

def main():
    rclpy.init()
    debugger = Stage2Debugger()
    debugger.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

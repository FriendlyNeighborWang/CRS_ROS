#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import LoadMap
from gazebo_msgs.srv import SetEntityState
import sys
import os
import math
import time

class Stage2Debugger(Node):
    def __init__(self):
        super().__init__('stage2_debugger')
        self.navigator = BasicNavigator()
        
        # Stage 2 Start (Top-Left)
        # Using the "safe" coordinates: x=0.22, y=1.5
        # Yaw: -1.57 (facing down)
        self.start_pose = [0.22, 1.5, -1.57] 
        
        # Stage 2 Goal (Top-Right)
        self.goal_pose = [1.6, 1.6, 0.0]
        
        self.map_base_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/maps'
        self.map2_path = os.path.join(self.map_base_path, 'maze_stage2.yaml')

        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state')

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
        # 1. Teleport to start
        self.teleport_robot()
        
        # 2. Load Map
        self.load_stage2_map()
        
        # 3. Set AMCL Pose
        self.set_amcl_pose()
        
        # 4. Clear Costmaps
        self.navigator.clearAllCostmaps()
        
        # 5. Navigate
        print("--- Starting Stage 2 Debug ---")
        self.go_to_goal()

def main():
    rclpy.init()
    debugger = Stage2Debugger()
    debugger.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

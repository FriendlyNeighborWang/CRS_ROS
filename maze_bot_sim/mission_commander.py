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

class MissionCommander(Node):
    def __init__(self):
        super().__init__('mission_commander')
        self.navigator = BasicNavigator()
        
        # Coordinates
        # Initial Pose: Bottom-Right (Spawn Point)
        self.initial_pose = [1.6, 0.2, 3.14] # x, y, yaw (facing left)
        
        # Goal 1: Start Position (Top-Left) - Stage 1 (Ignore Soft Obstacles)
        # Adjusted to be centered between wall (x=0) and obstacle (x=0.45)
        # Robot radius is 0.10, so x=0.22 is safe.
        # Yaw set to -1.57 (facing down) to prepare for next move
        self.goal1 = [0.22, 1.5, -1.57]        
        
        # Goal 2: End Position (Top-Right) - Stage 2 (Respect Soft Obstacles)
        self.goal2 = [1.6, 1.6, 0.0]
        
        # Map Paths
        # Assuming the script is run from the workspace or paths are absolute
        # We will construct absolute paths based on the package location
        # But for simplicity, we'll hardcode the expected location or pass it as arg
        self.map_base_path = '/home/yahboom/yahboomcar_ws/src/maze_bot_sim/maps'
        self.map1_path = os.path.join(self.map_base_path, 'maze_stage1.yaml')
        self.map2_path = os.path.join(self.map_base_path, 'maze_stage2.yaml')

        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state')

    def reset_robot_pose_gazebo(self):
        """Resets the robot pose in Gazebo to the initial spawn point."""
        if not self.set_entity_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Gazebo /set_entity_state service not available. Cannot reset robot pose.')
            return

        req = SetEntityState.Request()
        req.state.name = 'maze_bot'
        req.state.pose.position.x = self.initial_pose[0]
        req.state.pose.position.y = self.initial_pose[1]
        req.state.pose.position.z = 0.05 # Slightly above ground
        
        # Yaw to Quaternion (approx for 3.14/180 deg)
        if self.initial_pose[2] > 3.0:
             req.state.pose.orientation.z = 1.0
             req.state.pose.orientation.w = 0.0
        else:
             req.state.pose.orientation.z = 0.0
             req.state.pose.orientation.w = 1.0
             
        future = self.set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
             self.get_logger().info('Successfully reset robot pose in Gazebo.')
        else:
             self.get_logger().warn('Failed to reset robot pose in Gazebo.')

    def set_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_pose[0]
        initial_pose.pose.position.y = self.initial_pose[1]
        # Simple conversion for yaw=3.14 (facing left/west) -> q_z=1.0, q_w=0.0 approx
        # Or use transforms3d but let's keep it simple for now since we just need rough init
        if self.initial_pose[2] > 3.0:
             initial_pose.pose.orientation.z = 1.0
             initial_pose.pose.orientation.w = 0.0
        else:
             initial_pose.pose.orientation.z = 0.0
             initial_pose.pose.orientation.w = 1.0
             
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

    def go_to_pose(self, x, y, yaw=0.0):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        
        # Convert yaw (radians) to quaternion
        # q_w = cos(yaw/2), q_z = sin(yaw/2) for rotation around Z axis
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.navigator.goToPose(goal_pose)
        
        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(f'Distance remaining: {feedback.distance_remaining:.2f}')
                # Some navigation timeout to demo cancellation
                # if feedback.navigation_time > 600.0:
                #     self.navigator.cancelTask()
            
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            return True
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            return False
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            # Try to get more info if possible, though BasicNavigator doesn't expose much
            return False
        return False

    def switch_map(self, map_path):
        print(f"Switching map to: {map_path}")
        while not self.map_client.wait_for_service(timeout_sec=1.0):
            print('Map server service not available, waiting...')
            
        req = LoadMap.Request()
        req.map_url = map_path
        future = self.map_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            print(f"Map switch result: {future.result().result}")
        else:
            print("Map switch failed")

    def run(self):
        # Reset Gazebo Pose first
        self.reset_robot_pose_gazebo()
        
        # Stage 1
        print("--- Starting Stage 1 ---")
        self.set_initial_pose()
        print("Navigating to Goal 1 (Start Position - Top Left)...")
        # Pass yaw from goal1
        if not self.go_to_pose(self.goal1[0], self.goal1[1], self.goal1[2]):
            print("Stage 1 Failed. Aborting mission.")
            return
        
        # Switch Map
        print("--- Switching to Stage 2 Map ---")
        self.switch_map(self.map2_path)
        
        # Clear costmaps to remove any artifacts from previous map or dynamic obstacles
        print("Clearing costmaps...")
        self.navigator.clearAllCostmaps()
        
        # Stage 2
        print("--- Starting Stage 2 ---")
        # Print current pose to debug
        # We need to subscribe to amcl_pose or just trust the last goal?
        # Let's just proceed.
        
        print("Navigating to Goal 2 (End Position - Top Right)...")
        if not self.go_to_pose(self.goal2[0], self.goal2[1]):
            print("Stage 2 Failed.")
            return
        
        print("Mission Complete!")

def main():
    rclpy.init()
    commander = MissionCommander()
    commander.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

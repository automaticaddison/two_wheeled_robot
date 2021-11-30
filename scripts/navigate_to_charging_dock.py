#! /usr/bin/env python3

"""
Description:
  Navigate to a charging dock once the battery gets low.
-------
Subscription Topics:
  Current battery state
  /battery_status - sensor_msgs/BatteryState
-------
Publishing Topics:
  Velocity command to navigate to the charging dock.
  /cmd_vel - geometry_msgs/Twist
-------
Author: Addison Sears-Collins
Website: AutomaticAddison.com
Date: November 16, 2021
"""

import time  # Time library

from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor
from robot_navigator import BasicNavigator, NavigationResult # Helper module
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist # Velocity command
from sensor_msgs.msg import BatteryState # Battery status

# Holds the current state of the battery
this_battery_state = BatteryState()
prev_battery_state = BatteryState()

# Flag for detecting the change in the battery state
low_battery = False
low_battery_min_threshold = 0.25

class ConnectToChargingDockNavigator(Node):
    """
    Navigates and connects to the charging dock
    """      
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('connect_to_charging_dock_navigator')
    
      # Create a publisher
      # This node publishes the desired linear and angular velocity of the robot
      self.publisher_cmd_vel = self.create_publisher(
        Twist,
        '/cmd_vel',
        10)  
      timer_period = 0.1
      self.timer = self.create_timer(timer_period, self.navigate_to_dock)
        
      # Declare velocities
      self.linear_velocity = 0.0
      self.angular_velocity = 0.15
      
    def navigate_to_dock(self):
      global low_battery
      
      if low_battery == False:
        return None 
      
      self.get_logger().info('Navigating to the charging dock...')
      
      # Launch the ROS 2 Navigation Stack
      navigator = BasicNavigator()

      # Wait for navigation to fully activate. Use this line if autostart is set to true.
      navigator.waitUntilNav2Active()

      # If desired, you can change or load the map as well
      # navigator.changeMap('/path/to/map.yaml')

      # You may use the navigator to clear or obtain costmaps
      # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
      # global_costmap = navigator.getGlobalCostmap()
      # local_costmap = navigator.getLocalCostmap()

      # Set the robot's goal pose
      goal_pose = PoseStamped()
      goal_pose.header.frame_id = 'map'
      goal_pose.header.stamp = navigator.get_clock().now().to_msg()
      goal_pose.pose.position.x = 0.0
      goal_pose.pose.position.y = 2.0
      goal_pose.pose.position.z = 0.25
      goal_pose.pose.orientation.x = 0.0
      goal_pose.pose.orientation.y = 0.0
      goal_pose.pose.orientation.z = 0.0
      goal_pose.pose.orientation.w = 1.0

      # Go to the goal pose
      navigator.goToPose(goal_pose)

      i = 0

      # Keep doing stuff as long as the robot is moving towards the goal
      while not navigator.isNavComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
          print('Distance remaining: ' + '{:.2f}'.format(
            feedback.distance_remaining) + ' meters.')
   
          # Some navigation timeout to demo cancellation
          if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            navigator.cancelNav()

      # Do something depending on the return code
      result = navigator.getResult()
      if result == NavigationResult.SUCCEEDED:
        print('Successfully reached charging dock staging area...')
        low_battery = False
        self.connect_to_dock()
      elif result == NavigationResult.CANCELED:
        print('Goal was canceled!')
      elif result == NavigationResult.FAILED:
        print('Goal failed!')
      else:
        print('Goal has an invalid return status!')  
        
    def connect_to_dock(self):  
      
      # While the battery is not charging
      while this_battery_state.power_supply_status != 1:
    
        # Publish the current battery state
        self.get_logger().info('NOT CHARGING...')
      
        # Send the velocity command to the robot by publishing to the topic
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.linear_velocity
        cmd_vel_msg.angular.z = self.angular_velocity
        self.publisher_cmd_vel.publish(cmd_vel_msg)      
        time.sleep(0.1)
    
      # Stop the robot
      cmd_vel_msg = Twist()
      cmd_vel_msg.linear.x = 0.0
      cmd_vel_msg.angular.z = 0.0
      self.publisher_cmd_vel.publish(cmd_vel_msg)
    
      self.get_logger().info('CHARGING...')
      self.get_logger().info('Successfully connected to the charging dock!')

class BatteryStateSubscriber(Node):
    """
    Subscriber node to the current battery state
    """      
    def __init__(self):
  
      # Initialize the class using the constructor
      super().__init__('battery_state_subscriber')
    
      # Create a subscriber 
      # This node subscribes to messages of type
      # sensor_msgs/BatteryState
      self.subscription_battery_state = self.create_subscription(
        BatteryState,
        '/battery_status',
        self.get_battery_state,
        10)
      
    def get_battery_state(self, msg):
      """
      Update the current battery state.
      """
      global this_battery_state
      global prev_battery_state
      global low_battery
      prev_battery_state = this_battery_state
      this_battery_state = msg
      
      # Check for low battery
      if prev_battery_state.percentage >= low_battery_min_threshold and this_battery_state.percentage < low_battery_min_threshold:
        low_battery = True
        
def main(args=None):
  """
  Entry point for the program.
  """
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  try: 
  
    # Create the nodes
    connect_to_charging_dock_navigator = ConnectToChargingDockNavigator()
    battery_state_subscriber = BatteryStateSubscriber()
    
    # Set up mulithreading
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(connect_to_charging_dock_navigator)
    executor.add_node(battery_state_subscriber)
    
    try:
      # Spin the nodes to execute the callbacks
      executor.spin()
    finally:
      # Shutdown the nodes
      executor.shutdown()
      connect_to_charging_dock_navigator.destroy_node()
      battery_state_subscriber.destroy_node()

  finally:
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
  main()

#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by AutomaticAddison.com

import time # Time library
from copy import deepcopy

from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from robot_navigator import BasicNavigator, NavigationResult # Helper module

# Euler angle to quaternion conversion
from two_wheeled_robot.euler_to_quaternion import get_quaternion_from_euler 

'''
Inspection demo. We assume the robot has cameras or other sensors to 
collect information during inspection.
'''
def main():

  # Start the ROS 2 Python Client Library
  rclpy.init()

  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()

  # Inspection route, probably read in from a file for a real application
  # from either a map or drive and repeat. 
  # The values are [x in meters, y in meters, yaw in radians].
  inspection_route = [
    [4.0, 0.0, 0],
    [7.0, -2.5, -0.785],
    [0.0, -4.0, 3.1415],
    [-2.5, -1.5, 3.1415],
    [-5.0, -4.0, 4.71],
    [-2.5, -1.5, 1.5708],
    [-8.0, -0.25, 0]]

  # Set the robot's initial pose 
  initial_pose = PoseStamped()
  initial_pose.header.frame_id = 'map'
  initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  initial_pose.pose.position.x = 0.0
  initial_pose.pose.position.y = 0.0
  initial_pose.pose.position.z = 0.0
  initial_pose.pose.orientation.x = 0.0
  initial_pose.pose.orientation.y = 0.0
  initial_pose.pose.orientation.z = 0.0
  initial_pose.pose.orientation.w = 1.0
  navigator.setInitialPose(initial_pose)

  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active()

  # Send our route
  inspection_points = []
  inspection_pose = PoseStamped()
  inspection_pose.header.frame_id = 'map'
  inspection_pose.header.stamp = navigator.get_clock().now().to_msg()

  for pt in inspection_route:
  
    # Set the position of waypoint along inspection route
    inspection_pose.pose.position.x = pt[0]
    inspection_pose.pose.position.y = pt[1]
    inspection_pose.pose.position.z = 0.0
    
    # Set the orientation of waypoint along inspection route   
    quat = get_quaternion_from_euler(0.0, 0.0, pt[2])     
    inspection_pose.pose.orientation.x = quat[0]
    inspection_pose.pose.orientation.y = quat[1]
    inspection_pose.pose.orientation.z = quat[2]
    inspection_pose.pose.orientation.w = quat[3]    
    inspection_points.append(deepcopy(inspection_pose))
  
  nav_start = navigator.get_clock().now()
  navigator.followWaypoints(inspection_points)

  # Do something during our route (e.g. Analyze world or upload to the cloud)
  # Simply the current waypoint ID for the demonstation
  i = 0
  while not navigator.isNavComplete():
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Executing current waypoint: ' +
        str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

  result = navigator.getResult()
  if result == NavigationResult.SUCCEEDED:
    print('Inspection is complete! Returning to start...')
  elif result == NavigationResult.CANCELED:
    print('Inspection was canceled. Returning to start...')
    exit(1)
  elif result == NavigationResult.FAILED:
    print('Inspection failed! Returning to start...')

  # go back to start
  initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  navigator.goToPose(initial_pose)
  while not navigator.isNavComplete():
    pass

  exit(0)

if __name__ == '__main__':
    main()

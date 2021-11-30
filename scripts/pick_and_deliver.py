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
from copy import deepcopy # Modifying a deep copy does not impact the original

from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from robot_navigator import BasicNavigator, NavigationResult # Helper module

# Positions for picking up items
pick_positions = {
  "front_reception_desk": [-2.5, 1.4],
  "rear_reception_desk": [-0.36, 20.0],
  "main_conference_room": [18.75, 15.3],
  "main_break_room": [15.5, 5.4]}

# Positions for delivery of items
shipping_destinations = {
  "front_reception_desk": [-2.5, 1.4],
  "rear_reception_desk": [-0.36, 20.0],
  "main_conference_room": [18.75, 15.3],
  "main_break_room": [15.5, 5.4]}

'''
Pick up an item in one location and deliver it to another. 
The assumption is that there is a person at the pick and delivery location
to load and unload the item from the robot.
'''
def main():
  # Recieved virtual request for picking item at front_reception_desk and bring to
  # employee at main_conference_room. This request would
  # contain the pick position ("front_reception_desk") and shipping destination ("main_conference_room")
  ####################
  request_item_location = 'front_reception_desk'
  request_destination = 'main_conference_room'
  ####################

  # Start the ROS 2 Python Client Library
  rclpy.init()

  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()

  # Set the robot's initial pose if necessary
  # initial_pose = PoseStamped()
  # initial_pose.header.frame_id = 'map'
  # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  # initial_pose.pose.position.x = 0.0
  # initial_pose.pose.position.y = 1.0
  # initial_pose.pose.position.z = 0.0
  # initial_pose.pose.orientation.x = 0.0
  # initial_pose.pose.orientation.y = 0.0
  # initial_pose.pose.orientation.z = 0.0
  # initial_pose.pose.orientation.w = 1.0
  # navigator.setInitialPose(initial_pose)

  # Wait for navigation to fully activate
  navigator.waitUntilNav2Active()

  # Set the pick location  
  pick_item_pose = PoseStamped()
  pick_item_pose.header.frame_id = 'map'
  pick_item_pose.header.stamp = navigator.get_clock().now().to_msg()
  pick_item_pose.pose.position.x = pick_positions[request_item_location][0]
  pick_item_pose.pose.position.y = pick_positions[request_item_location][1]
  pick_item_pose.pose.position.z = 0.0
  pick_item_pose.pose.orientation.x = 0.0
  pick_item_pose.pose.orientation.y = 0.0
  pick_item_pose.pose.orientation.z = 0.0
  pick_item_pose.pose.orientation.w = 1.0
  print('Received request for item picking at ' + request_item_location + '.')
  navigator.goToPose(pick_item_pose)

  # Do something during our route
  # (e.g. queue up future tasks or detect person for fine-tuned positioning)
  # Simply print information for employees on the robot's distance remaining
  i = 0
  while not navigator.isNavComplete():
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Distance remaining: ' + '{:.2f}'.format(
            feedback.distance_remaining) + ' meters.')

  result = navigator.getResult()
  if result == NavigationResult.SUCCEEDED:
    print('Got product from ' + request_item_location +
          '! Bringing product to shipping destination (' + request_destination + ')...')
    shipping_destination = PoseStamped()
    shipping_destination.header.frame_id = 'map'
    shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
    shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
    shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
    shipping_destination.pose.position.z = 0.0
    shipping_destination.pose.orientation.x = 0.0
    shipping_destination.pose.orientation.y = 0.0
    shipping_destination.pose.orientation.z = 0.0
    shipping_destination.pose.orientation.w = 1.0
    navigator.goToPose(shipping_destination)

  elif result == NavigationResult.CANCELED:
    print('Task at ' + request_item_location + ' was canceled. Returning to staging point...')
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)

  elif result == NavigationResult.FAILED:
    print('Task at ' + request_item_location + ' failed!')
    exit(-1)

  while not navigator.isNavComplete():
    pass

  exit(0)

if __name__ == '__main__':
  main()

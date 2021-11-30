#!/usr/bin/env python3 

# Test code for controlling the lift mechanism on a warehouse robot
# Author: Addison Sears-Collins
# Website: https://automaticaddison.com

# ROS Client Library for Python
import rclpy
 
# Handles the creation of nodes
from rclpy.node import Node
 
# Enables usage of the standard message
import std_msgs.msg

# Enables use of sensor messages
import sensor_msgs.msg 

 
class LiftController(Node):
  """
  Create a LiftController class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('lift_controller')
     
    # Create the publisher. This publisher will publish a JointState message
    # to a topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(sensor_msgs.msg.JointState, 'joint_states', 10)

    # Create the subscriber. This subscriber will subscribe to a JointState message
    self.subscription = self.create_subscription(sensor_msgs.msg.JointState, 'joint_states', self.listener_callback, 10)
         
  def listener_callback(self, msg):
    """
    Callback function.
    """
    # Create a JointStates message
    new_msg = sensor_msgs.msg.JointState()
 
    # Set the message's data
    new_msg.header.stamp = self.get_clock().now().to_msg()
    new_msg.name = msg.name
    new_msg.position = msg.position
    new_msg.position[2] = 0.30 
     
    # Publish the message to the topic
    self.publisher_.publish(new_msg)
     
def main(args=None):
 
  # Initialize the rclpy library
  rclpy.init(args=args)
 
  # Create the node
  lift_controller = LiftController()
 
  # Spin the node so the callback function is called.
  rclpy.spin(lift_controller)
 
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  lift_controller.destroy_node()
 
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()

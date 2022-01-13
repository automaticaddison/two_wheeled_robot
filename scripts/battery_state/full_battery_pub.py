#!/usr/bin/env python3 

"""
Description:
Publish the battery state at a specific time interval
In this case, the battery is a full battery.
-------
Publishing Topics:
/battery_status – sensor_msgs/BatteryState
-------
Subscription Topics:
None
-------
Author: Addison Sears-Collins
Website: AutomaticAddison.com
Date: January 13, 2022
"""
 
import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from sensor_msgs.msg import BatteryState # Enable use of the sensor_msgs/BatteryState message type
 
class BatteryStatePublisher(Node):
  """
  Create a BatteryStatePublisher class, which is a subclass of the Node class.
  The class publishes the battery state of an object at a specific time interval.
  """
  
  def __init__(self):
    """
    Class constructor to set up the node
    """
   
    # Initiate the Node class's constructor and give it a name
    super().__init__('battery_state_pub')
     
    # Create publisher(s)  
     
    # This node publishes the state of the battery.
    # Maximum queue size of 10. 
    self.publisher_battery_state = self.create_publisher(BatteryState, '/battery_status', 10)
     
    # Time interval in seconds
    timer_period = 0.1 
    self.timer = self.create_timer(timer_period, self.get_battery_state)
    
    # Initialize battery level
    self.voltage = 9.0 # Initialize the battery voltage level
    self.percentage = 1.0  # Initialize the percentage charge level
    self.power_supply_status = 3 # Initialize the power supply status (e.g. NOT CHARGING)
     
  def get_battery_state(self):
    """
    Callback function.
    This function reports the battery state at the specific time interval.
    """
    msg = BatteryState() # Create a message of this type 
    msg.voltage = self.voltage
    msg.percentage = self.percentage
    msg.power_supply_status = self.power_supply_status
    self.publisher_battery_state.publish(msg) # Publish BatteryState message 
   
def main(args=None):
 
  # Initialize the rclpy library
  rclpy.init(args=args)
 
  # Create the node
  battery_state_pub = BatteryStatePublisher()
 
  # Spin the node so the callback function is called.
  # Publish any pending messages to the topics.
  rclpy.spin(battery_state_pub)
 
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  battery_state_pub.destroy_node()
 
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()

#!/usr/bin/env python

"""
This module contains the definition of the Action Client Class

.. module::node_a
   :platform:Unix
   :synopsys:This code creates a ROS node(Action Client)
   
.. moduleauthor:: Tomoha Neki tomohaneki@keio.jp

It subscribes to the odem topic to get the position and velocity information

It publishes these information as a custom massage ("pos") on the topic ("/robot_pos" )
"""
import rospy
import actionlib 
from geometry_msgs.msg import Point
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import pos, PlanningAction, PlanningGoal

class  ActionClient:
    def __init__(self):
        """
        Initialize the ActionClient
        
        This function initialize the ROS nodeand sets up publishers, subscribers, and action 		client.
      
        
        """
         
        # Initialize the ROS node
        #rospy.init_node('ActionClient')

        # Create a publisher for the robot position information 
        self.pos_pub = rospy.Publisher('/robot_pos',pos, queue_size=1)
        
        #For Action Client 
        # Create an action client for the reaching_goal action
        self.action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        #wait for server to be ready
        self.action_client.wait_for_server()
        
        # Create a publisher for the target coordinates
        self.target_pub = rospy.Publisher('/target_coordinates', Point, queue_size=1)
        
        # Create a subscriber for the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Create a goal message
        self.goal = PlanningGoal()

    def odom_callback(self, data):
        """
        Callback function for /odom topic
        
        Extracts position and linear velocity information from the odometry message and publishes 		it as a custom message. 
        
        :param data: Odometry message containing position and velocity information
        
        
        """
        # Extracts position and linear velocity information from the message 
        position = data.pose.pose.position
        linear_velocity = data.twist.twist.linear

        # Create a custom message
        pos_msg = pos()
        pos_msg.x = position.x
        pos_msg.y = position.y
        pos_msg.vx = linear_velocity.x
        pos_msg.vy = linear_velocity.y

        # Publish the custom message
        self.pos_pub.publish(pos_msg)

    def set_goal(self, x, y):
        """
        Set goal coordinates and send the goal to the action server.
        
        :param x: X coordinate of the goal(float)
        :param y: Y coordinate of the goal(float)
        """
        # Set the goal coordinates in the action goal message
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y

        # Send the goal to the action server
        self.action_client.send_goal(self.goal)
        
        # Publish the target coordinates
        target_msg = Point()
        target_msg.x = x
        target_msg.y = y
        self.target_pub.publish(target_msg)

    def cancel_goal(self):
        """
        Cancel the current goal.
        """
        # Cancel the current goal
        self.action_client.cancel_goal()
        rospy.loginfo("Goal cancelled")

def main():
    # Initialize the ROS node
    rospy.init_node('ActionClient')
    
    # Create an object for the  ActionClient class
    Action_Client =  ActionClient()

    # Get user input to set the goal or cancel it
    while not rospy.is_shutdown():
        user_input = input("Enter goal coordinates (x,y) or 'c' to cancel: ")

        if user_input.lower() == 'c':
            Action_Client.cancel_goal()
          
        else:
            try:
                x, y = map(float, user_input.split(','))
                Action_Client.set_goal(x, y)
            except ValueError:
                rospy.logwarn("Invalid input. Please enter the goal coordinates in the format 'x,y'")
    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    main()


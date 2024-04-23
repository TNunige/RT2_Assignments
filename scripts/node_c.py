#!/usr/bin/env python
"""

This module creates a ROS service 

.. module:: node_c
   :synopsis: This code creates a ROS service to provide statistics
   
.. moduleauthor:: Tomoha Neki
"""

import rospy
from assignment_2_2023.srv import GetRobotStats, GetRobotStatsResponse
from assignment_2_2023.msg import pos
from geometry_msgs.msg import Point
from math import sqrt

class RobotStatsServer:
    """
    RobotStatsServer Class
    
    This class represents a ROS service to provide statistics about the robot
    """
    def __init__(self):
        
        # Initialize the ROS node
        rospy.init_node('robot_stats_server')

        # Variables to store robot position and velocity
        self.robot_position = Point()
        self.robot_velocity = Point()
        
        # Flag to indicate whether data has been received
        self.data_received = False
        
        # Get the parameter for window
        self.averaging_window_size = rospy.get_param('window_size', 10)
        
        # Create a service for getting robot statistics
        rospy.Service('get_robot_stats', GetRobotStats, self.handle_get_robot_stats)
        rospy.loginfo("Get Robot Stats service is ready.")

        # Create a subscriber for the robot position and velocity
        rospy.Subscriber('/robot_pos', pos, self.pos_callback)

    def pos_callback(self, data):
        """
        Callback function for the /robot_position topic
        
        :param data: Message containing the robot position and velocity
        """
        
        self.robot_position = Point()
        self.robot_position.x = data.x
        self.robot_position.y = data.y
        
        self.robot_velocity = Point()
        self.robot_velocity.x = data.vx
        self.robot_velocity.y = data.vy
        
        # Set the flag to indicate that data has been received
        self.data_received = True

    def handle_get_robot_stats(self, request):
        """
        Callback function for the 'get_robot_stats' service
        
        :param request: Service request containing the target coordinates
        :return: Service response containing the distance to the target and average speed of the 		robot
        """
        
        response = GetRobotStatsResponse()
        
        # Check if data has been received
        if not self.data_received:
            rospy.logwarn("No data received. Check the /robot_position topic.")
            return response

        # Calculate the distance from the robot to the target using the Euclidean distance formula
        distance_to_target = sqrt((self.robot_position.x - request.target.x)**2 +
                                  (self.robot_position.y - request.target.y)**2)
        response.distance_to_target = distance_to_target

        # Calculate the average speed of the robot
        average_speed = sqrt(self.robot_velocity.x**2 + self.robot_velocity.y**2)
        response.average_speed = average_speed

        return response

def robot_stats_server():
    """
    Main function
    """
    # Create an object for the RobotStatsServer class
    robot_stats_server = RobotStatsServer()

    # Spin to keep the service node alive
    rospy.spin()

if __name__ == '__main__':
    robot_stats_server()


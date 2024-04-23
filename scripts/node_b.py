#!/usr/bin/env python
"""

This module creates a ROS service server to provide the last target coordinates

.. module:: node_b
    :synopsis: This code creates a ROS service server

.. moduleauthor:: Tomoha Neki tomohaneki@keio.jp
"""

import rospy
from assignment_2_2023.srv import GetLastTarget, GetLastTargetResponse
from geometry_msgs.msg import Point

# Global variable to store the last target coordinates
last_target = Point()

# Callback function for the 'get_last_target' service
def handle_get_last_target(request):
    """
    Callback function for the ROS service
    
    :param request: Service request
    :return: Service response containing the last target coordinates
    """

    #Declare the variables as global
    global last_target
    
    response = GetLastTargetResponse()
    response.last_target = last_target
    return response
# Callback function for the target coordinates
def target_callback(msg):
    """
    Callback function for the target coordinates topic
    
    :param msg: Message containing the target coordinates
    """
    global last_target
    last_target = msg   
# Callback function for the '/target_coordinates' topic
def get_last_target_server():
    """
    Main function
    """
    # Initialize the ROS node
    rospy.init_node('get_last_target_server')

    # Create a service for getting the last target coordinates
    rospy.Service('get_last_target', GetLastTarget, handle_get_last_target)
    rospy.loginfo("Get Last Target service is ready.")
    
    # Create a subscriber for the target coordinates
    rospy.Subscriber('/target_coordinates', Point, target_callback)

    # Spin to keep the service node alive
    rospy.spin()

if __name__ == '__main__':
    get_last_target_server()


#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import Point
from mrs_formation_reshaping.srv import SrvGetAssignment, SrvGetAssignmentRequest
import sys

def generate_random_points(num_points, box_size):
    """
    Generate a list of random points within a cubic box.
    :param num_points: Number of points to generate
    :param box_size: Size of the box (assumed cubic)
    :return: List of geometry_msgs/Point
    """
    points = []
    for _ in range(num_points):
        point = Point()
        point.x = random.uniform(-box_size / 2, box_size / 2)
        point.y = random.uniform(-box_size / 2, box_size / 2)
        point.z = random.uniform(-box_size / 2, box_size / 2)
        points.append(point)
    return points

def main():
    rospy.init_node('get_assignment_client')

    # Parameters
    num_points = 100  # Default number of points
    box_size = 50  # Box dimensions in meters for initial and goal configurations

    if len(sys.argv) >= 2:
        try:
            num_points = int(sys.argv[1])  # Parse the argument as a number
            print(f"Provided number of points: {num_points}")
        except ValueError:
            print("Provided number of points is invalid. Using default number: {num_points}")

    if len(sys.argv) >= 3: 
        try:
            box_size = float(sys.argv[2])  # Parse the argument as a number
            print(f"Provided box_size: {box_size}")
        except ValueError:
            print("Provided box size is invalid. Using default number: {box_size}")
    

    rospy.loginfo(f"Generating {num_points} initial points in a {box_size}x{box_size}x{box_size} box.")
    rospy.loginfo(f"Generating {num_points} goal points in a {box_size}x{box_size}x{box_size} box.")

    # Wait for the service to become available
    service_name = '/mrs_formation_reshaping/get_reshaping_assignment'
    rospy.wait_for_service(service_name)

    try:
        # Create the service proxy
        get_assignment_service = rospy.ServiceProxy(service_name, SrvGetAssignment)

        # Fill the request
        request = SrvGetAssignmentRequest()
        request.initial_configurations = generate_random_points(num_points, box_size)
        request.goal_configurations = generate_random_points(num_points, box_size)
        request.publish_visualization = True

        rospy.loginfo(f"Sending request with {len(request.initial_configurations)} initial points and {len(request.goal_configurations)} goal points.")

        # Call the service
        response = get_assignment_service(request)

        # Handle the response
        if response.success:
            rospy.loginfo("Service call succeeded.")
        else:
            rospy.logerr(f"Service call failed: {response.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    main()

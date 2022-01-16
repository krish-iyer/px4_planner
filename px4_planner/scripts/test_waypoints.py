#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from px4_planner.srv import target_queue

class test_wapoints:
    def __init__(self):
        self.waypoints_list = []
        self.waypoint_server = rospy.ServiceProxy('/px4_planner/target_queue', target_queue)

    def add_waypoints(self, points):
        for point in points:
            self.waypoints_list.append(Point(point[0], point[1], point[2]))

    def call_waypoint_server(self):
        resp = self.waypoint_server(self.waypoints_list)
        return resp.status

if __name__ == '__main__':

    rospy.init_node('test_waypoint', anonymous=True)

    test_wapoints_obj = test_wapoints()
    points = [[-1,-6,0], [0,0,0], [-1,7,0], [-1,-6,0]]
    test_wapoints_obj.add_waypoints(points)
    print(test_wapoints_obj.call_waypoint_server())

    rospy.spin()
#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PoseStamped
from px4_planner.srv import target, target_queue
import threading
import time
from mavros_msgs.srv import SetMode, CommandBool

class DroneControl:
    def __init__(self):
        self.goal_loc_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.cmd = PoseStamped()

    def arm(self, action):
        exec_cmd = CommandBool()
        exec_cmd.value = action
        res = self.arming(action)
        return res.success
    
    def setMode(self):
        res = self.set_mode(base_mode=0, custom_mode="OFFBOARD")
        return res.mode_sent

class Planner:
    def __init__(self, drone):
        rospy.Subscriber("/laser/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/mavros/local_position/pose",PoseStamped, self.pose_callback)
        self.features_pub = rospy.Publisher("/features/marker", Marker, queue_size=1)
        self.robot_pose_marker = rospy.Publisher("/features/robot_marker", Marker, queue_size=1)
        self.goal_marker = rospy.Publisher("/features/goal_marker", Marker, queue_size=1)
        self.target_srv = rospy.Service('/px4_planner/target', target, self.goal)
        self.target_queue_srv = rospy.Service('/px4_planner/target_queue', target_queue, self.goal_queue)
        self.obstacles = [[]]
        self.goal = []
        self.drone = drone
        self.execute_goal_th = threading.Thread()
        self.execute_goal_queue_th = threading.Thread()
        self.execute_goal_th_status = False
        self.current_loc = []
        self.current_goal = []
        self.obstacle_obstruct = False
        self.roam = False
        self.nearest_goal_offset = 0.5#rospy.get_param("~nearest_goal_offset")
        self.nearest_goal_min = 4#rospy.get_param("~nearest_goal_min")
        self.interm_goal_resolution = 0.5#rospy.get_param("~interm_goal_resolution")
        self.final_goal_resolution = 0.1#rospy.get_param("~final_goal_resolution")
        # self.inflation_line_segement = rospy.get_param("~inflation_line_segement")
        # self.interm_goal_distance = rospy.get_param("~interm_goal_distance")
        self.target_cordinate_queue = []

    def current_goal_reached(self):

        if(len(self.current_goal)):
            if((abs(self.current_goal[0] - self.current_loc[0])) < self.interm_goal_resolution and (abs(self.current_goal[1] - self.current_loc[1]) < self.interm_goal_resolution)):
                return True
            else: 
                return False

        return False

    def final_goal_reached(self):

        if(len(self.goal)):
            if((abs(self.goal[0] - self.current_loc[0])) < self.final_goal_resolution and (abs(self.goal[1] - self.current_loc[1]) < self.final_goal_resolution)):
                return True
            else: 
                return False

        return False
    
    def get_slope(self, point1, point2):
        return math.degrees(math.atan((point1[1] - point2[1])/(point1[0] - point2[0])))

    def execute_goal(self):

        nearest = []
        while(self.final_goal_reached() == False):
            current_num_obstacles = len(self.obstacles)
            obstacles = self.obstacles
            if(current_num_obstacles):
                for i in range(0, current_num_obstacles, 2):

                    if(self.doGoalIntersect([obstacles[i], self.goal[:-1], obstacles[i+1], self.current_loc[:-1]]) == True):
                        nearest = self.nearestPoint(self.current_loc[:-1], obstacles[i][:-1], obstacles[i+1][:-1])  

                        if((abs(self.get_slope(self.obstacles[i], self.obstacles[i+1])) < 45) and (abs(nearest[0]-self.current_loc[0]) > 0.2)):
                            self.drone.cmd.pose.position.x = nearest[0]
                            self.current_goal = [self.drone.cmd.pose.position.x, self.current_loc[1]]

                        elif((abs(self.get_slope(self.obstacles[i], self.obstacles[i+1])) >= 45) and (abs(nearest[1]-self.current_loc[1]) > 0.2)):
                            self.drone.cmd.pose.position.y = nearest[1]
                            self.current_goal = [self.current_loc[0], self.drone.cmd.pose.position.y]

                        while(self.current_goal_reached() == False):
                            time.sleep(0.1)

                        time.sleep(0.2)

                        self.drone.cmd.pose.position.x = nearest[0]
                        self.drone.cmd.pose.position.y = nearest[1]

                        self.current_goal = [self.drone.cmd.pose.position.x, self.drone.cmd.pose.position.y]

                        while(not self.current_goal_reached()):
                            time.sleep(0.1)

                        self.obstacle_obstruct = True
                        time.sleep(0.1)
                        break

                    else:
                        self.obstacle_obstruct = False
                
                if(self.getDistance(self.current_loc, self.goal) > self.nearest_goal_min and self.obstacle_obstruct == False):

                    lenAB = math.sqrt(math.pow(self.current_loc[0] - self.goal[0], 2.0) + math.pow(self.current_loc[1] - self.goal[1], 2.0))

                    c_x = self.current_loc[0] - (self.current_loc[0] - self.goal[0]) / lenAB * 3.0
                    c_y = self.current_loc[1] - (self.current_loc[1] - self.goal[1]) / lenAB * 3.0

                    self.drone.cmd.pose.position.x = c_x
                    self.drone.cmd.pose.position.y = c_y

                    self.current_goal = [self.drone.cmd.pose.position.x, self.drone.cmd.pose.position.y]

                    while(not self.current_goal_reached()):
                        time.sleep(0.1)

                    self.roam = True

                elif (self.roam == True):
                    self.roam = False

                if(self.obstacle_obstruct == False and self.roam == False):
                    self.drone.cmd.pose.position.x = self.goal[0] 
                    self.drone.cmd.pose.position.y = self.goal[1]
                    
                    self.current_goal = self.goal[:-1]

                    while(not self.current_goal_reached()):
                        time.sleep(1)

        self.execute_goal_th_status = False
        rospy.loginfo("[PX4_PLANNER] Goal Completed :)")

    def nearestPoint(self, loc, point1, point2):
        distance1 = self.getDistance(loc, point1)
        distance2 = self.getDistance(loc, point2)

        if(point2[1] < 0):
            point2[1] = point2[1] - self.nearest_goal_offset
        else:
            point2[1] = point2[1] + self.nearest_goal_offset

        if(point1[1] < 0):
            point1[1] = point1[1] - self.nearest_goal_offset
        else:
            point1[1] = point1[1] + self.nearest_goal_offset

        if(distance1 > distance2):
            if(point2[1] < 0):
                point2[1] = point2[1] 
            
            return point2

        elif(distance1 < distance2):
            return point1

    def getDistance(self, point1, point2):
        
        return abs(math.sqrt(math.pow((point1[0] - point2[0]), 2) + math.pow((point1[1] - point2[1]), 2)))

    def goal(self, req):

        rospy.loginfo("[PX4_PLANNER] Goal Request Received - x : %f y: %f", req.x, req.y)
        
        self.obstacle_obstruct = False

        if(self.execute_goal_th_status == False):
            self.execute_goal_th = threading.Thread(target=self.execute_goal)

            if(len(self.goal)):
                del self.goal[:]
            self.goal = [-req.y, -req.x, req.z]

            self.execute_goal_th_status = True
            self.execute_goal_th.start()

            return True

        return False

    def execute_goal_queue(self):


        for coordinates in self.target_cordinate_queue:

            while(self.execute_goal_th_status):
                time.sleep(5)
            
            time.sleep(5)

            self.obstacle_obstruct = False

            self.execute_goal_th = threading.Thread(target=self.execute_goal)

            if(len(self.goal)):
                del self.goal[:]

            self.goal = [-coordinates.y, -coordinates.x, coordinates.z]

            self.execute_goal_th_status = True
            self.execute_goal_th.start()

    def goal_queue(self, req):

        rospy.loginfo("[PX4_PLANNER] Goal Queue Request Received ")
        # print("###Queue: {}".format(req))

        self.target_cordinate_queue = []
        
        for r in req.coordinates:
            print("###each Queue: {}".format(r))
            self.target_cordinate_queue.append(r)

        if(self.execute_goal_th_status == False):

            self.execute_goal_queue_th = threading.Thread(target=self.execute_goal_queue)
            self.execute_goal_queue_th.start()

            return True

        return False

    def detect_lines(self, edge_points, range_data):
        lines = []
        for e in edge_points:
            for e_c in edge_points:
                if(e_c != e):
                    chk = e
                    for it_e in range(e,e_c):
                        if(math.isinf(range_data.ranges[it_e])):
                            break
                        chk = it_e
                    if(chk == (e_c-1)):
                        lines.append({"a":e , "b":e_c})

        for line in range(len(lines)):
            for it_r in range(len(lines)):
                if(line != it_r):
                    if(lines[line]["a"] == 0 and lines[it_r]["b"] == 359):
                        f_r = lines[line]
                        t_o = lines[it_r]
                        lines.append({"a":t_o["a"], "b":f_r["b"]})

        for i in range(2):
            for line in range(len(lines)):
                if(lines[line]["a"] == 0 or lines[line]["b"] == 359):
                    del lines[line]
                    break
                
        return lines

    def pose_callback(self, data):
        
        self.current_loc = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    def laser_callback(self, data):
        
        edges = [0, 359]
        feat_points = []

        min_val = min(data.ranges)
        min_val_index = data.ranges.index(min_val)
        x = min_val*math.sin(math.radians(min_val_index))
        y = min_val*math.cos(math.radians(min_val_index)) 

        for r in range(len(data.ranges)-1):
            if(math.isinf(data.ranges[r])):
                if(math.isinf(data.ranges[r+1]) == False):
                    edges.append(r+1)
                elif(math.isinf(data.ranges[r-1]) == False):
                    edges.append(r-1)
        
        n_lines = self.detect_lines(edges, data)
        

        if(len(n_lines) != 0):
            self.obstacles = []
            for itr in n_lines:
                for key, values in itr.items():

                    y1_ = (data.ranges[values]*math.sin(math.radians(values))) - self.current_loc[1]
                    x1_ = (data.ranges[values]*math.cos(math.radians(values))) - self.current_loc[0]

                    self.obstacles.append([-x1_, -y1_, 0.0])

        for i in range(0, len(self.obstacles), 2):

            lenAB = math.sqrt(math.pow(self.obstacles[i][0] - self.obstacles[i+1][0], 2.0) + math.pow(self.obstacles[i][1] - self.obstacles[i+1][1], 2.0))

            c_x = self.obstacles[i+1][0] + (self.obstacles[i+1][0] - self.obstacles[i][0]) / lenAB * 1.0
            c_y = self.obstacles[i+1][1] + (self.obstacles[i+1][1] - self.obstacles[i][1]) / lenAB * 1.0

            d_x = self.obstacles[i][0] + (self.obstacles[i][0] - self.obstacles[i+1][0]) / lenAB * 1.0
            d_y = self.obstacles[i][1] + (self.obstacles[i][1] - self.obstacles[i+1][1]) / lenAB * 1.0

            self.obstacles[i][0] = c_x
            self.obstacles[i][1] = c_y

            self.obstacles[i+1][0] = d_x
            self.obstacles[i+1][1] = d_y

        for i in range(len(self.obstacles)):
            feat_points.append(Point(self.obstacles[i][0], self.obstacles[i][1], self.obstacles[i][2]))


    def onSegment(self, p, q, r):
        if ( (q.x <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and 
            (q.y <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
            return True
        return False

    def orientation(self, p, q, r):
        
        val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1]))

        if (val > 0):
            return 1
        elif (val < 0):
            return 2
        else:
            return 0

    def doGoalIntersect(self, points):
        
        o1 = self.orientation(points[0], points[2], points[1])
        o2 = self.orientation(points[0], points[2], points[3])
        o3 = self.orientation(points[1], points[3], points[0])
        o4 = self.orientation(points[1], points[3], points[2])

        if ((o1 != o2) and (o3 != o4)):
            return True

        if ((o1 == 0) and self.onSegment(points[0], points[1], points[2])):
            return True

        if ((o2 == 0) and self.onSegment(points[0], points[3], points[2])):
            return True

        if ((o3 == 0) and self.onSegment(points[1], points[0], points[3])):
            return True

        if ((o4 == 0) and self.onSegment(points[1], points[2], points[3])):
            return True

        return False


if __name__ == '__main__':
    rospy.init_node('px4_planner', anonymous=True)

    altitude = rospy.get_param("~altitude")

    DroneObj = DroneControl()
    plannerObj = Planner(DroneObj)
    
    DroneObj.cmd.pose.position.z = altitude
    DroneObj.arm(True)
    
    time.sleep(1)
    
    DroneObj.setMode()
    r = rospy.Rate(20) 
    
    rospy.loginfo("[PX4_PLANNER] Planner has been initialised")

    while not rospy.is_shutdown():
        DroneObj.goal_loc_pub.publish(DroneObj.cmd)
        r.sleep()

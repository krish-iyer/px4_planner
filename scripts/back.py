#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, PoseStamped
from aepl_planner.srv import target
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
        self.target_srv = rospy.Service('/aepl_planner/target', target, self.goal)
        self.obstacles = [[]]
        self.goal = []
        self.drone = drone
        self.execute_goal_th = threading.Thread()
        self.execute_goal_th_status = False
        self.current_loc = []
        self.current_goal = []
        self.obstacle_obstruct = False

    def current_goal_reached(self):

        if(len(self.current_goal)):
            if((abs(self.current_goal[0] - self.current_loc[0])) < 0.1 and (abs(self.current_goal[1] - self.current_loc[1]) < 0.1)):
                return True
            else: 
                return False

        return False


    def final_goal_reached(self):

        if(len(self.goal)):
            if((abs(self.goal[0] - self.current_loc[0])) < 0.1 and (abs(self.goal[1] - self.current_loc[1]) < 0.1)):
                return True
            else: 
                return False

        return False

    def execute_goal(self):
        # check if current location to goal intersect with any walls
        # if yes then overcome first obstacle and then check again iteractively
        # if no then go direct to goal
        # more number of dodging
        # update obstacle after every goal achieved
        
        # while True:
        if(len(self.goal)):
            # print(self.goal)
            # print("obstacles: {}".format(self.obstacles))
            print("number of obastacles : {}".format(len(self.obstacles)))
            nearest = []
            current_num_obstacles = len(self.obstacles)
            obstacles = self.obstacles
            if(current_num_obstacles):
                for i in range(0, current_num_obstacles, 2):
                    print("obstacles : {}".format(obstacles))
                    print("obst: {} obst1: {} goal1: {} goal2: {}".format(obstacles[i][:-1], obstacles[i+1][:-1], self.goal[:-1], self.current_loc[:-1]))

                    if(self.doIntersect([obstacles[i], self.goal[:-1], obstacles[i+1], self.current_loc[:-1]]) == True):
                        print("Goal Intersects")

                        nearest = self.nearestPoint(self.current_loc[:-1], obstacles[i][:-1], obstacles[i+1][:-1])
                        
                        if(nearest[0] >= 0):
                            self.drone.cmd.pose.position.x = nearest[0] + 1.5
                        elif(nearest[0] < 0):
                            self.drone.cmd.pose.position.x = nearest[0] - 1.5

                        if(nearest[1] >= 0):
                            self.drone.cmd.pose.position.y = nearest[1] + 1.5
                        elif(nearest[1] < 0):
                            self.drone.cmd.pose.position.y = nearest[1] - 1.5

                        self.current_goal = [self.drone.cmd.pose.position.x, self.drone.cmd.pose.position.y]

                        print("nearest point: {}".format(nearest))


                        while(not self.current_goal_reached()):
                            time.sleep(0.1)
                        
                        print("goal reached")

                        time.sleep(0.1)

                        # self.drone.cmd.pose.position.x = self.goal[0] 
                        # self.drone.cmd.pose.position.y = self.goal[1]
                        
                        # self.current_goal = self.goal[:-1]


                        # while(not self.goal_reached()):
                        #     time.sleep(0.1)

                        # self.obstacle_obstruct = True

                        # print("goal reached")

                if(self.obstacle_obstruct == False):
                    self.drone.cmd.pose.position.x = self.goal[0] 
                    self.drone.cmd.pose.position.y = self.goal[1]
                    
                    self.current_goal = self.goal[:-1]


                    while(not self.goal_reached()):
                        time.sleep(1)

                    print("goal reached")

            self.execute_goal_th_status = False
            print("goal completed")
            # features = Marker()
            # features.header.frame_id = "rplidar_link"
            # features.type = features.POINTS
            # features.action = features.ADD
            # features.pose.orientation.w = 1
            # if(len(nearest) == 2):
            #     features.points = [Point(self.goal[0], self.goal[1], self.goal[2]), Point(nearest[0], nearest[1], 0.0)]
            # else:
            #     features.points = [Point(self.goal[0], self.goal[1], self.goal[2])]
            # t = rospy.Duration(20)
            # features.lifetime = t
            # features.scale.x = 0.4
            # features.scale.y = 0.4
            # features.scale.z = 0.0
            # features.color.a = 0.7
            # features.color.r = 1.0
            # features.color.g = 1.0
            # features.color.b = 1.0
            # self.goal_marker.publish(features)
            
            # time.sleep(1)
            # while not self.goal_reached():
            #     time.sleep(1)

            # print("goal reached")
            
    def nearestPoint(self, loc, point1, point2):
        distance1 = self.getDistance(loc, point1)
        distance2 = self.getDistance(loc, point2)

        if(distance1 > distance2):
            return point2

        elif(distance1 < distance2):
            return point1

    def getDistance(self, point1, point2):
        
        return abs(math.sqrt(math.pow((point1[0] - point2[0]), 2) + math.pow((point1[1] - point2[1]), 2)))

    def goal(self, req):

        print("req: {} {}".format(req.x, req.y, req.z))

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

        # check for stiching
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
        features = Marker()
        features.header.frame_id = "rplidar_link"
        features.type = features.POINTS
        features.action = features.ADD
        features.pose.orientation = data.pose.orientation
        features.points = [data.pose.position]
        t = rospy.Duration(3)
        features.lifetime = t
        features.scale.x = 0.2
        features.scale.y = 0.2
        features.scale.z = 0.0
        features.color.a = 0.7
        features.color.r = 1.0
        features.color.g = 0.0
        features.color.b = 0.0

        self.robot_pose_marker.publish(features)

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
        
        # print("scan time: {} index: {} pos - x: {} y: {} lines: {}".format(min_val, min_val_index, x, y, n_lines))
        # print("lenght of lines: {}".format(len(n_lines)))
        if(len(n_lines) != 0):
            self.obstacles = []
            for itr in n_lines:
                for key, values in itr.items():
                    y1_ = (data.ranges[values]*math.sin(math.radians(values)))
                    x1_ = (data.ranges[values]*math.cos(math.radians(values))) 
                    if(y1_ < 0):
                        y1_ = y1_ + self.current_loc[1]
                        y1_ = y1_ - 1
                    else:
                        y1_ = y1_ - self.current_loc[1]
                        y1_ = y1_ + 1


                    if(x1_ < 0):
                        x1_ = x1_ + self.current_loc[0] 
                        x1_ = x1_ - 1
                    else:
                        x1_ = x1_ - self.current_loc[0] 
                        x1_ = x1_ + 1

                    # print("line cordinated: {}".format([-y1_, -x1_]))
                    self.obstacles.append([-x1_, -y1_, 0.0])
                    feat_points.append(Point(-x1_, -y1_, 0.0))    
        # print("###############################")
        # print("feature points:{}".format(feat_points))
        features = Marker()
        features.header.frame_id = "rplidar_link"
        features.type = features.POINTS
        features.action = features.ADD
        features.pose.orientation.w = 1
        features.points = feat_points
        t = rospy.Duration(3)
        features.lifetime = t
        features.scale.x = 0.2
        features.scale.y = 0.2
        features.scale.z = 0.0
        features.color.a = 0.7
        features.color.r = 0.0
        features.color.g = 1.0
        features.color.b = 0.0
        self.features_pub.publish(features)


    # Line intersections tools

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

    def doIntersect(self, points):
        

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
    rospy.init_node('aepl_planner', anonymous=True)
    DroneObj = DroneControl()
    plannerObj = Planner(DroneObj)
    # while(len(plannerObj.current_loc) == 0):
    #     time.sleep(0.1)
    # DroneObj.cmd.pose.position.x = plannerObj.current_loc[0]
    # DroneObj.cmd.pose.position.x = plannerObj.current_loc[1]
    DroneObj.cmd.pose.position.z = 1.0
    DroneObj.arm(True)
    time.sleep(1)
    DroneObj.setMode()
    r = rospy.Rate(20) # 10hz 
    while not rospy.is_shutdown():
        DroneObj.goal_loc_pub.publish(DroneObj.cmd)

        r.sleep()

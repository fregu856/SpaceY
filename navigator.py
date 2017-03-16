#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
import matplotlib.pyplot as plt
import tf
from std_msgs.msg import Float32MultiArray, String
from astar import AStar, StochOccupancyGrid2D, StochOccupancyGrid2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Navigator:

    def __init__(self):
        rospy.init_node('navigator', anonymous=True)

        self.plan_resolution = 0.25
        self.plan_horizon = 15

        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None

        self.nav_sp = None
        self.nav_path = None

        self.trans_listener = tf.TransformListener()

        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("map_metadata", MapMetaData, self.map_md_callback)
        rospy.Subscriber("/turtlebot_control/nav_goal", Float32MultiArray, self.nav_sp_callback)

        self.pose_sp_pub = rospy.Publisher('/turtlebot_control/position_goal', Float32MultiArray, queue_size=10)
        # create publisher to publish goal paths:
        self.nav_path_pub = rospy.Publisher('/turtlebot_control/path_goal', Path, queue_size=10)
        # create publisher to publish debug info:
        self.debug_pub = rospy.Publisher("/turtlebot_control/navigator_debug",
                                       String, queue_size=10)

    def map_md_callback(self,msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        self.map_probs = msg.data
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  int(self.plan_resolution / self.map_resolution) * 2,
                                                  self.map_probs)

    def nav_sp_callback(self,msg):
        self.nav_sp = (msg.data[0], msg.data[1], msg.data[2])
        self.send_pose_sp()

    def send_pose_sp(self):
        try:
            (robot_translation,robot_rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            self.has_robot_location = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            robot_translation = (0,0,0)
            robot_rotation = (0,0,0,1)
            self.has_robot_location = False

        if self.occupancy and self.has_robot_location and self.nav_sp:
            state_min = (-int(round(self.plan_horizon)), -int(round(self.plan_horizon)))
            state_max = (int(round(self.plan_horizon)), int(round(self.plan_horizon)))
            x_init = (int(round(robot_translation[0])), int(round(robot_translation[1])))
            x_goal = (int(round(self.nav_sp[0])), int(round(self.nav_sp[1])))
            astar = AStar(state_min,state_max,x_init,x_goal,self.occupancy,self.plan_resolution)

            rospy.loginfo("Computing navigation plan")
            if astar.solve():
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                for state in astar.path:
                    pose_st = PoseStamped()
                    pose_st.pose.position.x = state[0]
                    pose_st.pose.position.y = state[1]
                    pose_st.pose.orientation.w = self.nav_sp[2] # (theta)
                    pose_st.header.frame_id = 'map'
                    path_msg.poses.append(pose_st)
                self.nav_path_pub.publish(path_msg)
                self.nav_path = path_msg.poses

            else:
                rospy.logwarn("Could not find path")

    def check_if_free_path(self):
        current_nav_path = self.nav_path
        for index in range(len(current_nav_path) - 1):
            if index > 0: # (don't check the first or last point, doesn't work)
                pose_obj = current_nav_path[index]
                xy_state = [pose_obj.pose.position.x, pose_obj.pose.position.y]
                if not self.occupancy.is_free(xy_state):
                    # replan and send a new path:
                    self.send_pose_sp()
                    # publish debug info:
                    debug_msg = String()
                    debug_msg.data = "Current path not free, new path sent!"
                    self.debug_pub.publish(debug_msg)
                    break
                else:
                    # just publish debug info:
                    debug_msg = String()
                    debug_msg.data = "Current path OK"
                    self.debug_pub.publish(debug_msg)

    def run(self):
        #rospy.spin()
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.nav_path is not None:
                # check if the current planned path is free, replan if not:
                self.check_if_free_path()
            rate.sleep()

if __name__ == '__main__':
    nav = Navigator()
    nav.run()

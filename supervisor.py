#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np

def pose_to_xyth(pose):
    th = tf.transformations.euler_from_quaternion((pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w))[2]
    return [pose.position.x, pose.position.y, th]


class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        # create publisher to publish goal pose:
        self.pose_pub = rospy.Publisher("/turtlebot_control/nav_goal",
                                       Float32MultiArray, queue_size=10)
        # create publisher to publish rviz goal pose:
        self.rviz_pose_pub = rospy.Publisher("/turtlebot_control/nav_goal",
                                       Float32MultiArray, queue_size=10)
        # create publisher to publish the current state:
        self.state_pub = rospy.Publisher("/turtlebot_control/state", String,
                                         queue_size=10)
        # create publisher to publish debug info:
        self.debug_pub = rospy.Publisher("/turtlebot_control/supervisor_debug",
                                       Float32MultiArray, queue_size=10)

        # subscribe to the topic with the current goal pose sent manually from RVIZ:
        rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                    self.rviz_goal_callback)
        # subscribe to the topic with the mission specification:
        rospy.Subscriber("/mission", Int32MultiArray, self.mission_callback)
        # subscribe to the topic with the current robot status:
        rospy.Subscriber("/turtlebot_control/robot_status", String,
                    self.status_callback)

        self.state = "MANUAL" # (current state in the state machine)

        self.mission = None
        self.arrived = False
        self.visited_tags = []
        self.current_tag = []

        self.trans_listener = tf.TransformListener()
        self.trans_broad = tf.TransformBroadcaster()

        self.waypoint_locations = {}    # dictionary that caches the most updated locations of each mission waypoint
        self.waypoint_offset = PoseStamped()
        self.waypoint_offset.pose.position.z = .4    # waypoint is located 40cm in front of the AprilTag, facing it
        quat = tf.transformations.quaternion_from_euler(0., np.pi/2, np.pi/2)
        self.waypoint_offset.pose.orientation.x = quat[0]
        self.waypoint_offset.pose.orientation.y = quat[1]
        self.waypoint_offset.pose.orientation.z = quat[2]
        self.waypoint_offset.pose.orientation.w = quat[3]

    # callback function for the pose goal sent manually from RVIZ:
    def rviz_goal_callback(self, msg):
        if self.state == "MANUAL":
            self.rviz_goal_pose = pose_to_xyth(msg.pose)
            # publish the rviz goal pose:
            rviz_pose_msg = Float32MultiArray()
            rviz_pose_msg.data = self.rviz_goal_pose
            self.rviz_pose_pub.publish(rviz_pose_msg)

    def status_callback(self, msg):
        robot_status = msg.data
        if robot_status == "MOVING":
            self.arrived = False
        elif robot_status == "STATIONARY":
            self.arrived = True

    # callback function for the mission specification:
    def mission_callback(self, msg):
        self.mission = msg.data
        self.unique_tags = list(set(self.mission))
        self.final_tag_index = len(self.mission) - 1

    def update_waypoints(self):
        for tag_number in self.mission:
            try:
                self.waypoint_offset.header.frame_id = "/tag_{0}".format(tag_number)
                self.waypoint_locations[tag_number] = self.trans_listener.transformPose("/map", self.waypoint_offset)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    # function for updating the state in the state machine:
    def update_state(self):
        if self.state == "MANUAL":
            if len(self.waypoint_locations) == len(self.unique_tags):
                self.next_tag_index = 0
                self.state = "AUTO/SENDING"

        # #
        elif self.state == "AUTO/SENDING":
            self.state = "AUTO/MOVING"
            self.counter = 0

        # #
        elif self.state == "AUTO/MOVING":
            if self.arrived and self.counter > 20: # (at least 2 sec between tags, to stop us from skipping tags)
                self.visited_tags.append(self.mission[self.next_tag_index])

                if self.next_tag_index < self.final_tag_index:
                    self.next_tag_index += 1
                    self.state = "AUTO/SENDING"
                else:
                    self.state = "FINISHED"

        elif self.state == "FINISHED":
            pass


    def execute_state(self):
        if self.state == "MANUAL":
            pass

        # #
        elif self.state == "AUTO/SENDING":
            next_tag = self.mission[self.next_tag_index]
            self.current_tag = [next_tag]
            next_pose = pose_to_xyth(self.waypoint_locations[next_tag].pose)
            pose_msg = Float32MultiArray()
            pose_msg.data = next_pose
            self.pose_pub.publish(pose_msg)

        elif self.state == "AUTO/MOVING":
            self.counter +=1

        elif self.state == "FINISHED":
            pass

    # function for actually publishing all information:
    def publish(self):
        # publish debug info:
        # # the all tags we need to find in exploration:
        debug_msg = Float32MultiArray()
        debug_msg.data = self.unique_tags
        self.debug_pub.publish(debug_msg)

        # # all tags we have found in exploration:
        debug_msg = Float32MultiArray()
        debug_msg.data = [tag_no for tag_no in self.waypoint_locations]
        self.debug_pub.publish(debug_msg)

        # # mission specification:
        debug_msg = Float32MultiArray()
        debug_msg.data = self.mission
        self.debug_pub.publish(debug_msg)

        # # the tags we have visited in AUTO:
        debug_msg = Float32MultiArray()
        debug_msg.data = self.visited_tags
        self.debug_pub.publish(debug_msg)

        # # the tag we're currently moving to in AUTO:
        debug_msg = Float32MultiArray()
        debug_msg.data = self.current_tag
        self.debug_pub.publish(debug_msg)

        # publish the current state:
        state_msg = self.state
        self.state_pub.publish(state_msg)

    def run(self):
        rate = rospy.Rate(10) # (10 Hz)
        while not rospy.is_shutdown():
            if self.mission is not None:
                self.update_waypoints()
                self.update_state()
                self.execute_state()
                self.publish()
            rate.sleep() # (to get loop freq. of 10 Hz)

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()

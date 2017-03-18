#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from nav_msgs.msg import Path
import tf
import numpy as np

class Controller:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)

        # subscribe to the topic with goal paths:
        rospy.Subscriber("/turtlebot_control/path_goal", Path,
                         self.path_goal_callback)

        # create publisher to publish control commands to the turtlebot:
        self.pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        # create publisher to publish debug info:
        self.debug_pub = rospy.Publisher("/turtlebot_control/controller_debug",
                                       Float32MultiArray, queue_size=10)
        # create publisher to publish current robot status to supervisor:
        self.status_pub = rospy.Publisher("/turtlebot_control/robot_status",
                    String, queue_size=10)

        self.goal_path = None
        self.goal_pose = None

        # TransformListener to get the estimated robot position:
        self.trans_listener = tf.TransformListener()

    # callback function for reading goal paths:
    def path_goal_callback(self, msg):
        if len(msg.poses) > 1: # (skip paths that are just one point)
            self.goal_path = msg.poses # (list of poses)
            self.path_length = len(self.goal_path)
            # start with the second point of the path as the goal pose:
            self.next_pose_index = 1

    def wrapToPi(self, a):
        if isinstance(a, list):    # backwards compatibility for lists (distinct from np.array)
            return [(x + np.pi) % (2*np.pi) - np.pi for x in a]
        return (a + np.pi) % (2*np.pi) - np.pi

    def get_ctrl_output(self):
        # move towards the goal position using the controller:
        cmd_x_dot = 0.0 # forward velocity (= V)
        cmd_theta_dot = 0.0

        x_g = self.goal_pose[0]
        y_g = self.goal_pose[1]
        th_g = self.goal_pose[2]

        k1 = 0.4
        k2 = 0.4
        k3 = 0.4

        rho = np.sqrt((self.x-x_g)**2 + (self.y-y_g)**2)
        alpha = np.arctan2(self.y-y_g, self.x-x_g) - self.theta + np.pi
        alpha = self.wrapToPi(alpha)
        delta = np.arctan2(self.y-y_g, self.x-x_g) + np.pi - th_g
        delta = self.wrapToPi(delta)

        # define control inputs (V,om) - without saturation constraints
        V = k1*rho*np.cos(alpha)
        om = k2*alpha+k1*np.sinc(alpha/np.pi)*np.cos(alpha)*(alpha+k3*delta)

        # apply saturation limits
        V = np.sign(V)*min(0.5, np.abs(V))
        om = np.sign(om)*min(1, np.abs(om))

        cmd_x_dot = V
        cmd_theta_dot = om

        cmd = Twist()
        cmd.linear.x = cmd_x_dot
        cmd.angular.z = cmd_theta_dot

        return cmd

    def close_enough_xy(self):
        if np.linalg.norm(np.array(self.goal_pose[0:2]) - np.array(self.robot_pose[0:2])) <= 0.2:
            return True
        else:
            return False

    def close_enough_theta(self):
        if np.abs(self.wrapToPi(self.robot_pose[2] - self.goal_pose[2])) <= 1.0:
            return True
        else:
            return False

    def update_robot_pose(self):
        # get the current estimated robot pose:
        try:
            (position, quaternion) = self.trans_listener.lookupTransform(
                        "/map", "/base_footprint", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            position = (0,0,0) # (x, y, z)
            quaternion = (0,0,0,1) # (x, y, z, w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.x = position[0]
        self.y = position[1]
        self.theta = euler[2]
        self.robot_pose = [self.x, self.y, self.theta]

    def update_goal_pose(self):
        goal_path = self.goal_path

        if self.next_pose_index == 1:
            # set the next goal pose to be the second point on the path:
            next_pose = goal_path[self.next_pose_index].pose
            x_goal = next_pose.position.x
            y_goal = next_pose.position.y
            th_goal = next_pose.orientation.w
            self.goal_pose = [x_goal, y_goal, th_goal]
            self.next_pose_index += 1

        elif self.next_pose_index < self.path_length:
            if self.close_enough_xy():
                # set the next next goal pose to be the next point on the path:
                next_pose = goal_path[self.next_pose_index].pose
                x_goal = next_pose.position.x
                y_goal = next_pose.position.y
                th_goal = next_pose.orientation.w
                self.goal_pose = [x_goal, y_goal, th_goal]
                self.next_pose_index += 1

        elif self.next_pose_index == self.path_length:
            # wait until we're close enough to the final point on the path:
            if self.close_enough_xy() and self.close_enough_theta():
                self.next_pose_index += 1

        else:
            # we have moved through each point on the path, wait for new goal:
            self.goal_pose = None

    def run(self):
        rate = rospy.Rate(10) # (10 Hz)
        while not rospy.is_shutdown():
            if self.goal_pose is not None:
                # publish the current goal pose for debug purposes:
                debug_msg = Float32MultiArray()
                debug_msg.data = self.goal_pose
                self.debug_pub.publish(debug_msg)

                # tell the supervisor that the robot is moving to a goal pose:
                status_msg = String()
                status_msg.data = "MOVING"
                self.status_pub.publish(status_msg)
            else:
                # publish a dummy goal pose indicating that the robot is currently
                # not moving, for debug purposes:
                debug_msg = Float32MultiArray()
                debug_msg.data = [-1000.0, -1000.0, -1000.0]
                self.debug_pub.publish(debug_msg)

                # tell the supervisor that th robot is stationary (it has reached
                # a goal pose):
                status_msg = String()
                status_msg.data = "STATIONARY"
                self.status_pub.publish(status_msg)

            # get the latest estimated robot pose:
            self.update_robot_pose()

            if self.goal_path is not None:
                # get the pose we currently are supposed to move towards along
                # the goal path:
                self.update_goal_pose()
                if self.goal_pose is not None:
                    # get and publish the control command for moving toward the
                    # current goal pose:
                    ctrl_output = self.get_ctrl_output()
                    self.pub.publish(ctrl_output)
            rate.sleep() # (to get loop freq. of 10 Hz)

if __name__ == '__main__':
    ctrl = Controller()
    ctrl.run()

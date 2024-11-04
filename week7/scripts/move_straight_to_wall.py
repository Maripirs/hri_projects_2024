#!/usr/bin/python3

import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# this is based on the Robotics Back-End: https://roboticsbackend.com/oop-with-ros-in-python/

class MoveStraightOdom:
    def __init__(self):
        self.odom = Odometry()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.laser = rospy.Subscriber("/base_scan", LaserScan, self.laser_calback)
        rospy.sleep( rospy.Duration.from_sec(0.5) )
        self.closer_scan = 10

    def odom_callback(self, msg):
        self.odom = msg

    def laser_calback(self, data):
        min = 100
        for range in data.ranges:
            if range < min:
                min = range
        self.closer_scan = min
    
    def get_odom(self):
        return self.odom
    
    def get_laser(self):
        return self.laser

if __name__ == '__main__':
    rospy.init_node('move_straight')
    n = MoveStraightOdom()
    rate = rospy.Rate(15.0)


    # figure out where we started from
    start = n.get_odom()

    # start the robot's movement
    t = Twist()
    t.linear.x = 1.0
    n.pub.publish(t)
    closer_wall = 100
    while not rospy.is_shutdown():

        # maintain current rate
        n.pub.publish(t)
        n.get_laser()
        closer_wall = n.closer_scan
        
        if closer_wall < .5:
            t.linear.x = 0.0
            n.pub.publish(t)
            break

        rate.sleep()

    t.linear.x = 0.0
    n.pub.publish(t)
    rospy.sleep( rospy.Duration.from_sec(1.0) )

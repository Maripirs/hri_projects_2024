#!/usr/bin/python3

import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# this is based on the Robotics Back-End: https://roboticsbackend.com/oop-with-ros-in-python/

class MoveStraightOdom:
    def __init__(self):
        self.odom = Odometry()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.laser = rospy.Subscriber("/base_scan", LaserScan, self.laser_calback)
        rospy.sleep( rospy.Duration.from_sec(0.5) )
        self.clear_direction = 1

        self.closer_wall = 10
        self.clear_choice = 'front'

    def odom_callback(self, msg):
        self.odom = msg
    def get_yaw (self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw
    def laser_calback(self, data):

        right = min(data.ranges[:360])
    
        front = min(data.ranges[361:720])
        left = min(data.ranges[721:])

        if right > 5:

            self.clear_direction = -1
            self.clear_choice = 'right'

        elif left > 5:

            self.clear_direction = 1
            self.clear_choice = 'left'
        #There's 1081 scans
        
        self.closer_wall = min(left, front,  right)


    
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
    t.linear.x = 5.0
    n.pub.publish(t)
    start_twist = n.get_yaw(n.get_odom())
    prev_twist = start_twist
    moving = True
    twisting = False
    closer_wall = 100
    sum_turn = 0
    while not rospy.is_shutdown():
        # maintain current rate
        n.pub.publish(t)
        if moving:
            
            n.get_laser()
            closer_wall = n.closer_wall
            if closer_wall < 1:
                print(f'Changing directions to : {n.clear_choice}')
                t.linear.x = 0.0
                n.pub.publish(t)
                moving = False
                t.angular.z = 0.5 * n.clear_direction
                n.pub.publish(t)
                twisting = True
                
            
        if twisting:
            cur_twist = n.get_yaw(n.get_odom())
            diff = math.fabs(cur_twist - prev_twist)
            prev_twist = cur_twist
            if diff > math.pi:
                diff -= 2 * math.pi
            sum_turn += diff
            if sum_turn > math.radians(90):
                t.angular.z = 0.0
                t.linear.x = 5.0
                start_twist = cur_twist
                sum_turn = 0
                n.pub.publish(t)
                twisting = False
                moving = True
        rate.sleep()

    t.linear.x = 0.0
    n.pub.publish(t)
    rospy.sleep( rospy.Duration.from_sec(1.0) )

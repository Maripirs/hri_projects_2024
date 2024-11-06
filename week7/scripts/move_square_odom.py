#!/usr/bin/python3

import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# this is based on the Robotics Back-End: https://roboticsbackend.com/oop-with-ros-in-python/

class MoveStraightOdom:
    def __init__(self):
        self.odom = Odometry()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.sleep( rospy.Duration.from_sec(0.5) )

    def odom_callback(self, msg):
        self.odom = msg

    def get_yaw (self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw
    def get_odom(self):
        return self.odom

if __name__ == '__main__':
    rospy.init_node('move_straight')
    n = MoveStraightOdom()
    rate = rospy.Rate(15.0)


    # figure out where we started from
    start_move = n.get_odom()

    # start the robot's movement
    t = Twist()
    start_twist = n.get_yaw(n.get_odom())
    prev_twist = start_twist
    moving = True
    twisting = False
    for i in range (4):
        sum_turn = 0
        n.pub.publish(t)
        t.linear.x = 1.0
        print(f'moving {i}')
        while not rospy.is_shutdown():

            # maintain current rate
            n.pub.publish(t)

            # get current odom
            cur_move = n.get_odom()
            if moving:
                print("walking")
            # is distance greater than 1m?
                dx = cur_move.pose.pose            diff = math.fabs(cur_twist - prev_twist)
            prev_twist = cur_twist
            if diff > math.pi:
                diff -= 2 * math.pi
            sum_turn += diff
            print(sum_turn)
            if sum_turn > math.radians(180):
                t.angular.z = 0.0
                t.linear.x = 1.0
                start_move = cur_move
                start_twist = cur_twist
                n.pub.publish(t)
                twisting = False
                moving = True.position.x - start_move.pose.pose.position.x
                dy = cur_move.pose.pose.position.y - start_move.pose.pose.position.y

            # distance
                dist = math.sqrt( dx*dx + dy*dy )
                print(dist)

                if dist > 1.0:
                    t.linear.x = 0.0
                    t.angular.z = 0.5
                    n.pub.publish(t)
                    moving = False
                    twisting = True
            if twisting:
                print("twisting")
                cur_twist = n.get_yaw(n.get_odom())
                diff = math.fabs(cur_twist - prev_twist)
                prev_twist = cur_twist
                if diff > math.pi:
                    diff -= 2 * math.pi
                sum_turn += diff
                print(sum_turn)
                if sum_turn > math.radians(90):
                    t.angular.z = 0.0
                    t.linear.x = 1.0
                    start_move = cur_move
                    start_twist = cur_twist
                    n.pub.publish(t)
                    twisting = False
                    moving = True
                    break
                

            rate.sleep()

        t.linear.x = 0.0
        n.pub.publish(t)
        rospy.sleep( rospy.Duration.from_sec(1.0) )

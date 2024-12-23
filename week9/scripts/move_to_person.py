#!/usr/bin/python3

import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from people_msgs.msg import PositionMeasurementArray

# this is based on the Robotics Back-End: https://roboticsbackend.com/oop-with-ros-in-python/

class MoveStraightOdom:
    def __init__(self):
        self.odom = Odometry()

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.laser = rospy.Subscriber("/base_scan", LaserScan, self.laser_calback)
        self.person = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, self.person_callback)
        rospy.sleep( rospy.Duration.from_sec(0.5) )
        self.clear_direction = 1

        self.closer_wall = 10
        self.clear_choice = 'front'
        self.person_angle = 0
        self.person_distance = 100
        self.count = 1
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

    def odom_callback(self, msg):
        self.odom = msg
    def get_yaw (self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw
    def laser_calback(self, data):

        right = min(data.ranges[:400])
    
        front = min(data.ranges[401:600])
        left = min(data.ranges[601:])

        if right > 5:

            self.clear_direction = -1
            self.clear_choice = 'right'

        elif left > 5:

            self.clear_direction = 1
            self.clear_choice = 'left'
        #There's 1081 scans
        
        self.closer_wall = min(left, front,  right)
    def person_callback(self, data):
        if data.people:
            
            trans = self.tfBuffer.lookup_transform('base_link', data.people[0].name, rospy.Time())

            # print(f'Person from base_link {trans.transform.translation.x}, {trans.transform.translation.y}')


            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            self.person_distance = math.sqrt( dx*dx + dy*dy )
            # print(f'Pioneer coordinates are {self.odom.pose.pose.position.x}, {self.odom.pose.pose.position.y}')
            # print(f'Person is x:{dx} y: {dy} . {person_distance}m distance')
            self.person_angle = math.atan2(dy, dx) 
            if self.count == 20:

                self.count = 0
            else:
                self.count += 1
            
            
        


    
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
    t.linear.x = .5 
    n.pub.publish(t)
    start_move = n.get_odom()
    start_twist = n.get_yaw(n.get_odom())
    prev_twist = start_twist
    moving = True
    twisting = False
    closer_wall = 100
    sum_turn = 0
    obstacle = 0
    dist = 0
    person_found = False
    while not rospy.is_shutdown():
        # maintain current rate
        n.pub.publish(t)
        cur_move = n.get_odom()
        
        if not person_found:
            print("No person")
            if abs(n.person_angle) > 0:

                person_found = True
                target_angle = abs(n.person_angle)
                direction =  1 if n.person_angle > 0 else -1 
                print(f"Identified a person, starting Twist to {target_angle}")
                moving = False
                t.linear.x = 0
                t.angular.z = .2 * direction
                twisting = True
        if moving:
            dx = cur_move.pose.pose.position.x - start_move.pose.pose.position.x
            dy = cur_move.pose.pose.position.y - start_move.pose.pose.position.y
            dist = math.sqrt( dx*dx + dy*dy )
            n.get_laser()
            closer_wall = n.closer_wall
            if dist > 2:
                print("Re checking for person")
                
                person_found = False

            if closer_wall < 1 :

                print("We're about to hit something")
                t.linear.x = 0.0
                n.pub.publish(t)
                moving = False
                if n.person_distance > 2:
                    print(f'Not a person. Turning {n.clear_choice} to avoid obstacle')
                    t.angular.z = 0.2 * n.clear_direction
                    n.pub.publish(t)
                    obstacle = 1
                    twisting = True
                else:
                    print('Found The Person!')
                    t.linear.x = 0
                    t.angular.z = 0
                    moving = False
                    twisting = False
                    person_found = True
            elif obstacle and dist > 1:
                t.linear.x = 0.0
                n.pub.publish(t)
                moving = False
                print(f'Changing directions to : {n.clear_choice}')
                t.angular.z = 0.2 * n.clear_direction
                n.pub.publish(t)
                obstacle = 0
                person_found = False
                twisting = True
                
                
                
            
        if twisting:

            if obstacle:
                target_angle = math.radians(50)
            cur_twist = n.get_yaw(n.get_odom())
            diff = math.fabs(cur_twist - prev_twist)
            prev_twist = cur_twist
            if diff > math.pi:
                diff -= 2 * math.pi
            sum_turn += diff
            # print(f'target turn = {target_angle}, sum = {sum_turn}')
            if sum_turn > target_angle:
                print("Completed turn")
                t.angular.z = 0.0
                t.linear.x = 0.5
                start_twist = cur_twist
                start_move = n.get_odom()
                sum_turn = 0
                n.pub.publish(t)
                twisting = False
                moving = True
        rate.sleep()

    t.linear.x = 0.0
    n.pub.publish(t)
    rospy.sleep( rospy.Duration.from_sec(1.0) )

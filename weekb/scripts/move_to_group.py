#!/usr/bin/python3

import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from weekb.msg import GroupInfo

# this is based on the Robotics Back-End: https://roboticsbackend.com/oop-with-ros-in-python/

class MoveStraightOdom:
    def __init__(self):
        self.odom = Odometry()

        self.pub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/robot_0/odom", Odometry, self.odom_callback)
        self.laser = rospy.Subscriber("/robot_0/base_scan", LaserScan, self.laser_calback)
        self.group = rospy.Subscriber("/group_info", GroupInfo, self.group_callback)
        rospy.sleep( rospy.Duration.from_sec(0.5) )
        self.clear_direction = 1

        self.closer_wall = 10
        self.clear_choice = 'front'
        self.target_angle = 0
        self.group_distance = 100
        self.count = 1
        self.tfBuffer = tf2_ros.Buffer()
        self.group_type = "NONE"
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
    def group_callback(self, data):
        if self.count > 20:
        	self.count = 0
        elif self.count > 0:
        	self.count += 1
		
	
        if self.count == 0 and (data.group_type == 'Line' or data.group_type == 'Circle'):
            self.count = 1
            self.group_type = data.group_type
            trans = self.tfBuffer.lookup_transform('robot_0/base_link', 'group', rospy.Time())

            # print(f'Person from base_link {trans.transform.translation.x}, {trans.transform.translation.y}')


            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            self.group_distance = math.sqrt( dx*dx + dy*dy )
            if self.group_type == 'Circle':
            	print(f'circle location {data.x} {data.y} - distance {self.group_distance} radius {data.r}')
            	self.group_distance -= data.r
            # print(f'Pioneer coordinates are {self.odom.pose.pose.position.x}, {self.odom.pose.pose.position.y}')
            # print(f'Person is x:{dx} y: {dy} . {person_distance}m distance')
            self.target_angle = math.atan2(dy, dx) 
            
            
            
        


    
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
    t.linear.x = 0
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
    target_found = False
    while not rospy.is_shutdown():
        # maintain current rate
        n.pub.publish(t)
        cur_move = n.get_odom()
        
        if not target_found:
            print("No target")
            if abs(n.target_angle) > 0:

                target_found = True
                target_angle = abs(n.target_angle)
                direction =  1 if n.target_angle > 0 else -1 
                print(f"Identified a target, starting Twist to {target_angle}")
                moving = False
                t.linear.x = 0
                t.angular.z = .2 * direction
                twisting = True
                n.pub.publish(t)
              
        if moving:
            dx = cur_move.pose.pose.position.x - start_move.pose.pose.position.x
            dy = cur_move.pose.pose.position.y - start_move.pose.pose.position.y
            dist = math.sqrt( dx*dx + dy*dy )
            n.get_laser()
            closer_wall = n.closer_wall
            if n.group_distance < 0.5:
            	print("We're here")
            	t.linear.x = 0.0
            	n.pub.publish(t)
            	moving = False
            if closer_wall < 1 :

                print("We're about to hit something")
                t.linear.x = 0.0
                n.pub.publish(t)
                moving = False
                if n.group_distance > 1:
                    print(f'Not near target. Turning {n.clear_choice} to avoid obstacle')
                    t.angular.z = 0.2 * n.clear_direction
                    n.pub.publish(t)
                    obstacle = 1
                    twisting = True
                else:
                    print('Found The Group!')
                    t.linear.x = 0
                    t.angular.z = 0
                    moving = False
                    twisting = False
                    target_found = True
            elif obstacle and dist > 1:
                t.linear.x = 0.0
                n.pub.publish(t)
                moving = False
                print(f'Changing directions to : {n.clear_choice}')
                t.angular.z = 0.2 * n.clear_direction
                n.pub.publish(t)
                obstacle = 0
                target_found = False
                twisting = True
                
                
                
            
        if twisting:
            print("twisting")
            if obstacle:
                target_angle = math.radians(50)
            cur_twist = n.get_yaw(n.get_odom())
            diff = math.fabs(cur_twist - prev_twist)
            prev_twist = cur_twist
            if diff > math.pi:
                diff -= 2 * math.pi
            sum_turn += diff
            print(f'target turn = {target_angle}, sum = {sum_turn}')
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

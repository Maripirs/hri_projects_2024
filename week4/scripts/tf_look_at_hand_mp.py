#!/usr/bin/python3
import rospy

import math
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
# this is based on the ROS tf2 tutorial: http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29




	#rosmsg show to see the structure of the data
	#


"""

	x = trans.transform.translation.x
	y = trans.transform.translation.y
	z = trans.transform.translation.z
	
	headYawPosition = atan2(y,x)  
	headPitchPosition = atan2(x,z)  # or math.radians(atan2(x,z))

	#Check how to grab the joint Tuple
	newJS = jointState()
	oldJS = joint_state_original  # coming from the subscriber
	tempArr = list(oldJS.position)
	tempArr[0] = headYawPosition
	tempArr[1] = headPitchPosition
	tempTuple = tuple(tempArr)
	newJS.position = tempTuple
	
	pub.publish(newJS)
	
"""


class JointStateObject:
	def __init__(self):
		self.pub = rospy.Publisher("joint_states", JointState, queue_size = 10)
		self.js_subscriber = rospy.Subscriber("joint_states_original", JointState, self.callback_js)
		self.msg = JointState()
	def callback_js(self, msg):
		newJS = JointState(msg)
		positionArr = list(newJS.position) #Do copy constructors work like this?
		headYawPosition = positionArr[0] + 0.1
		headPitchPosition = positionArr[1] +0.1
		positionArr[0] = headYawPosition
		positionArr[1] = headPitchPosition
		positionTuple = tuple(positionArr)
		newJS.position = positionTuple # can we reasign the tuple like this
		self.msg = newJS
	def get_msg(self):
		return self.msg
		
		

#
if __name__ == '__main__':
    rospy.init_node('tf2_look_at_hand')
    

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
   
    rate = rospy.Rate(10.0)
    js_global = JointStateObject()
    while not rospy.is_shutdown():
    
    	js = js_global
        try:
            trans = tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        print("trans: x: %f y: %f z: %f", trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
        rate.sleep()
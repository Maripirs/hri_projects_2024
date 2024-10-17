#!/usr/bin/python3

import rospy
import tf

if __name__ == '__main__':
	rospy.init_node('hand_pointing')
	hp = tf.TransformBroadcaster()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		hp.sendTransform((1.0,0.0,0.0), (0.0,0.0,0.0,1.0), rospy.Time.now(), "pointing", 'LForeArm')
		rate.sleep()

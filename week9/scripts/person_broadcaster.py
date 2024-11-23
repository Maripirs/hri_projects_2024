#!/usr/bin/python3

import rospy
import tf
from people_msgs.msg import PositionMeasurementArray
def callback(data):

    print(data)


    hp = tf.TransformBroadcaster()
    for person in data.people:
        hp.sendTransform((person.pos.x,person.pos.y,person.pos.z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),person.name,'odom')

if __name__ == '__main__':
    print("Running")
    rospy.init_node('person_broadcaster')
    sub = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, callback)
    hp = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        rate.sleep()

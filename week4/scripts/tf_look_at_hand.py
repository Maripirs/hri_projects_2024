#!/usr/bin/python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState

# this is based on the ROS tf2 tutorial: http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29

yawx = 0
yawy = 0
yawz = 0
pitchx = 0
pitchy = 0
pitchz = 0
trans = 0

class JointStateObject:
    def __init__(self):
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.js_subscriber = rospy.Subscriber("joint_states_original", JointState, self.callback_js)
        self.msg = JointState()

    def callback_js(self, msg):
        newJS = JointState()
        newJS.header = msg.header
        newJS.name = msg.name
        positionArr = list(msg.position)
        print("updating head")
        headYawPosition = math.atan2(yawy, yawx)  
        headPitchPosition = math.atan2(pitchy,pitchx)
        positionArr[0] = headYawPosition
        positionArr[1] = headPitchPosition
        
        positionTuple = tuple(positionArr)
        newJS.position = positionTuple  
       
        self.msg = newJS
        self.pub.publish(newJS)

    def get_msg(self):
        return self.msg

if __name__ == '__main__':
    rospy.init_node('tf2_look_at_hand')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10)
    js_global = JointStateObject()
    
    while not rospy.is_shutdown():
        print("hello")
        js = js_global
        try:
            trans = tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
            print("But we got it over here")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        print("trans: x: %f y: %f z: %f" % (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z))
        yawx = trans.transform.translation.x
        yawy = trans.transform.translation.y
        yawz = trans.transform.translation.z

        try:
            trans2 = tfBuffer.lookup_transform('torso', 'l_gripper', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        pitchx = trans2.transform.translation.x
        pitchy = trans2.transform.translation.y
        pitchz = trans2.transform.translation.z

        rate.sleep()


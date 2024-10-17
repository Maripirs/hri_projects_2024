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


class JointStateObject:
    def __init__(self):
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.js_subscriber = rospy.Subscriber("joint_states_original", JointState, self.callback_js)
        self.msg = JointState()
        self.readyForMath = True

    def callback_js(self, msg):

        self.msg.header = msg.header
        self.msg.name = msg.name
        positionArr = list(msg.position)
        headYawPosition = 0
        headPitchPosition = 0
        if(len(self.msg.position) > 1):
        	headYawPosition = self.msg.position[0]
        	headPitchPosition = self.msg.position[1]
        if(self.readyForMath):
        	print("updating head")
        	if(len(self.msg.position) > 1):
        		print("current position: ",self.msg.position[1])
        		print("math gives us: ", math.atan2(pitchx, pitchz))
        	headYawPosition = math.atan2(yawy, yawx)  

        	if(self.msg.position):
        		headPitchPosition = -math.atan2(pitchz,pitchx) + headPitchPosition
        	self.readyForMath = False

        positionArr[0] = headYawPosition
        positionArr[1] = headPitchPosition
        
        positionTuple = tuple(positionArr)
        self.msg.position = positionTuple  
       
     
        self.pub.publish(self.msg)

    def get_msg(self):
        return self.msg
    def ready(self):
    	self.readyForMath = True

if __name__ == '__main__':
    rospy.init_node('tf2_look_at_hand')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(5)
    js_global = JointStateObject()
    
    while not rospy.is_shutdown():

        js = js_global
        try:
            trans = tfBuffer.lookup_transform('torso', 'l_gripper', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        yawx = trans.transform.translation.x
        yawy = trans.transform.translation.y


        try:
            trans2 = tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        
        pitchx = trans2.transform.translation.x
        pitchz = trans2.transform.translation.z
        print("Pitch x: ", pitchx, " Pitch z: ", pitchz )
  
        js_global.ready()

        rate.sleep()


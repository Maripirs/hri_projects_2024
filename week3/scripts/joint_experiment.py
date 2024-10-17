#!/usr/bin/python3
# license removed for brevity
import rospy
import math

from sensor_msgs.msg import JointState
class Limb:
    def __init__(self, name,initial, final, rate):
        self.name = name
        self.initial_position = initial
        self.final_position = final
        self.current_position = initial
        self.rate = rate
        self.current_direction = 1
        self.total_distance = self.final_position - self.initial_position
        self.step = self.total_distance / self.rate

    def displayData(self):
        print(f'This limb is a {self.name}')
        print(f'Current position is {self.current_position}')
        print(f'Moving at a rate of {self.rate} cycle(s) per second')

    def move(self):
        new_position = self.current_position + (self.step * self.current_direction)
        if new_position > self.final_position or new_position < self.initial_position:
            self.current_direction*= -1
            new_position = self.current_position + (self.step * self.current_direction)
        
        self.current_position = new_position
        
    def change_rate(self, rate):
        self.rate = rate
        self.step = self.total_distance / self.rate
        
        
def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # set initial angle



    #This one nods	
    #headPitch = Limb("HeadPitch", 0, 0.5, 10)
    
    #This one shakes his head
    headPitch = Limb("HeadPitch", 0, 0.0, 1)
    headYaw = Limb("HeadYaw", -0.5, 0.5 , 10)
    
    lShoulderPitch = Limb("LShoulderPitch", 1.35, 1.35, 1)
    #lShoulderPitch = Limb("LShoulderPitch", 0, 0, 1)
    lShoulderRoll = Limb("LShoulderRoll", 0,0, 1)
    
    
    #This one waves the hand. My robot's arms detatch so I can't see it
    rElbowRoll = Limb("RElboyRoll", -.5, .5 , 10)
    rShoulderPitch = Limb("RShoulderPitch", -1.30, -1.30, 1)
    rShoulderRoll = Limb("RShoulderRoll", 1.33, 1.33 , 1) 
      '''
    rElbowRoll = Limb("RElboyRoll", 0, 0 , 10)
    rShoulderPitch = Limb("RShoulderPitch", 0, 0, 1)
    rShoulderRoll = Limb("RShoulderRoll", 0, 0 , 1)   
    '''
    
    lHipYawPitch = Limb("LHipYawPitch", 0, 0 , 1)
    lHipRoll = Limb("LHipRoll", 0, 1 , 10)
    lHipPitch = Limb("LHipPitch", 0, 0 , 1)
    lKneePitch = Limb("LKneePitch", 0, 0 , 1)
    lAnklePitch = Limb("LAnklePitch", 0, 0 , 1)
    lAnkleRoll = Limb("LAnkleRoll", 0, 0 , 1)
    rHipRoll = Limb("RHipRoll", 0, 0 , 1)
    rHipPitch = Limb("RHipPitch", 0, 1 , 30)
    rKneePitch = Limb("RKneePitch", 0, 1, 10)
    rAnklePitch = Limb("RAnklePitch", 0, 0 , 1)
    rAnkleRoll = Limb("RAnkleRoll", 0, 0 , 1)
    lElbowYaw = Limb("LElbowYaw", 0, 1 , 10)
    lElbowRoll = Limb("LElboyRoll", 0,0 , 1)
    lWristYaw = Limb("LWristYaw", 0, 0 , 1)
    lHand = Limb("LHand", 0, 0 , 1)

    rElbowYaw = Limb("RElbowYaw", 0, 1 , 10)
    rWristYaw = Limb("RWristYaw", 0, 0 , 1)
    rHand = Limb("RHand", 0, 0 , 1)

    
    
    
    
    
    #headPitch,headYaw,lShoulderPitch,lShoulderRoll,lHipYawPitch,lHipRoll,lHipPitch,lKneePitch,lAnklePitch,lAnkleRoll,lrHipRoll,rHipPitch,rKneePitch,rAnklePitch,rAnkleRoll,LElbowYaw,LElbowRoll,LWristYaw,LHand,rShoulderPitch,rShoulderRoll,rElbowYaw,rElbowRoll,rWristYaw,rHand
    #limbs = [headPitch, lShoulderPitch, lShoulderRoll]
    limbs = [headPitch,headYaw,lShoulderPitch,lShoulderRoll,lHipYawPitch,lHipRoll,lHipPitch,lKneePitch,lAnklePitch,lAnkleRoll,rHipRoll,rHipPitch,rKneePitch,rAnklePitch,rAnkleRoll,lElbowYaw,lElbowRoll,lWristYaw,lHand,rShoulderPitch,rShoulderRoll,rElbowYaw,rElbowRoll,rWristYaw,rHand]


    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        js = JointState()
        
        # header info
        js.header.stamp = rospy.get_rostime()
        js.header.frame_id="Torso"


        # put in some joints that we'll edit
        
        for limb in limbs:
        	js.name.append(limb.name)

        for limb in limbs:
        	js.position.append(limb.current_position)

	
        #comment this out once it gets noisy
        rospy.loginfo(js)
        
        pub.publish(js)
        for limb in limbs:
        	limb.move()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/python3
# license removed for brevity
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import String



class Limb:
    def __init__(self, name, initial, final, rate):
        self.name = name
        self.initial_position = initial
        self.final_position = final
        self.current_position = initial
        self.rate = rate
        self.current_direction = 1
        self.total_distance = self.final_position - self.initial_position
        self.step = self.total_distance / self.rate


    def move(self):
        new_position = self.current_position + (self.step * self.current_direction)
        if self.initial_position == self.final_position:
        	new_position = self.initial_position
        if new_position > self.final_position or new_position < self.initial_position:
            self.current_direction *= -1
            new_position = self.current_position + (self.step * self.current_direction)

        self.current_position = new_position


    def change_rate(self, rate):
        self.rate = rate
        self.step = self.total_distance / self.rate
    def reset(self):
    	self.initial_position = 0
    	self.final_position = 0
    	self.current_position = 0
    	self.rate = 1


class Nao:
    def __init__(self):
        self.headPitch = Limb("HeadPitch", 0, 0.0, 1)
        self.headYaw = Limb("HeadYaw", 0, 0, 1)
        self.lShoulderPitch = Limb("LShoulderPitch", 0, 0, 1)
        self.lShoulderRoll = Limb("LShoulderRoll", 0, 0, 1)
        self.rElbowRoll = Limb("RElbowRoll", 0, 0, 1)
        self.rShoulderPitch = Limb("RShoulderPitch", 0, 0, 1)
        self.rShoulderRoll = Limb("RShoulderRoll", 0, 0, 1)
        self.lHipYawPitch = Limb("LHipYawPitch", 0, 0, 1)
        self.lHipRoll = Limb("LHipRoll", 0, 0, 1)
        self.lHipPitch = Limb("LHipPitch", 0, 0, 1)
        self.lKneePitch = Limb("LKneePitch", 0, 0, 1)
        self.lAnklePitch = Limb("LAnklePitch", 0, 0, 1)
        self.lAnkleRoll = Limb("LAnkleRoll", 0, 0, 1)
        self.rHipRoll = Limb("RHipRoll", 0, 0, 1)
        self.rHipPitch = Limb("RHipPitch", 0, 0, 1)
        self.rKneePitch = Limb("RKneePitch", 0, 0, 1)
        self.rAnklePitch = Limb("RAnklePitch", 0, 0, 1)
        self.rAnkleRoll = Limb("RAnkleRoll", 0, 0, 1)
        self.lElbowYaw = Limb("LElbowYaw", 0, 0, 1)
        self.lElbowRoll = Limb("LElbowRoll", 0, 0, 1)
        self.lWristYaw = Limb("LWristYaw", 0, 0, 1)
        self.lHand = Limb("LHand", 0, 0, 1)
        self.rElbowYaw = Limb("RElbowYaw", 0, 0, 1)
        self.rWristYaw = Limb("RWristYaw", 0, 0, 1)
        self.rHand = Limb("RHand", 0, 0, 1)
        self.sub = rospy.Subscriber('/speech_recognition/final_result', String, self.speech_callback)
        self.pub = rospy.Publisher('/tts/phrase', String, queue_size = 10)
        self.counter = 0
        self.animation_length = 30
        self.limbs = [
            self.headPitch,
            self.headYaw,
            self.lShoulderPitch,
            self.lShoulderRoll,
            self.rElbowRoll,
            self.rShoulderPitch,
            self.rShoulderRoll,
            self.lHipYawPitch,
            self.lHipRoll,
            self.lHipPitch,
            self.lKneePitch,
            self.lAnklePitch,
            self.lAnkleRoll,
            self.rHipRoll,
            self.rHipPitch,
            self.rKneePitch,
            self.rAnklePitch,
            self.rAnkleRoll,
            self.lElbowYaw,
            self.lElbowRoll,
            self.lWristYaw,
            self.lHand,
            self.rElbowYaw,
            self.rWristYaw,
            self.rHand
        ]
    def checkCounters(self):
    	if self.counter > 0:
    		self.counter -= 1
    	else:
    		self.reset()
    			

    def shake_head(self):

        current = self.limbs[1].current_position
        self.limbs[1] = Limb("HeadYaw", -0.2, 0.2, 8)
        self.limbs[1].current_position = current
        self.counter = self.animation_length

    def reset(self):
        for limb in self.limbs:
        	limb.reset()

    		
    		
    def nod(self):
        current = self.limbs[0].current_position
        self.limbs[0] = Limb("HeadPitch", -0.2, 0.2, 10)
        self.limbs[0].current_position = current
        self.counter = self.animation_length

  

    def wave(self):
        self.limbs[4] = Limb("RElbowRoll", 0, 1, 10)
        self.limbs[5] = Limb("RShoulderPitch", -1.22, -1.22, 1)
        self.limbs[6] = Limb("RShoulderRoll", -.4, -.4, 1)
        self.counter = self.animation_length
        
    def speech_callback(self, data):
    	print(f'Person said: {data.data}')
    	words = data.data.split(' ')
    	hello = ['hi', 'hello', 'howdy', 'hey']
    	yes = ['yes', 'yeah']
    	no = ['no', 'nah']
    	stop = ['stop', 'still']
    	for word in words:
    		self.pub.publish(word)
    		if word.lower() in hello:
    			print('We should say hi')
    			self.wave()
    		elif word.lower() in yes:
    			print('We should nod')
    			self.nod()
    		elif word.lower() in no:
    			print('We should shake head')
    			self.shake_head()
    		elif word.lower() in stop:
    			print('Not moving')
    			self.reset()
    	
def nao_speech():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    nao = Nao()
    nao.wave()
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        

            js = JointState()
            js.header.stamp = rospy.get_rostime()
            js.header.frame_id = "Torso"

            for limb in nao.limbs:
                js.name.append(limb.name)

            for limb in nao.limbs:
                js.position.append(limb.current_position)

            # comment this out once it gets noisy
            #rospy.loginfo(js)

            pub.publish(js)
            for limb in nao.limbs:
                limb.move()
            nao.checkCounters()
            rate.sleep()
            


	


if __name__ == '__main__':
    try:
        nao_speech()
    except rospy.ROSInterruptException:
        pass


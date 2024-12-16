#!/usr/bin/python3
import rospy
from std_msgs.msg import String


def callback(data):
    print(f'{data.data}')
    words = data.data.split(' ')
    hello = ['hi', 'hello', 'howdy', 'hey']
    yes = ['yes', 'yeah']
    no = ['no', 'nah']
    for word in words:
    	if word.lower() in hello:
    		print('Say hi')
    	elif word.lower() in yes:
    		print('nod')
    	elif word.lower() in no:
    		print('shake head')
    
    

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/speech_recognition/final_result', String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()

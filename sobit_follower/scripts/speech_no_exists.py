#!/usr/bin/env python3
# coding: utf-8
import rospy
from text_to_speech.srv import TextToSpeech
from multiple_sensor_person_tracking.msg import FollowingPosition


speech_flag_global = None

def speech(word):
    rospy.wait_for_service('/speech_word')
    try:
        text_to_speech = rospy.ServiceProxy('/speech_word', TextToSpeech)
        print(word)
        resp = text_to_speech( word )
    except rospy.ServiceException as e:
        print ("Service call failed: {0}".format(e))

def callback_msg(msg):
    global speech_flag_global
    speech_flag_global = msg.status

def main():
    global speech_flag_global
    rospy.init_node('speech2')
    while True:
        rospy.Subscriber('sobit_follower/following_position', FollowingPosition, callback_msg)
        if(speech_flag_global == 0):
            speech_word = "LOST, STOP"
            speech(speech_word)
            rospy.sleep(1)

if __name__ == '__main__':
    main()
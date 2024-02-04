#!/usr/bin/env python3
# coding: utf-8
import rospy
from text_to_speech.srv import TextToSpeech
from person_id_follow_nodelet.msg import SOBITTarget
from multiple_sensor_person_tracking.msg import FollowingPosition


speech_flag_global = None
no_exists_flag_global = None

def speech(word):
    rospy.wait_for_service('/speech_word')
    try:
        text_to_speech = rospy.ServiceProxy('/speech_word', TextToSpeech)
        print(word)
        resp = text_to_speech( word )
    except rospy.ServiceException as e:
        print ("Service call failed: {0}".format(e))

def callback_reid(msg):
    global speech_flag_global
    speech_flag_global = msg.state.data

def callback_state(msg):
    global no_exists_flag_global
    no_exists_flag_global = msg.status

def now():
    return rospy.get_time()

def initial_training_switch(time):
    if (time >= 51):
        return 6
    if (time >= 41):
        return 5
    if (time >= 31):
        return 4
    elif (time >= 21):
        return 3
    elif (time >= 11):
        return 2
    else:
        return 1


# TODO：NO_EXISTSかつre-identificationに拡張
def main():
    global speech_flag_global, no_exists_flag_global
    initial_training_flag = None
    tracking_count = 0
    initial_training_count = 0
    reid_count = 0
    rospy.init_node('speech1')
    start = now()
    while True:
        rospy.Subscriber('/person_id_follow_nodelet/target', SOBITTarget, callback_reid)
        rospy.Subscriber('sobit_follower/following_position', FollowingPosition, callback_state)
        if(speech_flag_global == "initial"):
                start = now()
                speech_word = "準備中"
                speech(speech_word)
                initial_training_count = 0
                rospy.sleep(2)
                continue
        if(speech_flag_global == "initial_training"):
            time = now() - start
            # print("time : ", time)
            initial_training_flag = initial_training_switch(time)
            if(initial_training_flag == 1):
                if initial_training_count == 1:
                    continue
                speech_word = "初期学習開始"
                speech(speech_word)
                initial_training_count = 1
                continue
            # if(initial_training_flag == 2):
            #     if initial_training_count == 2:
            #         continue
            #     speech_word = "反対を向いてください"
            #     speech(speech_word)
            #     initial_training_count = 2
            #     continue
            # if(initial_training_flag == 3):
            #     if initial_training_count == 3:
            #         continue
            #     speech_word = "前に進んでください"
            #     speech(speech_word)
            #     initial_training_count = 3
            #     continue
            # if(initial_training_flag == 4):
            #     if initial_training_count == 4:
            #         continue
            #     speech_word = "左を向いてください"
            #     speech(speech_word)
            #     initial_training_count = 4
            # if(initial_training_flag == 5):
            #     if initial_training_count == 5:
            #         continue
            #     speech_word = "右を向いてください"
            #     speech(speech_word)
            #     initial_training_count = 5
            # if(initial_training_flag == 6):
            #     if initial_training_count == 6:
            #         continue
            #     speech_word = "正面を向いてください"
            #     speech(speech_word)
            #     initial_training_count = 6
        # 追跡開始
        if (no_exists_flag_global == 0):
            speech_word = "見失いました"
            speech(speech_word)
            if (speech_flag_global == "re-identification"):
                if(reid_count > 3):
                    continue
                speech_word = "再同定中"
                speech(speech_word)
                tracking_count = 0
                reid_count += 1
                rospy.sleep(1)
        elif((no_exists_flag_global != 0) and (speech_flag_global == "tracking")):
            if(tracking_count > 0):
                continue
            speech_word = "ついじゅう成功、同定成功"
            speech(speech_word)
            tracking_count = 1
            reid_count = 0
            rospy.sleep(1)
        elif((no_exists_flag_global != 0) and (speech_flag_global == "re-identification")):
            if(reid_count > 3):
                continue
            speech_word = "ついじゅう成功、同定中"
            speech(speech_word)
            tracking_count = 0
            reid_count += 1
            rospy.sleep(1)
        else:
            continue
        
            


if __name__ == '__main__':
    main()
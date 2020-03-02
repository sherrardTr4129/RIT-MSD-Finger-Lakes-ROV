#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

global A_Button_State, lightSetting, motorSpeedSettingVert, motorSpeedSettingHorizontal, joyStamp
A_Button_State, lightSetting, motorSpeedSettingVert, motorSpeedSettingHorizontal, joyStamp = (0, 0, 0, 0, 0)

def joyCallback(msg):
    global A_Button_State, lightSetting, motorSpeedSettingVert, motorSpeedSettingHorizontal, joyStamp

    joyStamp = msg.header.stamp.secs
    A_Button_State = msg.buttons[0]
    lightSetting = msg.axes[-1]

def pubCallback(event):
    global A_Button_State, lightSetting, motorSpeedSettingVert, motorSpeedSettingHorizontal, joyStamp

    pubButton = rospy.Publisher("AButtonState", Int32, queue_size=10)
    pubMotorSpeedVert = rospy.Publisher("VertMotorSpeed", Int32, queue_size=10)
    pubMotorSpeedHori = rospy.Publisher("HorizontalMotorSpeed", Int32, queue_size=10)
    pubLightSetting = rospy.Publisher("LightSettingDelta", Int32, queue_size=10)

    pubButton.publish(A_Button_State)
    pubMotorSpeedVert.publish(motorSpeedSettingVert)
    pubMotorSpeedHori.publish(motorSpeedSettingHorizontal)
    pubLightSetting.publish(lightSetting)

def subProcPubJoy():
    rospy.init_node('joy_to_cmd_node', anonymous=True)
    rospy.Subscriber('joy', Joy, joyCallback)
    timer = rospy.Timer(rospy.Duration(0.0005), pubCallback)

    rospy.spin()
    timer.shutdown()

if __name__ == "__main__":
    print("joy_to_cmd_node initialized")
    subProcPubJoy()

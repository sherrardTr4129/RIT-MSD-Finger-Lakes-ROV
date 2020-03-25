#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Joy

# define constant values
UPPER_DEADBAND = 0.05
LOWER_DEADBAND = -0.05
UPPER_SPEED_THRESH = 0.6
LOWER_SPEED_THRESH = -0.6

# declare and initialize global variables
global A_Button_State, lightSetting, motorSpeedSettingVert, motorSpeedSettingHorizontal, rotationSpeed, joyStamp
global cruiseControlEnabled
A_Button_State, lightSetting, motorSpeedSettingVert, motorSpeedSettingHorizontal, rotationSpeed, joyStamp = (0, 0, 0, 0, 0, 0)
cruiseControlEnabled = 0

def joyStateCheck(joyState):
    motorSpeedToReturn = 0

    # slow forward speed
    if(joyState > UPPER_DEADBAND and joyState < UPPER_SPEED_THRESH):
       motorSpeedToReturn = 1

    # fast forward speed
    elif(joyState > UPPER_SPEED_THRESH):
       motorSpeedToReturn = 2

    # slow inverse speed
    elif(joyState < LOWER_DEADBAND and joyState > LOWER_SPEED_THRESH):
       motorSpeedToReturn = -1

    # fast inverse speed
    elif(joyState < LOWER_SPEED_THRESH):
       motorSpeedToReturn = -2

    # within deadband (0 speed)
    elif(joyState > LOWER_DEADBAND and joyState < UPPER_DEADBAND):
       motorSpeedToReturn = 0

    return motorSpeedToReturn

def joyCallback(msg):
    global A_Button_State, lightSetting, motorSpeedSettingVert
    global motorSpeedSettingHorizontal, rotationSpeed, cruiseControlEnabled, joyStamp

    # get button and axes states
    joyStamp = msg.header.stamp.secs
    A_Button_State = msg.buttons[0]
    lightSettingDelta = msg.axes[-1]
    rightAnalogHoriState = msg.axes[3]
    rightAnalogVertState = msg.axes[4]
    leftAnalogVertState = msg.axes[1]
    rightTriggerState = msg.axes[5]
    leftTriggerState = msg.axes[2]

    # calculate new light setting
    lightSetting += int(lightSettingDelta)
    if(lightSetting > 3):
       lightSetting = 3
    elif(lightSetting < 0):
       lightSetting = 0

    # update motor speed settings (if cruise control is disabled)
    if(cruiseControlEnabled == 0):
        motorSpeedSettingVert = joyStateCheck(rightAnalogVertState)
        motorSpeedSettingHorizontal = joyStateCheck(leftAnalogVertState)
        rotationSpeed = joyStateCheck(rightAnalogHoriState)

    # update cruise control flags
    if(rightTriggerState < 0):
        cruiseControlEnabled = 1
    elif(leftTriggerState < 0):
        cruiseControlEnabled = 0

def pubCallback(event):
    global A_Button_State, lightSetting, cruiseControlEnabled
    global motorSpeedSettingVert, motorSpeedSettingHorizontal, rotationSpeed, joyStamp

    pubButton = rospy.Publisher("AButtonState", Int8, queue_size=10)
    pubMotorSpeedVert = rospy.Publisher("VertMotionSpeed", Int8, queue_size=10)
    pubMotorSpeedHori = rospy.Publisher("HorizontalMotionSpeed", Int8, queue_size=10)
    pubLightSetting = rospy.Publisher("LightSetting", Int8, queue_size=10)
    pubRotation = rospy.Publisher("rotationSpeed", Int8, queue_size=10)
    pubCruiseControl = rospy.Publisher("cruiseEnabled", Int8, queue_size=10)

    pubButton.publish(A_Button_State)
    pubMotorSpeedVert.publish(motorSpeedSettingVert)
    pubMotorSpeedHori.publish(motorSpeedSettingHorizontal)
    pubLightSetting.publish(lightSetting)
    pubRotation.publish(rotationSpeed)
    pubCruiseControl.publish(cruiseControlEnabled)

def subProcPubJoy():
    rospy.init_node('joy_to_cmd_node', anonymous=True)
    rospy.Subscriber('joy', Joy, joyCallback)
    timer = rospy.Timer(rospy.Duration(0.0005), pubCallback)

    rospy.spin()
    timer.shutdown()

if __name__ == "__main__":
    print("joy_to_cmd_node initialized")
    subProcPubJoy()

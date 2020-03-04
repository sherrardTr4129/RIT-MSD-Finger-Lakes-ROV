#!/usr/bin/env python

# Authors: Trevor Sherrard,
# Since: 02/10/2020
# Project: RIT MSD P20250 Finger Lakes ROV Exploration
# filename: flask_node.py

# import required libraries
import rospy
import time
import threading
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from flask import Flask
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

# declare and initialize global variable
global imuData
imuData = [0,0,0,0,0,0,0,0,0]

global VertMotionSpeed, rotationSpeed, HoriMotionSpeed
VertMotionSpeed, HoriMotionSpeed, rotationSpeed = (0, 0, 0)

global cruiseControlEnabled, lightSetting, AButtonState
cruiseControlEnabled, lightSetting, AButtonState = (0, 0, 0)

def imuDataCallback(msg):
    global imuData
    imuData = msg.data

def buttonStateCallback(msg):
    global AButtonState
    AButtonState = msg.data

def lightSettingCallback(msg):
    global lightSetting
    lightSetting = msg.data

def VertMotionSpeedCallback(msg):
    global VertMotionSpeed
    VertMotionSpeed = msg.data

def HoriMotionSpeed(msg):
    global HoriMotionSpeed
    HoriMotionSpeed = msg.data

def rotationSpeedCallback(msg):
    global rotationSpeed
    rotationSpeed = msg.data

def cruiseControlCallback(msg):
    global cruiseControlEnabled
    cruiseControlEnabled = msg.data

@app.route("/imu")
def imuDataFlask():
    global imuData
    now = rospy.get_time()
    return str(now) + "!" + str(imuData)

@app.route("/missiondata")
def missionDataFlask():
    now = rospy.get_time()
    global lightSetting
    temp = "70"
    depth = "250"
    pressure = "7470"
    strToReturn = str(now) + "!"+ temp + "!" + depth + "!" + pressure + "!"
    return strToReturn

@app.route("/motiondata")
def motionDataFlask():
    now = rospy.get_time()
    global VertMotionSpeed, rotationSpeed, HoriMotionSpeed
    strToReturn = str(now) + "!" + str(VertMotionSpeed) + "!" + str(rotationSpeed) + "!" + str(HoriMotionSpeed)
    return strToReturn

@app.route("/misccmddata")
def miscCommandsFlask():
    now = rospy.get_time()
    global cruiseControlEnabled, AButtonState
    strToReturn = str(now) + "!" + str(cruiseControlEnabled) + "!" + AButtonState
    return strToReturn

def main():
    # init node
    threading.Thread(target=lambda: rospy.init_node('flask_node', disable_signals=True)).start()
    print("node initalized...")

    # start subscribing to imu data
    rospy.Subscriber("/imu_data", Float32MultiArray, imuDataCallback)
    rospy.Subscriber("/AButtonState", Int32, buttonStateCallback)
    rospy.Subscriber("/LightSetting", Int32, lightSettingCallback)
    rospy.Subscriber("/VertMotionSpeed", Int32, VertMotionSpeedCallback)
    rospy.Subscriber("/HorizontalMotionSpeed", Int32, HoriMotionSpeed)
    rospy.Subscriber("/rotationSpeed", Int32, rotationSpeedCallback)
    rospy.Subscriber("/cruiseEnabled", Int32, cruiseControlCallback)

    print("began subscribing to data ...")

    app.run(host="0.0.0.0", port=5000)

if __name__ == "__main__":
    main()

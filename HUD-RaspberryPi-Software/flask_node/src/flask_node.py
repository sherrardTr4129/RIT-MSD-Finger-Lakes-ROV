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
from flask import Flask
app = Flask(__name__)

global imuData
imuData = [0,0,0,0,0,0,0,0,0]

def imuDataCallback(msg):
    global imuData
    imuData = msg.data

@app.route("/imu")
def imuDataFlask():
    global imuData
    now = rospy.get_time()
    return str(now) + "!" + str(imuData)

def main():
    # init node
    threading.Thread(target=lambda: rospy.init_node('flask_node', disable_signals=True)).start()
    print("node initalized...")

    # start subscribing to imu data
    sub = rospy.Subscriber("/imu_data", Float32MultiArray, imuDataCallback)
    print("began subscribing to data ...")

    app.run(host="0.0.0.0", port=5000)

if __name__ == "__main__":
    main()

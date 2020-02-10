#!/usr/bin/env python

# Authors: Trevor Sherrard,
# Since: 02/10/2020
# Project: RIT MSD P20250 Finger Lakes ROV Exploration
# filename: imudata_to_rovpose.py

# import required libraries
import rospy
import time
from std_msgs.msg import Float32MultiArray

# global pose data array representing the yaw, pitch
# and roll state estimate of the ROV
poseData = [0.0,0.0,0.0]

def estimatePoseCallback(msg):
    # see message data payload documentation under
    # micro-controller code section of GitHub repo
    imuData = msg.data
    
    yawEstimate = 0.0
    pitchEstimate = 0.0
    rollEstimate = 0.0
    # do something here to estimate the pose of the ROV
    # from message payload

    # update global pose list with new estimates
    poseData[0] = yawEstimate
    poseData[1] = pitchEstimate
    poseData[2] = rollEstimate

def pubPoseData():
    pub = rospy.Publisher('imu_pose_pub', Float32MultiArray, queue_size=10)
    rospy.init_node('imu_pose_publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(poseData)
        rate.sleep()

def main():
    # init node
    rospy.init_node('imudata_to_rovpose')

    # start subscribing to imu data
    sub = rospy.Subscriber("imu_data", Float32MultiArray, estimatePoseCallback)
    rospy.spin()

    # try to start up publisher
    try:
        pubPoseData()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()

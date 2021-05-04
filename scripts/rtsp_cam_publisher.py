#! /usr/bin/env python3

import os
import cv2
import sys
import time
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import *
from threading import Thread
from time import sleep


verbose = True
camera_frame = ''
camera_info_manager = None
camera_info_publisher = None
is_shutdown = False
async_info_publisher = False
async_info_publisher_freq = 30

def camera_info_publisher_fcn(_):
    global camera_frame, camera_info_manager, camera_info_publisher, async_info_publisher_freq, is_shutdown
    wait_time = 1.0 / async_info_publisher_freq
    while True:
        if is_shutdown or rospy.is_shutdown():
            return
        # publish camera calibration
        camera_info_msg = camera_info_manager.getCameraInfo()
        camera_info_msg.header.frame_id = camera_frame
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_publisher.publish( camera_info_msg )
        # wait for 1 second
        sleep(wait_time)


if __name__ == '__main__':
    # initialize ROS node
    print ("Initializing ROS node... "),
    rospy.init_node('rtsp_camera_driver_node')
    print ("Done!")

    # get ROS parameters
    resource = rospy.get_param('~rtsp_resource', 'rtsp://192.168.1.101:8554/mjpeg/1')
    camera_name = rospy.get_param('~camera_name', 'webcam')
    camera_frame = rospy.get_param('~camera_frame', 'webcam')
    image_raw_topic = rospy.get_param('~image_raw_topic', 'image_raw')
    camera_info_topic = rospy.get_param('~camera_info_topic', 'camera_info')
    camera_info_url = rospy.get_param('~camera_info_url', '/tmp')
    camera_fps = rospy.get_param('~camera_fps', 10)
    verbose = rospy.get_param('~verbose', False)

    async_info_publisher_freq = camera_fps

    os.environ['ROS_HOME'] = camera_info_url

    # check the environment variable ROS_HOME
    if( 'ROS_HOME' not in os.environ ):
        rospy.logerr("The environment variable ROS_HOME is not set. Please set it before you run this node. It should be pointing to your `catkin_ws` directory.")
        exit(0)

    # open RTSP stream
    cap = cv2.VideoCapture(resource)
    if not cap.isOpened():
        rospy.logerr("Error opening resource `%s`. Please check." % resource)
        exit(0)
    rospy.loginfo("Resource successfully opened")

    # create publishers
    image_pub = rospy.Publisher(image_raw_topic, Image, queue_size=1)
    camera_info_publisher = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=1)

    # initialize ROS_CV_Bridge
    ros_cv_bridge = CvBridge()

    # initialize Camera Info Manager
    camera_info_manager = CameraInfoManager(cname=camera_name, namespace=camera_name)
    camera_info_manager.loadCameraInfo()
    if not camera_info_manager.isCalibrated():
        rospy.logwarn("No calibration found for the current camera")

    # if async_info_publisher, create and launch async publisher node
    if async_info_publisher:
        camera_info_pub = Thread(target=camera_info_publisher_fcn, args=(camera_info_manager, ))
        camera_info_pub.start()

    # initialize variables
    print ("Correctly opened resource, starting to publish feed.")
    rval, cv_image = cap.read()
    last_t = time.time()
    last_print_t = time.time()
    t_buffer = []

    # process frames
    while rval:
        # get new frame
        rval, cv_image = cap.read()
        # handle Ctrl-C
        key = cv2.waitKey(20)
        if rospy.is_shutdown() or key == 27 or key == 1048603:
            break
        # convert CV image to ROS message
        image_msg = ros_cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        image_msg.header.frame_id = camera_frame
        image_msg.header.stamp = rospy.Time.now()
        image_pub.publish( image_msg )
        # publish camera calibration in case of sync publisher
        if not async_info_publisher:
            camera_info_msg = camera_info_manager.getCameraInfo()
            camera_info_msg.header.frame_id = camera_frame
            camera_info_msg.header.stamp = image_msg.header.stamp
            camera_info_publisher.publish( camera_info_msg )
        # compute frequency
        cur_t = time.time()
        t_buffer.append( cur_t - last_t )
        # print frequency (if verbose)
        if cur_t - last_print_t > 1:
            wait_avg_sec = float(sum(t_buffer))/float(len(t_buffer))
            hz = 1. / wait_avg_sec
            if verbose: rospy.loginfo('Streaming @ %.1f Hz' % hz)
            last_print_t = cur_t
        last_t = cur_t

    # stop thread
    if async_info_publisher:
        is_shutdown = True
        camera_info_pub.join()

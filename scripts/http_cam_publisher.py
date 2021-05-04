#!/usr/bin/env python3
import cv2
import numpy as np
from sensor_msgs.msg import Image 
import roslib
import sys
import rospy
#from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 
import argparse
from urllib import request
from time import sleep

class HTTPcamera(object):
    def __init__(self):

        # get ROS parameters
        self.camera_url = rospy.get_param('~url', 'http://192.168.1.101/')
        self.camera_frame = rospy.get_param('~frame_id', 'http_cam')
        self.image_raw_topic = rospy.get_param('~image_raw_topic', 'image_raw')
        self.gui_view = rospy.get_param('~gui_view', False)
        self.camera_fps = rospy.get_param('~fps', 10)
        
        #self.camera_info_url = rospy.get_param('~camera_info_url', '')

        # open stream source
        rospy.loginfo("Open http video stream at: " + str(self.camera_url))
        try:
            self.stream=request.urlopen(self.camera_url)
        except:
            rospy.logerr('Unable to open camera stream: ' + str(self.camera_url))
            sys.exit() #'Unable to open camera stream')

        # set publisher and opencv bridge
        self.bytes=b''
        self.image_pub = rospy.Publisher(self.image_raw_topic, Image, queue_size=1)
        self.bridge = CvBridge()

    def loop(self):
        #rate = rospy.Rate(self.camera_fps)
        while not rospy.is_shutdown():
            self.bytes += self.stream.read(1024)
            a = self.bytes.find(b'\xff\xd8')
            b = self.bytes.find(b'\xff\xd9')
            if a!=-1 and b!=-1:
                jpg = self.bytes[a:b+2]
                self.bytes= self.bytes[b+2:]
                
                cv_image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                image_msg.header.frame_id = self.camera_frame
                image_msg.header.stamp = rospy.Time.now()
                self.image_pub.publish( image_msg )

                if self.gui_view:
                    cv2.imshow('HTTP Video stream ublisher', cv_image)
                if cv2.waitKey(1) ==27: # wait until ESC key is pressed in the GUI window to stop it
                    exit(0)
            #rate.sleep()        

if __name__ == '__main__':    
    rospy.init_node('HTTPcamera', anonymous=True)
    http_camera = HTTPcamera()
    http_camera.loop()

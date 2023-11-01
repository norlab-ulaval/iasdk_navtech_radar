#!/usr/bin/env python

#################################################################################################################
#
#################################################################################################################

# Imports
# Can be used for recording camera data in ROS from an RTSP stream
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import image_publisher
import sys
# Global variables
### ENTER IP ADDRESS OF RTSP CAMERA ###
camAddress = "YOUR IP ADDRESS"
frameWidth = 0
frameHeight = 0
frameFps = 0
#################################################################################################################


#################################################################################################################
# Main code begins here
#################################################################################################################



class cameraPublisher(object):
    def __init__(self,camAddress):
        self.camAddress = camAddress
        self.cap = cv2.VideoCapture(self.camAddress)

        self.frameWidth = int(self.cap.get(3))
        self.frameHeight = int(self.cap.get(4))
        self.frameFps = int(self.cap.get(4))
        # Print some stream details
        print("----------------------------------------")
        print("RTSP Stream Details")
        print("Width: {}".format(self.frameWidth))
        print("Height: {}".format(self.frameHeight))
        print("FPS: {}".format(self.frameFps))
        print("----------------------------------------")

        self.pub = rospy.Publisher('camera_image_publisher', Image, queue_size=10)
    
    def start(self):
        f = 0
        while not rospy.is_shutdown():
        # Capture frame-by-frame
            ret, frame = self.cap.read()
            cv2.waitKey(1)

            # Break on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            br = CvBridge()
            if frame is not None: 
                frame = cv2.resize(frame, (500,500), interpolation = cv2.INTER_AREA)

                immsg = br.cv2_to_imgmsg(frame,"bgr8")
                immsg.header.stamp = rospy.get_rostime()
                immsg.header.seq = f
                f=f+1
                self.pub.publish(immsg)


        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('image_streamer',anonymous=True)
    node = cameraPublisher(camAddress)
    node.start()


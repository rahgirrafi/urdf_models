'''
A ros2 node for image processing using openCV. This node will read from the two image topics from two camera and use openCV template matching method to identify the same objects in both image. 
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os


class OpenCVImageProcessing(Node):

    def __init__(self):
        global img1, img2
        super().__init__('openCV_img_processing')
        self.get_logger().info('OpenCV Image Processing Node Started')
        self.subscription1 = self.create_subscription( Image, '/camera1/image_raw', self.image_callback1, 10)
        self.subscription2 = self.create_subscription( Image,'/camera2/image_raw', self.image_callback2, 10)
        self.bridge = CvBridge()
        self.img1 = None
        self.img2 = None

    def image_callback1(self, msg):
        self.img1 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Image1', self.img1)
        cv2.waitKey(3)
        self.image_processing()

    def image_callback2(self, msg):
        self.img2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Image2', self.img2)
        cv2.waitKey(3)
        self.image_processing()

    def image_processing(self):
        if self.img1 is not None and self.img2 is not None:
            self.object_detector(self.img1, self.img2)
        else:
            self.get_logger().info('Images not received yet')

    def object_detector(self, img1, img2):
        img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        # Initialize ORB detector
        orb = cv2.ORB_create()

        # Find the keypoints and descriptors with ORB
        keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
        keypoints2, descriptors2 = orb.detectAndCompute(img2, None)

        # Create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Match descriptors
        matches = bf.match(descriptors1, descriptors2)

        # Sort them in the order of their distance
        matches = sorted(matches, key = lambda x:x.distance)

        # Draw first 10 matches
        img3 = cv2.drawMatches(img1, keypoints1, img2, keypoints2, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)


        cv2.imshow('Detected', img3)
        cv2.waitKey(3)




def main(args=None):
    rclpy.init(args=args)
    openCV_img_processing = OpenCVImageProcessing()
    rclpy.spin(openCV_img_processing)
    openCV_img_processing.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
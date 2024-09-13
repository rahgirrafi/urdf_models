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


stereo = cv2.StereoBM.create()

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
        cv2.imwrite('/home/rafi/ros2/urdf_ws/src/assembly_refined_description/captures/image1.jpg', self.img1)
        cv2.imshow('Image1', self.img1)
        cv2.waitKey(3)
        self.image_processing()

    def image_callback2(self, msg):
        self.img2 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite('/home/rafi/ros2/urdf_ws/src/assembly_refined_description/captures/image2.jpg', self.img2)
        cv2.imshow('Image2', self.img2)
        cv2.waitKey(3)
        self.image_processing()

    def image_processing(self):
        if self.img1 is not None and self.img2 is not None:
            self.object_detector(self.img1, self.img2)
        else:
            self.get_logger().info('Images not received yet')

    def object_detector(self, img1, img2):

        
        window_size = 2  # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely

        left_matcher = cv2.StereoSGBM_create(
            minDisparity=-1,
            numDisparities=8,  # max_disp has to be dividable by 16 f. E. HH 192, 256
            blockSize=window_size,
            P1=9 * 3 * window_size,
            # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
            P2=128 * 3 * window_size,
            disp12MaxDiff=12,
            uniquenessRatio=40,
            speckleWindowSize=50,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
        # FILTER Parameters
        lmbda = 70000
        sigma = 1.7
        visual_multiplier = 6

        wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma)

        displ = left_matcher.compute(img2, img1)  # .astype(np.float32)/16
        dispr = right_matcher.compute(img1, img2)  # .astype(np.float32)/16
        displ = np.int16(displ)
        dispr = np.int16(dispr)
        filteredImg = wls_filter.filter(displ, img2, None, dispr)  # important to put "img1" here!!!
        filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
        filteredImg = np.uint8(filteredImg)


        cv2.imshow('disparity', filteredImg)

        

        
        

        

def main(args=None):

    rclpy.init(args=args)
    openCV_img_processing = OpenCVImageProcessing()
    rclpy.spin(openCV_img_processing)
    cv2.destroyAllWindows()
    openCV_img_processing.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
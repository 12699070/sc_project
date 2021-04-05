import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

class ImageProcessing:
        def __init__(self):
                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        def image_callback(self, msg):
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
                gray = np.float32(gray)
                dst = cv2.cornerHarris(gray,2,3,0.04)
                #result is dilated for marking the corners, not important
                dst = cv2.dilate(dst,None)
                # Threshold for an optimal value, it may vary depending on the image.
                image[dst>0.01*dst.max()]=[0,0,255]

                cv2.imshow("window", image)
                cv2.waitKey(3)

rospy.init_node('follower')

imPros = ImageProcessing()
rospy.spin()
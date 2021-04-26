"""
TurtlebotVS class

Square size: 30x30 cm
Cam res: 1920 x 1080
"""

class TurtlebotVS:

    #IMPORT LIBRARIES
    import configparser, os
    import math
    import numpy as np

    import rospy
    import cv2, cv_bridge
    from sensor_msgs.msg import Image
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from geometry_msgs.msg import Twist

    import time
    import functions

  
    def __init__(self):
        self.rospy.init_node('visual_servoing')

        #CvBridge converts ROS Image messages to OpenCV images (http://wiki.ros.org/cv_bridge)
        self.bridge = self.cv_bridge.CvBridge()
        self.cv2.namedWindow("window", self.cv2.WINDOW_AUTOSIZE)

        #region SUBCRIBERS (Format: 'topic_name',MsgType,CallbackFunc)
        #Subcribe to Turtlebot camera (Intel Realsense R200)      
        self.image_sub = self.rospy.Subscriber('camera/rgb/image_raw',self.Image, self.image_callback)
        #Subcribe to Turtlebot odometer to get roll, pitch, yaw
        self.odom_sub = self.rospy.Subscriber ('/odom', self.Odometry, self.odom_callback)
        #Subcribe to Turtlebot laser scan distances
        self.laser_sub = self.rospy.Subscriber('/scan',self.LaserScan, self.scan_callback)
        #endregion


        # CLASS VARIABLES
        self.camera_center_x = 1920/2
        self.camera_center_y = 1080/2

        self.marker_centroid_x = None
        self.marker_centroid_y = None
        self.marker_found = False
        
        self.corner_0_top_left = None
        self.corner_1_top_right = None
        self.corner_2_bottom_right = None
        self.corner_3_bottm_left = None

        self.goal_point_0 = None
        self.goal_point_1 = None
        self.goal_point_2 = None
        self.goal_point_3 = None
        self.goal_point_4 = None

        self.corner_list = []
        self.range_list = []
        
        self.trigger_vs = False



    """
    Callbacks
    """
    def image_callback(self,msg):        
        global res
        img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        #Green color filter
        hsv = self.cv2.cvtColor(img, self.cv2.COLOR_BGR2HSV)
        lower_green= self.np.array([40,40,40])
        upper_green = self.np.array([70,255,255])
        mask = self.cv2.inRange(hsv, lower_green, upper_green)
        res = self.cv2.bitwise_and(img,img, mask= mask)

        #Convert to gray
        gray = self.cv2.cvtColor(res, self.cv2.COLOR_BGR2GRAY)    


        #Centroid of a blob https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
        ret,thresh = self.cv2.threshold(gray,127,255,0)
        try:
            M = self.cv2.moments(thresh)
            
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            self.cv2.circle(res, (cX, cY), 5, (255, 255, 255), -1)
            self.cv2.putText(res, "centroid", (cX - 25, cY - 25),self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            self.marker_centroid_x = cX
            self.marker_centroid_y = cY
            print('Marker found')
            self.marker_found = True

            # print('Centroid: ',self.marker_centroid_x,self.marker_centroid_y)
            print('------------------------------')

            self.x_list = []
            self.y_list = []

            # Shi-Tomasi corner detection
            # https://blog.francium.tech/feature-detection-and-matching-with-opencv-5fd2394a590
            corners = self.cv2.goodFeaturesToTrack(gray, maxCorners=4,qualityLevel=0.02, minDistance=20)
            for i in range(len(corners)):
                # print('Corner #' + str(i) + ': ',int(corners[i,0,0]),int(corners[i,0,1]))
                self.x_list.append(int(corners[i,0,0]))
                self.y_list.append(int(corners[i,0,1]))
                # self.cv2.putText(res, str(i), (int(corners[i,0,0]),int(corners[i,0,1])),self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                # self.cv2.circle(res, (int(corners[i,0,0]), int(corners[i,0,1])), 2, (255, 0, 0), -1)

            # Center line 1920 x 1080
            start_vertical_line = (917,0)
            end_vertical_line = (917,1080)

            start_horizontal_line = (0,464)
            end_horizontal_line = (1920,464)

            color = (0, 255, 255)
            thickness = 2

            res = self.cv2.line(res, start_vertical_line, end_vertical_line, color, thickness)
            res = self.cv2.line(res, start_horizontal_line, end_horizontal_line, color, thickness)

            # sort corner
            """
            [0,0]    +
                *--------->
                |
              + |    .Centroid
                |
                V
            """

            self.corner_list = []
            for i in range(len(self.y_list)):
                if self.y_list[i] < self.marker_centroid_y:
                    if self.x_list[i] < self.marker_centroid_x:
                        self.corner_0_top_left = [self.x_list[i],self.y_list[i]]
                    elif self.x_list[i] > self.marker_centroid_x:
                        self.corner_1_top_right = [self.x_list[i],self.y_list[i]]

                elif self.y_list[i] > self.marker_centroid_y:
                    if self.x_list[i] > self.marker_centroid_x:
                        self.corner_2_bottom_right = [self.x_list[i],self.y_list[i]]
                    elif self.x_list[i] < self.marker_centroid_x:
                        self.corner_3_bottm_left = [self.x_list[i],self.y_list[i]]

            self.corner_list = [
                self.corner_0_top_left,
                self.corner_1_top_right,
                self.corner_2_bottom_right,
                self.corner_3_bottm_left
            ]

            # print(self.corner_list)            
            for i in range(len(self.corner_list)):
                self.cv2.circle(res, (self.corner_list[i][0], self.corner_list[i][1]), 5, (255, 0, 0), -1)

        #Catch error if centroid not found
        except ZeroDivisionError or TypeError:
            print('Marker not found')
            self.marker_found = False
            self.marker_centroid_x = None
            self.marker_centroid_y = None


        self.cv2.imshow("window", res)
        self.cv2.waitKey(3)
        

    def scan_callback(self,msg):
        self.range_list = msg.ranges
        return self.range_list





    """
    Functions
    """
    def odom_callback(self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        from tf.transformations import euler_from_quaternion, quaternion_from_euler
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        # print ('Yaw deg: ',(self.math.degrees(yaw)-180*-1) %90)



    def servoing(self):
        while not self.rospy.is_shutdown():
            while self.marker_found == True:
                pub = self.rospy.Publisher('cmd_vel',self.Twist,queue_size=1)
                move = self.Twist()
                move.linear.x = 0 #m/s
                move.linear.y = 0 #m/s
                move.angular.z = 0 #yaw
                pub.publish(move)

                while not self.rospy.is_shutdown():
                    # Control
                    try:
                        f = [1400,1400]
                        p = [1920/2,1080/2]
                        Z = self.range_list[0]                        
                        l = 0.1

                        target = self.np.matrix([
                            [644,192],
                            [1190,192],
                            [1190,738],
                            [644,738],
                            [917,464]                        
                        ])

                        global res
                        for i in range(len(target)):
                            self.cv2.circle(res, (target[i][0,0], target[i][0,1]), 5, (0, 0, 255), -1)



                        obs = self.np.matrix([
                            [self.corner_0_top_left[0], self.corner_0_top_left[1]],
                            [self.corner_1_top_right[0], self.corner_1_top_right[1]],
                            [self.corner_2_bottom_right[0], self.corner_2_bottom_right[1]],
                            [self.corner_3_bottm_left[0], self.corner_3_bottm_left[1]],
                            [self.marker_centroid_x,self.marker_centroid_y]
                        ])

                        xy = (target-p)/f
                        obsxy = (obs-p)/f

                        n = len(target[:,1])
                        Lx = self.np.zeros((0,6))
                        for i in range(n):
                            Lxi = self.np.matrix(self.functions.FuncLx(xy[i,0],xy[i,1],Z))
                            Lx = self.np.concatenate((Lxi,Lx))

                        e2 = obsxy-xy

                        e = self.np.reshape(self.np.transpose(e2),(10,1))

                        de = -e*l

                        Lx2 = self.np.linalg.inv(self.np.transpose(Lx)*Lx)*self.np.transpose(Lx)
                        Vc = -l*Lx2*e

                        
                        if Z > 3:
                            move.linear.x = Vc[0]*-1
                            move.linear.y = Vc[1]
                            move.linear.z = Vc[2]
                            move.angular.x = Vc[3]
                            move.angular.y = Vc[4]
                            move.angular.z = Vc[5]

                        elif Z < 3 and Z > 1.2:
                            move.linear.x = Vc[0]*-1
                            move.linear.y = Vc[1]
                            move.linear.z = Vc[2]
                            move.angular.x = Vc[3]
                            move.angular.y = Vc[4]
                            move.angular.z = Vc[5]
                        
                        elif Z < 1.2 and Z > 0.2:
                            move.linear.x = Vc[0]*-1
                            move.linear.y = Vc[1]
                            move.linear.z = Vc[2]
                            move.angular.x = Vc[3]
                            move.angular.y = Vc[4]
                            move.angular.z = Vc[5]


                        pub.publish(move)
                        # print(self.np.matrix.round(Vc,2))

                    except:
                        self.spin('stop')

            while self.marker_found == False:
                self.trigger_vs = False


    def spin(self,direction):
        pub = self.rospy.Publisher('cmd_vel',self.Twist,queue_size=1)
        move = self.Twist()
        move.linear.x = 0
        move.linear.y = 0
        if direction == 'left':
            move.angular.z = -0.1 
        elif direction == 'right':
            move.angular.z = +0.1 
        elif direction == 'stop':
            move.angular.z = 0
        pub.publish(move)



    """
    Main
    """
    def run_time(self):
        tolerance = 100
        trigger_vs = False
        while not self.rospy.is_shutdown():
            while self.marker_found == True:
                while (self.marker_found == True and self.marker_centroid_x < self.camera_center_x - tolerance):
                    trigger_vs = False
                    self.spin('right')
                    print('right',self.marker_centroid_x, self.camera_center_x - tolerance)

                while (self.marker_found == True and self.marker_centroid_x > self.camera_center_x + tolerance):
                    trigger_vs = False
                    self.spin('left')
                    print('left',self.marker_centroid_x, self.camera_center_x - tolerance)
                    
                self.spin('stop')
                self.trigger_vs = True
                while self.trigger_vs == True:
                    self.servoing()
                    





"""
End of class. Begin runtime
"""
if __name__ == '__main__':
    a = TurtlebotVS()
    a.run_time()

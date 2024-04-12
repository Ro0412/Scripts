#!/usr/bin/env python
from geometry_msgs.msg import Vector3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32, Float32MultiArray, String
import sensor_msgs.point_cloud2 as pc2
import tf
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from tf.transformations import quaternion_matrix

from std_msgs.msg import Header

class FruitDetectionNode:
    def string_callback(self, msg):
        rospy.loginfo("Received string message: {}".format(msg.data))

        # Split the string message into words using spaces as separators
        self.words = msg.data.split()

        self.fruit = self.words[0]

    def plants_beds_callback(self, msg):
        self.plants_beds_received = True

    def __init__(self):
        rospy.init_node('fruit_detection_node', anonymous=True)
        self.plants_beds_received = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.point_cloud_msg = None
        rospy.Subscriber('/red/plants_beds', String, self.plants_beds_callback)
        while not self.plants_beds_received:
            print("Waiting")
            rospy.sleep(1)

        # Set up a subscriber for the camera feed
        rospy.Subscriber('/red/plants_beds', String, self.string_callback)
        rospy.Subscriber('/red/odometry', Odometry, self.current_position)
        rospy.Subscriber('/red/camera/depth/image_raw', Image, self.depth_image_callback)
        # Set up a publisher for fruit counts
        self.fruit_count_publisher = rospy.Publisher('/fruits', Int32, queue_size=10)
        self.fruit_publisher = rospy.Publisher('/fruit_count', Int32, queue_size=10)
        self.image_subscriber = rospy.Subscriber('/red/camera/color/image_raw', Image, self.image_callback)
        # Set up a subscriber for the UAV velocity
        self.reached = rospy.Subscriber('/reached', Int32, self.reached_callback)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        self.fruit_count = 0
        self.front_array = []
        self.back_array = []
        self.current_orientation_z = 0
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        self.centroids=[]
        self.depth=[]
        self.flag0=0
        # Flag to check if fruit count has been published
        self.fruit_count_published = False
        self.image_processing_lock = False
        self.flag=0
    def current_position(self, msg):
        self.current_x = round(msg.pose.pose.position.x, 2)
        self.current_y = round(msg.pose.pose.position.y, 2)
        self.current_z = round(msg.pose.pose.position.z, 2)
        self.current_orientation_z = msg.pose.pose.orientation.z

    def filter_color(self, image, lower_bound, upper_bound, color_name, min_area=100, max_area=5000):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mask_filtered = np.zeros_like(mask)
        cv2.imshow(f'Filtered {color_name} Mask', mask_filtered)
        color_count = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if min_area <= area <= max_area and self.fruit=="Eggplant":
                cv2.drawContours(mask_filtered, [contour], -1, 255, thickness=cv2.FILLED)
                    
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                self.centroids.append((cx, cy))
        
            elif area > 700 and self.fruit=="Pepper" and area>60:  # Check if area is greater than 700
            # Divide contours into two halves along a horizontal line
                _, _, w, h = cv2.boundingRect(contour)
                top_half = contour[contour[:, :, 1] < (contour[:, :, 1].max() + contour[:, :, 1].min()) / 2]
                bottom_half = contour[contour[:, :, 1] >= (contour[:, :, 1].max() + contour[:, :, 1].min()) / 2]

                # Find centroids for each half
                for half_contour in [top_half, bottom_half]:
                    M = cv2.moments(half_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        self.centroids.append((cx, cy))
                        # Draw centroid on the image
                        cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.drawContours(image, [contour], -1, (0, 0, 255), 2)

            elif self.fruit=="Pepper" and area>60:
                # Eliminate smaller regions from the mask
                cv2.drawContours(mask_filtered, [contour], -1, 0, thickness=cv2.FILLED)
                # Calculate centroid for the mask
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self.centroids.append((cx, cy))
                    # Draw centroid on the image
                    cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.drawContours(image, [contour], -1, (0, 0, 255), 2)
            
            elif self.fruit=="Tomato" and area>60:
                # Eliminate smaller regions from the mask
                cv2.drawContours(mask, [contour], -1, 0, thickness=cv2.FILLED)
                # Calculate centroid for the mask
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    if cx>80 and cy>180:
                        self.centroids.append((cx, cy))
                    # Draw centroid on the image
                    cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
                cv2.drawContours(mask_filtered, [contour], -1, (0, 0, 255), 2)

        
        return color_count, mask_filtered

    def image_callback(self, msg):
        if self.image_processing_lock:
            return
        self.image_processing_lock = True

        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Define color ranges
            yellow_lower = np.array([25, 150, 150])
            yellow_upper = np.array([35, 255, 255])

            purple_lower = np.array([130, 50, 50])
            purple_upper = np.array([170, 255, 255])

            red_lower = np.array([0, 100, 100])
            red_upper = np.array([15, 255, 255])  # Adjust this range as needed for red
            
            if self.reached==0:
                self.flag=0

            if self.reached == 1 and not self.fruit_count_published:
                # Publish fruit counts only when velocity is zero in all axes
                fruit_count_msg = Int32()
                if self.fruit == "Pepper":
                    yellow_count, _ = self.filter_color(cv_image, yellow_lower, yellow_upper, 'Yellow', min_area=60, max_area=5000)
                    total_yellow_count = yellow_count
                    if self.flag==0:
                        print(self.centroids)
                        self.flag=1
                    self.fruit_count = len(self.centroids)
                if self.fruit == "Tomato":
                    red_count, _ = self.filter_color(cv_image, red_lower, red_upper, 'Red', min_area=100, max_area=5000)
                    total_red_count = red_count
                    self.fruit_count = len(self.centroids)
                    if self.flag==0:
                        print(self.centroids)
                        self.flag=1
                if self.fruit == "Eggplant":
                    purple_count, _ = self.filter_color(cv_image, purple_lower, purple_upper, 'Purple', min_area=100, max_area=5000)
                    total_purple_count = purple_count
                    self.fruit_count = len(self.centroids)
                    if self.flag==0:
                        print(self.centroids)
                        self.flag=1

                if 1.25 <= self.current_x <= 1.45 or 7.25 <= self.current_x <= 7.45 or 13.2 <= self.current_x <= 13.4:
                    if not (25.9 <= self.current_y <= 26.1):
                        self.front_array.append(self.fruit_count)
                if 6.5 <= self.current_x <= 6.7 or 12.5 <= self.current_x <= 12.7 or 18.7 <= self.current_x <= 18.9:
                    if not (25.9 <= self.current_y <= 26.1):
                        self.back_array.append(self.fruit_count)

                print(self.front_array)
                print(self.back_array)
                print(self.current_x)
                print(self.current_y)

                self.fruit_count_published = True
            if self.reached == 0:
                self.fruit_count_published = False
            if self.reached == 2:
                fruit_msg = Int32()
                fruit_msg.data=0
                fruits=0
                for x in self.front_array:
                    fruits=fruits+x
                for y in self.back_array:
                    fruits=fruits+y
                fruit_msg.data=fruits
                self.fruit_publisher.publish(fruit_msg)
            # Display masks and counts
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(e)

        self.image_processing_lock = False
    def depth_image_callback(self, msg):
        self.depth = []
        invalid_indices = []  # List to store indices of centroids with invalid depths

        try:
            depth_image_np = self.bridge.imgmsg_to_cv2(msg)

            if self.reached == 0:
                self.depth = []
                self.flag0 = 0

            if self.reached == 1 and self.flag0 == 0:
                for i, centroid in enumerate(self.centroids):
                    depth = depth_image_np[centroid[1], centroid[0]]  # [y, x]
                    if 2 <= depth <= 2.5:  # Check if depth is within the specified range
                        self.depth.append(depth)
                    else:
                        invalid_indices.append(i)  # Add index of centroid with invalid depth

                # Remove centroids with invalid depths from the centroids array
                for index in sorted(invalid_indices, reverse=True):
                    del self.centroids[index]

                self.flag0 = 1

        except CvBridgeError as e:
            rospy.logerr(e)

    def reached_callback(self, msg):
        self.reached = msg.data
        if self.reached == 0:
            self.centroids = []
        





if __name__ == '__main__':
    try:
        fruit_detection_node = FruitDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
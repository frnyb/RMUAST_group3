#!/usr/bin/python2

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from math import sqrt
import transforms3d

class ArucoLander():
    def __init__(
            self,
            node_name="aruco_lander",
            image_topic="/iris_fpv_cam/usb_cam/image_raw",
            image_detected_topic="/aruco_image",
            pose_topic="/pose",
            camera_matrix=np.array([
                [277.191356, 0.0, 160],
                [0.0, 277.191356, 120],
                [0.0, 0.0, 1.0]]),
            distortion_matrix=np.array([[0, 0, 0, 0, 0]], dtype='f'),
            aruco_length=2
    ):
        rospy.init_node(node_name)

        self.setpoint = PoseStamped()
        self.setpoint.pose.position.x = 0
        self.setpoint.pose.position.y = 0
        self.setpoint.pose.position.z = 20

        self.pos_thresh = 0.1

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.camera_matrix = camera_matrix
        self.distortion_matrix = distortion_matrix
        self.aruco_length = aruco_length

        self.odom_pose = None

        rospy.sleep(0.05)

        self.state = "init"

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
                image_topic,
                Image,
                callback=self.on_new_image,
                queue_size=1
        )

        self.pose_pub = rospy.Publisher(
                pose_topic,
                PoseStamped,
                queue_size=1
        )

        self.image_detected_pub = rospy.Publisher(
                image_detected_topic,
                Image,
                queue_size=1
        )

        self.state_sub = rospy.Subscriber(
                "/mavros/local_position/odom",
                Odometry,
                callback=self.update_odom
        )

        rospy.spin()

    def update_odom(
            self,
            msg
    ):
        self.odom_pose = msg.pose

    def on_new_image(
            self,
            msg
    ):
        img = self.bridge.imgmsg_to_cv2(
                msg,
                "bgr8"
        )

        if self.state == "init":
            self.pose_pub.publish(self.setpoint)

            if self.odom_pose != None and self.odom_pose.pose.position.z > 0.5:
                self.state = "ascending"

        elif self.state == "ascending" and self.odom_pose != None:
            euc_dist = sqrt((self.odom_pose.pose.position.x - self.setpoint.pose.position.x)**2 + (self.odom_pose.pose.position.y - self.setpoint.pose.position.y)**2 + (self.odom_pose.pose.position.z - self.setpoint.pose.position.z)**2)
            if euc_dist <= self.pos_thresh:
                self.state = "aruco"
        
        elif self.state == "aruco":
            if self.detect_aruco(img):
                self.setpoint.pose.position.x = self.odom_pose.pose.position.x - (self.aruco_pose[0, 3])
                self.setpoint.pose.position.y = self.odom_pose.pose.position.y - (self.aruco_pose[1, 3])

                self.pose_pub.publish(self.setpoint)

                self.state = "aligning"

        elif self.state == "aligning":
            euc_dist = sqrt((self.odom_pose.pose.position.x - self.setpoint.pose.position.x)**2 + (self.odom_pose.pose.position.y - self.setpoint.pose.position.y)**2 + (self.odom_pose.pose.position.z - self.setpoint.pose.position.z)**2)
            rospy.loginfo(euc_dist)

            if euc_dist <= self.pos_thresh:
                self.setpoint.pose.position.z = 0
                self.pose_pub.publish(self.setpoint)
                self.state = "descending"

        elif self.state == "descending":
            if self.detect_aruco(img):
                self.setpoint.pose.position.x = self.odom_pose.pose.position.x - self.aruco_pose[0, 3]
                self.setpoint.pose.position.y = self.odom_pose.pose.position.y - self.aruco_pose[1, 3]
                rospy.loginfo("Descending and found aruco")

            if self.odom_pose.pose.position.x <= 0.1:
                self.state = "landed"

        elif self.state == "landed":
            rospy.loginfo("landed")
            self.state = "finish"
            
    def detect_aruco(
            self,
            img
    ):
        corners, ids, rejected_pts = cv2.aruco.detectMarkers(
                img,
                self.dictionary
        )

        if ids != None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.aruco_length,
                    self.camera_matrix,
                    self.distortion_matrix
            )

            for i in range(rvecs.shape[0]):
                rvec = rvecs[i, :] 
                tvec = tvecs[i, :] 

            R, _ = cv2.Rodrigues(rvec)

            self.aruco_pose = transforms3d.affines.compose(
                    tvec[0],
                    R,
                    np.ones(3),
                    np.zeros(3)
            )

            img_drawn = cv2.aruco.drawDetectedMarkers(
                    img,
                    corners
            )

            img_drawn = cv2.aruco.drawAxis(
                    img_drawn, 
                    self.camera_matrix, 
                    self.distortion_matrix, 
                    rvec, 
                    tvec, 
                    self.aruco_length
            ) 

            ros_img = self.bridge.cv2_to_imgmsg(
                    img_drawn,
                    "bgr8"
            )

            self.image_detected_pub.publish(ros_img)

            return True
        else:
            ros_img = self.bridge.cv2_to_imgmsg(
                    img,
                    "bgr8"
            )

            self.image_detected_pub.publish(ros_img)

            return False

if __name__ == '__main__':
    ArucoLander()


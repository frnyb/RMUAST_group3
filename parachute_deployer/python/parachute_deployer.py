#!/usr/bin/python2

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from math import sqrt, pi
import transforms3d
from mavros_msgs.srv import SetMode
import os

class ParachuteDeployer():
    def __init__(
            self,
            node_name="parachute_deployer",
            imu_topic="/mavros/imu/data_raw"
    ):
        rospy.init_node(node_name)

        rospy.sleep(0.05)

        self.pitch_fails = []
        self.roll_fails = []
        self.alt_fails = []
        self.fail_interval = 50
        self.fail_state = "ok"

        self.imu_sub = rospy.Subscriber(
                imu_topic,
                Imu,
                callback=self.on_new_imu_data,
                queue_size=1
        )

        rospy.spin()

    def on_new_imu_data(
            self,
            msg
    ):
        if self.fail_state == "ok":
            pitch_vel = msg.angular_velocity.x
            roll_vel = msg.angular_velocity.y
            alt_acc = msg.linear_acceleration.z

            if abs(pitch_vel) > pi:
                self.pitch_fails.append(True)
                rospy.loginfo("pitch fail")
            else:
                self.pitch_fails.append(False)
            if abs(roll_vel) > pi:
                self.roll_fails.append(True)
                rospy.loginfo("roll fail")
            else:
                self.roll_fails.append(False)
            if alt_acc < 0.5:
                self.alt_fails.append(True)
                rospy.loginfo("alt fail")
            else:
                self.alt_fails.append(False)

            if len(self.pitch_fails) > self.fail_interval:
                self.pitch_fails = self.pitch_fails[-self.fail_interval:]
            if len(self.roll_fails) > self.fail_interval:
                self.roll_fails = self.roll_fails[-self.fail_interval:]
            if len(self.alt_fails) > self.fail_interval:
                self.alt_fails = self.alt_fails[-self.fail_interval:]

            if (
                    np.asarray(self.pitch_fails).any() == True and
                    np.asarray(self.roll_fails).any() == True
            ):
                rospy.loginfo("Attitude fail")
                self.fail_state = "fail"
                self.set_mode_lockdown()

            if(np.asarray(self.alt_fails).any() == True):
                rospy.loginfo("Altitude fail")
                self.fail_state = "fail"
                self.set_mode_lockdown()

    def set_mode_lockdown(self):
        os.system('rosservice call /mavros/cmd/command "{broadcast: false, command: 185,confirmation: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0,param5: 0.0, param6: 0.0, param7: 0.0}"')
        #rospy.wait_for_service('mavros/set_mode')

        #try:
        #    flight_mode_service = rospy.ServiceProxy(
        #            'mavros/set_mode',
        #            SetMode
        #    )
        #    flight_mode_service(custom_mode='Lockdown')
        #except rospy.ServiceException, e:
        #    print(e)

if __name__ == '__main__':
    ParachuteDeployer()


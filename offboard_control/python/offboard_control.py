#!/usr/bin/python2

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardController():
    def __init__(
            self,
            node_name,
            pose_sub_topic="/pose"
    ):
        rospy.init_node(node_name)

        self.rate = rospy.Rate(20)

        self.state = State()

        self.pose = None

        self.state_sub = rospy.Subscriber(
                "mavros/state",
                State,
                callback=self.on_new_state,
                queue_size=1
        )

        self.pose_pub = rospy.Publisher(
                "mavros/setpoint_position/local",
                PoseStamped,
                queue_size=1
        )

        self.pose_sub = rospy.Subscriber(
                pose_sub_topic,
                PoseStamped,
                callback=self.on_new_pose,
                queue_size=1
        )

    def start(self):
        while self.pose == None:
            self.rate.sleep()

        for i in range(100):
            self.pose_pub.publish(self.pose)
            self.rate.sleep()

        self.set_mode_offboard()
        self.arm_drone()

        while True:
            self.pose_pub.publish(self.pose)
            self.rate.sleep()


    def on_new_pose(
            self,
            msg
    ):
        self.pose = msg

    def arm_drone(self):
        rospy.wait_for_service('mavros/cmd/arming')

        try:
            arm_service = rospy.ServiceProxy(
                    'mavros/cmd/arming',
                    CommandBool
            )
            arm_service(True)
        except rospy.ServiceException, e:
            print(e)

    def set_mode_offboard(self):
        rospy.wait_for_service('mavros/set_mode')

        try:
            flight_mode_service = rospy.ServiceProxy(
                    'mavros/set_mode',
                    SetMode
            )
            flight_mode_service(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print(e)

    def on_new_state(
        self,
        msg
    ):
        self.state = msg

if __name__ == '__main__':
    offb_ctl = OffboardController("offboard_controller")
    offb_ctl.start()



#!/usr/bin/env python3
import os
import rospy, cv2
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range


class Example(object):

    def __init__(self):
        rospy.loginfo("igo320341 loaging")
        rospy.on_shutdown(self.shutdown)

        self.gui = os.getenv('GUI')=='true' or os.getenv('GUI')=='True'

        sub_image_topic_name = "/head/camera1/image_raw"
        self.camera_subscriber = rospy.Subscriber(sub_image_topic_name, Image, self.camera_cb)

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        
        self.range_front_subscriber = rospy.Subscriber("/range/front", Range, self.range_front_callback)
        self.range_left_subscriber = rospy.Subscriber("/range/left", Range, self.range_left_callback)
        self.range_right_subscriber = rospy.Subscriber("/range/right", Range, self.range_right_callback)

        self.command = Twist()

        self.sonar_data = [0, 0, 0]

        self.bridge = CvBridge()
        rospy.loginfo("igo320341 loaded")

    def shutdown(self):
        self.cmd_vel.publish(Twist())

    def camera_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # blue mask
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # red mask
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # detect range
        self.h, self.w, _ = frame.shape
        search_top = 2 * self.h // 3
        mask_blue[0:search_top, 0:self.w] = 0
        mask_red[0:search_top, 0:self.w] = 0

        self.mb = cv2.moments(mask_blue)
        self.mr = cv2.moments(mask_red)

        if self.gui != False:
            cv2.imshow("output", frame)
            cv2.imshow("mask_blue", mask_blue)
            cv2.imshow("mask_red", mask_red)
            cv2.waitKey(1)

    def range_front_callback(self, msg):
        self.sonar_data[0] = msg.range

    def range_left_callback(self, msg):
        self.sonar_data[1] = msg.range

    def range_right_callback(self, msg):
        self.sonar_data[2] = msg.range

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():

            # follow blue zones
            if self.mb['m00'] > 0:
                cx = self.mb['m10'] // self.mb['m00']
                err = cx - self.w / 2
                self.command.linear.x = 0.5
                self.command.linear.y = 0.0
                self.command.angular.z = -err / 100
                self.cmd_vel.publish(self.command)

            # avoid red zones
            elif self.mr['m00'] > 0:
                cx = self.mr['m10'] // self.mr['m00']
                err = cx - self.w / 2
                self.command.angular.z = np.sign(err)
                self.cmd_vel.publish(self.command)

            # discover world
            else:

                # move forward
                if self.sonar_data[0] > 0.7:
                    self.command.linear.x = 0.5
                    self.command.linear.y = 0.0
                    self.command.angular.z = 0.0
                    self.cmd_vel.publish(self.command)

                # avoid obstacle
                else:
                    self.command.linear.x = -0.4
                    self.command.linear.y = 0.0
                    self.command.angular.z = 0.0
                    self.cmd_vel.publish(self.command)
                    rospy.sleep(0.5)

                    self.command.linear.x = 0.0
                    self.command.linear.y = 0.0
                    if self.sonar_data[1] > self.sonar_data[2]:
                        self.command.angular.z = 2.4
                    else:
                        self.command.angular.z = -2.4
                    self.cmd_vel.publish(self.command)
                    rospy.sleep(0.5)
            rate.sleep()


def main(args=None):
    rospy.init_node("control_node")
    exp = Example()
    rospy.sleep(2)
    exp.spin()

if __name__ == "__main__":
    main()
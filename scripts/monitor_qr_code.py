#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import sensor_msgs.msg

import ros_qr_code.srv
import util

import argparse
import sys

class Monitor:
    def __init__(self, node_name):
        self.node_name = node_name
        rospy.loginfo("Start %s" % (self.node_name,))
        rospy.init_node(self.node_name)
        self.cv_bridge = cv_bridge.CvBridge()
        #
        detection_service_name = rospy.get_param(
            "~detection_service_name", 'detect_qr_code'
        )
        input_topic = rospy.resolve_name('input')
        output_topic = rospy.resolve_name('output')
        self.detection_service_name = detection_service_name
        self.input_topic = input_topic
        self.output_topic = output_topic
        rospy.loginfo('input_topic: %s' % (self.input_topic,))
        rospy.loginfo('output_topic: %s' % (self.output_topic,))

    def start_server(self):
        rospy.loginfo('Waiting for the service "%s"...'
                      % (self.detection_service_name,))
        rospy.wait_for_service(self.detection_service_name)
        self.detection_service = rospy.ServiceProxy(
            self.detection_service_name, ros_qr_code.srv.DetectQRCode
        )
        rospy.loginfo('Connected to the service "%s".'
                      % (self.detection_service_name,))
        #
        # Prepare the publisher object before registering the callback
        # function for subscription.
        self.image_pub = rospy.Publisher(
            self.output_topic, sensor_msgs.msg.Image, queue_size=10
        )
        #
        self.image_sub = rospy.Subscriber(
            self.input_topic, sensor_msgs.msg.Image,
            self.callback
        )
        rospy.loginfo("Ready to monitor QR codes.")
        rospy.spin()
        return

    def callback(self, data):
        try:
            res = self.detection_service(data)
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
            return

        cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        util.visualize_result_onto(cv_image, res)
        try:
            self.image_pub.publish(
                self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
            )
        except cv_bridge.CvBridgeError as e:
            print(e)



# Usage:
# rosrun ros_qr_code monitor_qr_code.py input:=/camera/color/image_raw output:=/monitor_qr_code/image
#
def main(args):
    parser = argparse.ArgumentParser(
        description='ROS services for monitoring QR codes.',
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    self_node_name = 'monitor_qr_code'
    monitor = Monitor(self_node_name)
    monitor.start_server()
    return

if __name__ == "__main__":
    main(sys.argv)

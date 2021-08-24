#!/usr/bin/env python
import roslib
import rospy

import pyzbar.pyzbar

import cv_bridge
import geometry_msgs.msg
import sensor_msgs.msg

import argparse
import sys

import ros_qr_code.srv

class QRCodeDetector:
    def __init__(self, node_name):
        self.node_name = node_name
        rospy.loginfo("Invoke rospy.init_node().")
        rospy.init_node(self.node_name)

        self.cv_bridge = cv_bridge.CvBridge()
        target_symbol_specifier = rospy.get_param("~target_symbols", None)
        if target_symbol_specifier is None:
            target_symbol_specifier = 'ALL'

        target_symbol_str_list = sorted(
            target_symbol_specifier.upper().split()
        )
        self.target_symbols = [
            pyzbar.pyzbar.ZBarSymbol[k] for k in target_symbol_str_list
        ]
        self.target_symbols_str = ' '.join(target_symbol_str_list)
        if 'ALL' in target_symbol_str_list:
            self.target_symbols = None
            self.target_symbols_str = 'ALL'

        rospy.loginfo("node_name: %s" % (self.node_name,))
        rospy.loginfo("target_symbols: %s" % (self.target_symbols_str,))

    def start_server(self):
        rospy.loginfo('Invoke rospy.Service().')
        rospy.Service(
            self.node_name, ros_qr_code.srv.DetectQRCode,
            self.respond_to_service_request
        )
        rospy.loginfo("Ready to detect QR codes.")
        rospy.spin()

    #
    def respond_to_service_request(self, req):
        res = ros_qr_code.srv.DetectQRCodeResponse()
        rospy.loginfo('Received a RGB image [%dx%d].'
                      % (req.image.width, req.image.height))
        rgb_cv_img = self.cv_bridge.imgmsg_to_cv2(req.image, "bgr8")
        rospy.loginfo('rgb_cv_img.shape: %s, rgb_cv_img.dtype: %s'
                      % (rgb_cv_img.shape, rgb_cv_img.dtype))

        detection_result_list = pyzbar.pyzbar.decode(
            rgb_cv_img, self.target_symbols
        )

        for result in detection_result_list:
            # regions
            roi_param = {
                'x_offset': result.rect.left,
                'y_offset': result.rect.top,
                'width': result.rect.width,
                'height': result.rect.height,
                'do_rectify': True
            }
            res.regions.append(
                sensor_msgs.msg.RegionOfInterest(**roi_param)
            )

            # polygons
            polygon = geometry_msgs.msg.Polygon()
            for p in result.polygon:
                z = 0
                polygon.points.append(geometry_msgs.msg.Point32(p.x, p.y, z))
            res.polygons.append(polygon)

            # decoded_strings
            res.decoded_strings.append(str(result.data, 'ascii'))

            # types
            res.types.append(result.type)
        #
        #
        return res


# Usage:
# rosrun ros_qr_code detect_qr_code.py
#
def main(args):
    parser = argparse.ArgumentParser(
        description='ROS services for detecting QR codes.',
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    self_node_name = 'detect_qr_code'
    qr_code_detector = QRCodeDetector(self_node_name)
    qr_code_detector.start_server()

if __name__ == '__main__':
    main(sys.argv)



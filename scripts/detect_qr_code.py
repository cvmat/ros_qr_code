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

        service_list = [
            ('detect_qr_code',
             ros_qr_code.srv.DetectQRCode,
             self.respond_to_service_request),
            ('detect_qr_code_until_success',
             ros_qr_code.srv.DetectQRCodeUntilSuccess,
             self.detect_until_success)
        ]

        self.service_info_list = []
        for default_name, type_obj, method in service_list:
            parameter_name = '~service_name_for_' + type_obj.__name__
            service_name = rospy.get_param(parameter_name, default_name)
            self.service_info_list.append(
                (service_name, type_obj, method)
            )

        self.cv_bridge = cv_bridge.CvBridge()
        target_symbol_specifier = rospy.get_param("~target_symbols", None)
        if target_symbol_specifier is None:
            target_symbol_specifier = 'ALL'

        target_symbol_str_list = sorted(
            target_symbol_specifier.upper().split()
        )
        if 'ALL' in target_symbol_str_list:
            self.target_symbols = None
            self.target_symbols_str = 'ALL'
        else:
            self.target_symbols = [
                pyzbar.pyzbar.ZBarSymbol[k] for k in target_symbol_str_list
            ]
            self.target_symbols_str = ' '.join(target_symbol_str_list)

        rospy.loginfo("node_name: %s" % (self.node_name,))
        rospy.loginfo("services: %s" % (self.service_info_list,))
        rospy.loginfo("target_symbols: %s" % (self.target_symbols_str,))

    def start_server(self):
        rospy.loginfo('Invoke rospy.Service().')
        self.service = []
        for name, type_obj, method in self.service_info_list:
            self.service.append(rospy.Service(name, type_obj, method))
        #
        rospy.loginfo("Ready to detect QR codes.")
        rospy.spin()

    def respond_to_service_request(self, req):
        rgb_cv_img = self.cv_bridge.imgmsg_to_cv2(req.image, "bgr8")

        detection_result_list = pyzbar.pyzbar.decode(
            rgb_cv_img, self.target_symbols
        )

        res = ros_qr_code.srv.DetectQRCodeResponse()
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

    #
    def detect_until_success(self, req):
        """Detect the largest QR code repeatedly until succeeding the detection.

        Arguments:
        req.topic -- the input topic from which an image will be retrieved.
        req.compressed_input -- True if the input topic publishes compressed images.
        req.timeout -- timeout.
        req.interval -- the interval for retrying image retrieval.
        """
        input_topic = req.topic
        input_is_compressed = req.compressed_input
        timeout = req.timeout
        interval_sec = req.interval
        timeout_flag = False
        def timeout_callback(event):
            timeout_flag = True
        #
        timer = rospy.Timer(timeout, timeout_callback, oneshot=True)
        res = ros_qr_code.srv.DetectQRCodeUntilSuccessResponse()
        while timeout_flag == False:
            try:
                input_img_msg = rospy.wait_for_message(
                    input_topic, sensor_msgs.msg.Image,
                    timeout=timeout.to_sec()
                )
            except rospy.exceptions.ROSException as e:
                # Catch the exception occurred by timeout.
                break
            #
            if timeout_flag:
                break
            current_req = ros_qr_code.srv.DetectQRCodeRequest()
            current_req.image = input_img_msg
            current_res = self.respond_to_service_request(current_req)
            if 0 < len(current_res.regions):
                # Disable the timer.
                timer.shutdown()
                # Copy ros_qr_code.srv.DetectQRCodeResponse into
                # ros_qr_code.srv.DetectQRCodeUntilSuccessResponse .
                for key in ['regions', 'polygons', 'decoded_strings', 'types']:
                    setattr(res, key, getattr(current_res, key))
                break
            #
            rospy.sleep(interval_sec)
        # Disable the timer.
        timer.shutdown()
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



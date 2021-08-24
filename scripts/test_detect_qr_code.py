#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import sensor_msgs.msg

import ros_qr_code.srv

import argparse
import sys


def visualize_detected_result(img_msg, result_msg):
    import util
    bridge = cv_bridge.CvBridge()
    cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    visualized_cv_img = util.visualize_result_onto(cv_img, result_msg)
    return visualized_cv_img

def main():
    parser = argparse.ArgumentParser(
        description='ROS client program that detect codes in an image.',
        epilog='''
EXAMPLE:
rosrun ros_qr_code test_detect_qr_code.py --input /camera/color/image_raw --json_output output.json
'''
    )
    parser.add_argument('--input', default='',
                        help='an input topic')
    parser.add_argument('--json_output', default='',
                        help='an output JSON filename')
    parser.add_argument('--visualized_output', default='',
                        help='an output image filename')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    input_topic = args.input
    json_output_filename = args.json_output
    visualized_output_filename = args.visualized_output

    rospy.init_node('test_detect_qr_code', anonymous=True)
    published_topics = rospy.get_published_topics()

    rospy.loginfo('Waiting for the topic "%s"...' % (input_topic,))
    input_img_msg = rospy.wait_for_message(input_topic, sensor_msgs.msg.Image)
    rospy.loginfo('Received an image [%dx%d].'
                  % (input_img_msg.width, input_img_msg.height))

    service_name = 'detect_qr_code'
    rospy.loginfo('Waiting for the service "%s"...' % (service_name,))
    rospy.wait_for_service(service_name)
    service = rospy.ServiceProxy(service_name, ros_qr_code.srv.DetectQRCode)
    rospy.loginfo('Connected to the service "%s".' % (service_name,))

    try:
        from timeit import default_timer as timer
        msg = ros_qr_code.srv.DetectQRCodeRequest()
        msg.image = input_img_msg
        start = timer()
        res = service(msg)
        end = timer()
        rospy.loginfo('Finished detection. (%f [sec])' % (end - start, ))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

    #
    result_list = []
    for region, polygon, decoded_string, code_type \
        in zip(res.regions, res.polygons, res.decoded_strings,
               res.types):
        region_as_dict = {
            key: getattr(region, key) for key
            in ['x_offset', 'y_offset', 'width', 'height']
        }
        #
        polygon_as_list = [
            {key: getattr(p, key) for key in ['x', 'y', 'z']}
            for p in polygon.points
        ]
        #
        result = {
            'region': region_as_dict,
            'polygon': polygon_as_list,
            'string': decoded_string,
            'type': code_type
        }
        result_list.append(result)
    #

    if json_output_filename != '':
        import json
        with open(json_output_filename, 'w') as fp:
            fp.write(json.dumps(result_list, sort_keys=True, indent=2))
    #
    print(result_list)

    if visualized_output_filename != '':
        import cv2
        visualized_cv_img = visualize_detected_result(input_img_msg, res)
        cv2.imwrite(visualized_output_filename, visualized_cv_img)

    return

if __name__ == "__main__":
    main()

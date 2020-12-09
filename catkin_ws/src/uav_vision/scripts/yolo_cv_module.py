#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, Int8
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes

import numpy as np
import math
#
# import cv2
# from cv_bridge import CvBridge, CvBridgeError
# bridge = CvBridge()
#
# import color_settings
import config as cfg
#
# # For ground truth callback:
# from nav_msgs.msg import Odometry
# from scipy.spatial.transform import Rotation as R
#
# Settings
global_is_simulator = cfg.is_simulator
# save_images = cfg.save_images
# draw_on_images = cfg.draw_on_images
# use_test_image = cfg.use_test_image
#
#
if global_is_simulator:
    camera_offset_x = 150 # mm
    camera_offset_z = -45 # mm
else:
    camera_offset_x = -60 # mm
    camera_offset_z = -45 # mm

# # Constants
# D_H_SHORT = 4.0 # cm
# D_H_LONG = 12.0 # cm
# D_ARROW = 30.0 # cm
# D_RADIUS = 39.0 # cm
#
IMG_WIDTH = 640
IMG_HEIGHT = 360
#
# global_image = None
#
# #################
# # Help functions #
# #################

def rad_to_deg(rad):
    return rad*180/math.pi

def deg_to_rad(deg):
    return deg*math.pi/180




# def hsv_save_image(image, label='image', is_gray=False):
#     if image is None:
#         print "The image with label " + label + " is none"
#     folder = './image_processing/'
#     if save_images:
#         if is_gray:
#             cv2.imwrite(folder+label+".png", image)
#         else:
#             cv2.imwrite(folder+label+".png", cv2.cvtColor(image, cv2.COLOR_HSV2BGR))
#     return image
#
#
# def load_bgr_image(filename):
#     bgr = cv2.imread(filename) # import as BGR
#     # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert to HSV
#     return bgr
#
#
# def draw_dot(img, position, color, size=3):
#     if draw_on_images:
#         cX = np.int0(position[1])
#         cY = np.int0(position[0])
#         cv2.circle(img, (cX, cY), size, color, -1)
#
# # Colors to draw with
# HSV_RED_COLOR = rgb_color_to_hsv(255,0,0)
# HSV_BLUE_COLOR = rgb_color_to_hsv(0,0,255)
# HSV_BLACK_COLOR = rgb_color_to_hsv(0,0,0)
#
# HSV_YELLOW_COLOR = [30, 255, 255]
# HSV_LIGHT_ORANGE_COLOR = [15, 255, 255]
#
#
# if global_is_simulator:
#     HUE_LOW_WHITE, HUE_HIGH_WHITE, SAT_LOW_WHITE, SAT_HIGH_WHITE, VAL_LOW_WHITE, VAL_HIGH_WHITE, \
#     HUE_LOW_ORANGE, HUE_HIGH_ORANGE, SAT_LOW_ORANGE, SAT_HIGH_ORANGE, VAL_LOW_ORANGE, VAL_HIGH_ORANGE, \
#     HUE_LOW_GREEN, HUE_HIGH_GREEN, SAT_LOW_GREEN, SAT_HIGH_GREEN, VAL_LOW_GREEN, VAL_HIGH_GREEN = color_settings.SIMULATOR_COLOR_LIMITS
# else:
#     HUE_LOW_WHITE, HUE_HIGH_WHITE, SAT_LOW_WHITE, SAT_HIGH_WHITE, VAL_LOW_WHITE, VAL_HIGH_WHITE, \
#     HUE_LOW_ORANGE, HUE_HIGH_ORANGE, SAT_LOW_ORANGE, SAT_HIGH_ORANGE, VAL_LOW_ORANGE, VAL_HIGH_ORANGE, \
#     HUE_LOW_GREEN, HUE_HIGH_GREEN, SAT_LOW_GREEN, SAT_HIGH_GREEN, VAL_LOW_GREEN, VAL_HIGH_GREEN = color_settings.REAL_COLOR_LIMITS


def get_test_bounding_boxes():

    Helipad = BoundingBox()
    Helipad.probability = 0.5
    Helipad.xmin = 312
    Helipad.ymin = 120
    Helipad.xmax = 337
    Helipad.ymax = 148
    Helipad.id = 2
    Helipad.Class = "Helipad"

    bbs = BoundingBoxes()
    bbs.bounding_boxes = [Helipad]
    return bbs


# ####
# ####
# ####
#
#
# global_image = None
# def image_callback(data):
#     global global_image
#
#     try:
#         global_image = bridge.imgmsg_to_cv2(data, 'bgr8') # {'bgr8' or 'rgb8}
#     except CvBridgeError as e:
#         rospy.loginfo(e)
#
global_ground_truth = None
def gt_callback(data):
    global global_ground_truth
    global_ground_truth = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])


global_bounding_boxes = None
def bb_callback(data):
    global global_bounding_boxes
    global_bounding_boxes = data
#
#
# ####
# ####
# ####
#
#
# def calculate_estimation_errors(estimate, gt):
#     global pub_est_error_Helipad
#     global pub_est_error_H
#     global pub_est_error_H_scaled
#
#     error = Twist()
#
#     error.linear.x = estimate.linear.x - gt[0]
#     error.linear.y = estimate.linear.y - gt[1]
#     error.linear.z = estimate.linear.z - gt[2]
#     error.angular.x = estimate.angular.x - gt[3]
#     error.angular.y = estimate.angular.y - gt[4]
#     error.angular.z = estimate.angular.z - gt[5]
#
#     return error
#
#

def transform_pixel_position_to_world_coordinates(center_px, radius_px):
    focal_length = 374.67
    real_radius = 390 # mm (780mm in diameter / 2)

    # Center of image
    x_0 = IMG_HEIGHT/2.0
    y_0 = IMG_WIDTH/2.0

    # Find distances from center of image to center of LP
    d_x = x_0 - center_px[0]
    d_y = y_0 - center_px[1]

    est_z = real_radius*focal_length / radius_px

    # Camera is placed 150 mm along x-axis of the drone
    # Since the camera is pointing down, the x and y axis of the drone
    # is the inverse of the x and y axis of the camera
    est_x = -((est_z * d_x / focal_length) + camera_offset_x) # mm adjustment for translated camera frame in x direction
    est_y = -(est_z * d_y / focal_length)
    est_z += camera_offset_z # mm adjustment for translated camera frame in z direction

    position = np.array([est_x, est_y, est_z]) / 1000.0

    return position

#
#
def find_best_bb_of_class(bounding_boxes, classname):
    matches =  list(item for item in bounding_boxes if item.Class == classname)
    best = max(matches, key=lambda x: x.probability)
    return best

def est_center_of_bb(bb):
    width = bb.xmax - bb.xmin
    height = bb.ymax - bb.ymin
    center = [bb.xmin + width/2,bb.ymin + height/2]
    map(int, center)
    return center

def est_radius_of_bb(bb):
    width = bb.xmax - bb.xmin
    height = bb.ymax - bb.ymin
    radius = int(max(width, height)/2)
    return radius


def estimate_center_and_radius_px(bounding_boxes):
    # rospy.loginfo(bounding_boxes)
    classes = list(item.Class for item in bounding_boxes)
    rospy.loginfo(classes)

    if 'H' in classes:
        if 'Arrow' in classes:
            pass
        else:
            pass

    elif 'Helipad' in classes:
        Helipad = find_best_bb_of_class(bounding_boxes, 'Helipad')
        center = est_center_of_bb(Helipad)
        radius = est_radius_of_bb(Helipad)

    else:
        center = [None, None]
        radius = None
    return center, radius
#
# def publish_ground_truth(current_ground_truth):
#     global pub_ground_truth
#
#     ground_truth_msg = Twist()
#     ground_truth_msg.linear.x = current_ground_truth[0]
#     ground_truth_msg.linear.y = current_ground_truth[1]
#     ground_truth_msg.linear.z = current_ground_truth[2]
#     ground_truth_msg.angular.z = current_ground_truth[5]
#
#     pub_ground_truth.publish(ground_truth_msg)


def main():
#     global pub_ground_truth
#
#     global pub_est_Helipad
#     global pub_est_H
#     global pub_est_Scaled_H
#
#     global pub_est_error_Helipad
#     global pub_est_error_H
#     global pub_est_error_Scaled_H
#-
#     global global_ground_truth
#     global yolo_output_image

    rospy.init_node('yolo_cv_module', anonymous=True)
    rospy.Subscriber('/drone_ground_truth', Twist, gt_callback)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, bb_callback)
    # rospy.Subscriber('/darknet_ros/detection_image')

    #
    # pub_bb_processed_image = rospy.Publisher('/bb_processed_image', Image, queue_size=10)
    #
    # pub_est_Helipad = rospy.Publisher("/estimate/Helipad", Twist, queue_size=10)
    # pub_est_H = rospy.Publisher("/estimate/H", Twist, queue_size=10)
    # pub_est_Scaled_H = rospy.Publisher("/estimate/Scaled_H", Twist, queue_size=10)
    #
    # pub_est_error_Helipad = rospy.Publisher("/estimate_error/error_Helipad", Twist, queue_size=10)
    # pub_est_error_H = rospy.Publisher("/estimate_error/error_H", Twist, queue_size=10)
    # pub_est_error_Scaled_H = rospy.Publisher("/estimate_error/error_Scaled_H", Twist, queue_size=10)est_px
    #
    #
    # pub_est = rospy.Publisher("/yolo_estimate", Twist, queue_size=10)
    # pub_est_method = rospy.Publisher("/estimate_method", Int8, queue_size=10)
    #
    # est_msg = Twist()
    #
    # rospy.loginfo("Starting yolo_CV module")
    #
    #
    #
    # if not global_is_simulator:
    #     global_ground_truth = np.zeros(6)
    #
    #
    # if use_test_image:
    #     test_image_filepath = './image_36.png'
    #     # test_image_filepath = './0_hsv.png'
    #     global_image = load_bgr_image(test_image_filepath)
    #
    #     # corner_test()

    use_test_bbs = 1
    previous_bounding_boxes = None
    current_pose_estimate = None
    count = 0
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        # rospy.loginfo('In loop')
        #
        current_ground_truth = global_ground_truth # Fetch the latest ground truth pose available
        if use_test_bbs:
            current_bounding_boxes = get_test_bounding_boxes()
        else:
            current_bounding_boxes = global_bounding_boxes
        # rospy.loginfo(global_bounding_boxes)
        if current_bounding_boxes is not None:
            if current_bounding_boxes != previous_bounding_boxes:
                previous_bounding_boxes = current_bounding_boxes
                center_px, radius_px = estimate_center_and_radius_px(current_bounding_boxes.bounding_boxes)
                rospy.loginfo('center_px: %s,  radius_px: %s', center_px, radius_px)
                current_pose_estimate = transform_pixel_position_to_world_coordinates(center_px, radius_px)
                rospy.loginfo(current_pose_estimate)



        else:
            rospy.loginfo("current_bounding_boxes: %s", current_bounding_boxes)

        # if (global_image is not None) and (current_ground_truth is not None):
        #     # denoised = cv2.fastNlMeansDenoisingColored(global_image,None,10,10,7,21) # denoising
        #     hsv = cv2.cvtColor(global_image, cv2.COLOR_BGR2HSV) # convert to HSV
        #     est, method, processed_image = get_estimate(hsv, count, current_ground_truth)
        #
        #     if processed_image is not None:
        #         pub_processed_image.publish(processed_image)
        #
        #     pub_est_method.publish(Int8(method))
        #
        #     # Publish the estimate
        #     est_msg.linear.x = est[0]
        #     est_msg.linear.y = est[1]
        #     est_msg.linear.z = est[2]
        #     est_msg.angular.z = est[5]
        #     pub_est.publish(est_msg)
        #
        #     # publish_ground_truth(current_ground_truth)
        #
        #     count += 1
        #     if use_test_image:
        #         break
        # else:
        #     rospy.loginfo("Waiting for image")


        rate.sleep()

if __name__ == '__main__':
    main()

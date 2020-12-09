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
H_scaling = [(1.0, 1.0), (0.975, 0.989), (0.951, 0.978), (0.928, 0.968),
(0.907, 0.958), (0.887, 0.948), (0.869, 0.94), (0.851, 0.931),
(0.834, 0.923), (0.818, 0.916), (0.803, 0.909), (0.789, 0.902),
 (0.775, 0.895), (0.762, 0.889), (0.75, 0.884), (0.738, 0.878),
 (0.727, 0.873), (0.717, 0.869), (0.707, 0.864), (0.697, 0.86),
  (0.688, 0.856), (0.68, 0.853), (0.672, 0.85), (0.664, 0.847),
   (0.656, 0.844), (0.649, 0.842), (0.643, 0.84), (0.636, 0.838),
    (0.63, 0.836), (0.624, 0.835), (0.619, 0.834), (0.614, 0.833),
     (0.609, 0.832), (0.604, 0.832), (0.6, 0.832), (0.595, 0.832),
     (0.591, 0.833), (0.588, 0.833), (0.584, 0.834), (0.581, 0.836),
      (0.578, 0.837), (0.575, 0.839), (0.572, 0.841), (0.57, 0.843),
       (0.568, 0.846), (0.566, 0.849), (0.564, 0.852), (0.562, 0.855),
        (0.561, 0.859), (0.559, 0.863), (0.558, 0.867), (0.557, 0.872),
         (0.556, 0.876), (0.556, 0.882), (0.555, 0.887), (0.555, 0.893),
          (0.555, 0.899), (0.555, 0.906), (0.555, 0.913), (0.555, 0.92),
           (0.556, 0.928), (0.557, 0.936), (0.557, 0.945), (0.559, 0.954),
            (0.56, 0.964), (0.561, 0.974), (0.563, 0.984), (0.564, 0.996),
             (0.566, 1.007), (0.569, 1.02), (0.571, 1.033), (0.573, 1.046),
              (0.576, 1.06), (0.579, 1.075), (0.582, 1.091), (0.586, 1.108),
               (0.589, 1.125), (0.593, 1.143), (0.597, 1.163), (0.601, 1.183),
               (0.606, 1.205), (0.611, 1.227), (0.616, 1.251), (0.621, 1.276),
               (0.626, 1.303), (0.632, 1.331), (0.639, 1.361), (0.645, 1.393),
                (0.652, 1.426), (0.659, 1.462), (0.667, 1.5)]
# #################
# # Help functions #
# #################

def rad2deg(rad):
    return rad*180/math.pi

def deg2rad(deg):
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

    H = BoundingBox()
    H.probability = 0.5
    H.xmin = 320
    H.ymin = 128
    H.xmax = 330
    H.ymax = 138
    H.id = 0
    H.Class = "H"

    Arrow = BoundingBox()
    Arrow.probability = 0.5
    Arrow.xmin = 333
    Arrow.ymin = 140
    Arrow.xmax = 335
    Arrow.ymax = 143
    Arrow.id = 1
    Arrow.Class = "Arrow"

    bbs = BoundingBoxes()
    bbs.bounding_boxes = [Helipad, H, Arrow]
    return bbs


global_ground_truth = None
def gt_callback(data):
    global global_ground_truth
    global_ground_truth = np.array([data.linear.x, data.linear.y, data.linear.z, 0, 0, data.angular.z])


global_bounding_boxes = None
def bb_callback(data):
    global global_bounding_boxes
    global_bounding_boxes = data



# def calculate_estimation_error(estimate, gt):
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

def est_rotation(H, Arrow):
    # rotation defined as positive right of line between H and arrow.
    x_h, y_h = est_center_of_bb(H)
    x_a, y_a = est_center_of_bb(Arrow)
    theta = math.atan2(y_a-y_h, x_a-x_h)
    theta -= math.pi/2
    theta *= -1
    return theta

def downscale_H_by_rotation(H, rotation):
    cx, cy = est_center_of_bb(H)
    theta = int(rad2deg(rotation))
    theta = theta % 180
    theta = abs(theta)

    cos = abs(math.cos(rotation))
    sin = abs(math.sin(rotation))
    scaling_width = (cos*2 + sin*3)/2
    scaling_height = (sin*2 + cos*3)/3

    width = H.xmax - H.xmin
    height = H.ymax - H.ymin
    new_width = width * scaling_width
    new_height = height * scaling_height

    H.xmin = cx - int(new_width/2)
    H.ymin = cy - int(new_height/2)
    H.xmax = cx + int(new_width/2)
    H.ymax = cy + int(new_height/2)

    return H


def estimate_center_rotation_and_radius(bounding_boxes):
    # rospy.loginfo(bounding_boxes)
    classes = list(item.Class for item in bounding_boxes)
    rospy.loginfo(classes)
    center = [None, None]
    radius = None
    rotation = None
    if 'H' in classes:
        H = find_best_bb_of_class(bounding_boxes, 'H')
        if 'Arrow' in classes:
            Arrow = find_best_bb_of_class(bounding_boxes, 'Arrow')
            rotation = est_rotation(H, Arrow)
            H = downscale_H_by_rotation(H, rotation)
        center = est_center_of_bb(H)
        radius = 3*est_radius_of_bb(H)

    else:
        if 'Helipad' in classes:
            Helipad = find_best_bb_of_class(bounding_boxes, 'Helipad')
            center = est_center_of_bb(Helipad)
            radius = est_radius_of_bb(Helipad)

    return center, radius, rotation
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
#     global pub_est_pose

#     global pub_est_pose_error

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
    est_pose_msg = Twist()

    rospy.loginfo("Starting yolo_CV module")
    # if not global_is_simulator:
    #     global_ground_truth = np.zeros(6)


    use_test_bbs = 1
    previous_bounding_boxes = None
    current_pose_estimate = None
    count = 0
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        current_ground_truth = global_ground_truth # Fetch the latest ground truth pose available
        if use_test_bbs:
            current_bounding_boxes = get_test_bounding_boxes()
        else:
            current_bounding_boxes = global_bounding_boxes
        # rospy.loginfo(global_bounding_boxes)
        if (current_bounding_boxes is not None) and (current_bounding_boxes != previous_bounding_boxes):
            previous_bounding_boxes = current_bounding_boxes
            center_px, radius_px, rotation = estimate_center_rotation_and_radius(current_bounding_boxes.bounding_boxes)
            rospy.loginfo('center_px: %s,  radius_px: %s,  rotation: %s', center_px, radius_px, rotation)
            current_pose_estimate = transform_pixel_position_to_world_coordinates(center_px, radius_px)
            rospy.loginfo('current_est: %s', current_pose_estimate)



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

#!/usr/bin/env python
import numpy as np
import math

img_format = 720.0/1280.0



def height_to_delta_x_y(height):
    R = 0.386 # m
    m = 0.0 # m

    a = height*math.tan(math.radians(45)) - (R + m)
    alpha = math.atan(360.0/640.0)

    delta_x = a * math.sin(alpha)
    delta_y = a * math.cos(alpha)

    print('delta_x:', delta_x)
    print('delta_y:', delta_y)

    return delta_x, delta_y

reference_height = 1.0
# delta_x, delta_y = height_to_delta_x_y(reference_height)
delta_x, delta_y = 0,0

offset_setpoint_x = -0.15 + delta_x             # 0.79 (at h = 2.0) # 1.77 (at h = 4.0)
offset_setpoint_y = delta_y                     # 1.41              # 3.15
controller_desired_pose = np.array([offset_setpoint_x, offset_setpoint_y, reference_height, 0.0, 0.0, -math.pi/2])


pid_with_estimate = False

if pid_with_estimate:
    ####################
    #  PID parameters  #
    ####################
    Kp_position_x = 0.15
    Ki_position_x = 0.001
    Kd_position_x = 0.03
    ####################
    Kp_position_y = Kp_position_x
    Ki_position_y = Ki_position_x
    Kd_position_y = Kd_position_x
    ####################
    Kp_position_z = 1.5
    Ki_position_z = 0.01
    Kd_position_z = 0.3
    ####################
    Kp_orientation = 0.0
    Ki_orientation = 0.0
    Kd_orientation = 0.0
    ####################

else:
    ####################
    #  PID parameters  #
    ####################
    Kp_position_x = 1.5
    Ki_position_x = 0.01
    Kd_position_x = 0.3
    ####################
    Kp_position_y = Kp_position_x
    Ki_position_y = Ki_position_x
    Kd_position_y = Kd_position_x
    ####################
    Kp_position_z = 1.5
    Ki_position_z = 0.01
    Kd_position_z = 0.3
    ####################
    Kp_orientation = 0.0
    Ki_orientation = 0.0
    Kd_orientation = 0.0
    ####################

# actuation_saturation = 5 # % of maximum velocity
actuation_saturation = 1 # % of maximum velocity
error_integral_limit = 40
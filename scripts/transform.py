#!/usr/bin/env python3  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Quaternion, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_common_modules import angle_manipulation as ang
from interbotix_common_modules import angle_manipulation as ang
import numpy as np
import time

if __name__ == '__main__':
    bot = InterbotixManipulatorXS("rx200", "arm", "gripper")
    bot.dxl.robot_set_motor_registers("single", "shoulder", "Position_P_Gain", 1500)
    bot.dxl.robot_set_motor_registers("single", "elbow", "Position_P_Gain", 1500)
    bot.dxl.robot_set_motor_registers("single", "wrist_angle", "Position_P_Gain", 1500)

    
    def get_transform(tfBuffer, target_frame, source_frame):
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to look up the transform from '%s' to '%s'." % (target_frame, source_frame))
            return np.identity(4)
        print(trans)
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        quat = trans.transform.rotation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        rpy = euler_from_quaternion(quat_list)
        T_TargetSource = ang.poseToTransformationMatrix([x, y, z, rpy[0], rpy[1], rpy[2]])
        return T_TargetSource


    # def get_transform():
    #     T_TargetSource = ang.poseToTransformationMatrix([0.247, -.11875, .016, 0.247, -.11875, .016])
    #     return T_TargetSource
    
    
    #rospy.init_node('transform')
 #   transtamped = tf2_ros.Buffer()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    #t = get_transform(tfBuffer, 'rx200/board_link','rx200/base_link')
    t = get_transform(tfBuffer, 'rx200/base_link','rx200/board_link')


    # m = np.array([0, 0, .1, 1])
    # print(m)
    # print('')
    # print(t)
    # am = np.matmul(t,m)
    # print('')
    # print(am)
    # bot.arm.go_to_home_pose()
    # bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
    # time.sleep(2)

    # m = np.array([0, .25, .1, 1])
    # am = np.matmul(t,m)
    # bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
    # time.sleep(2)

    # m = np.array([.25, .25, .1, 1])
    # am = np.matmul(t,m)
    # bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
    # time.sleep(2)

    # m = np.array([.25, 0, .1, 1])
    # am = np.matmul(t,m)
    # bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
    # time.sleep(2)

    # m = np.array([0, .125, .1, 1])
    # am = np.matmul(t,m)
    # bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
    # time.sleep(2)


    for i in range(8):
        for j in range(8):
            x = i * .03125 + .03125 
            y = j * .03125 + .03125
            m = np.array([x, y, .1, 1])
            am = np.matmul(t,m)
            print(am)
            bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
            time.sleep(1)

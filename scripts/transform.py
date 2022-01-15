#!/usr/bin/env python3  
import rospy
import math
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped, Quaternion, Point
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from interbotix_common_modules import angle_manipulation as ang
import numpy as np
import time

### @brief Helper function to lookup a transform and convert it into a 4x4 transformation matrix
### @param tfBuffer - tf2_ros buffer instance from which to lookup transforms from the 'tf' tree
### @param target_frame - the frame to which data should be transformed
### @param source_frame - the frame where the data originated
### @return T_TargetSource - desired 4x4 numpy transformation matrix
def get_transform(tfBuffer, target_frame, source_frame):
    try:
        trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to look up the transform from '%s' to '%s'." % (target_frame, source_frame))
        return np.identity(4)
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    z = trans.transform.translation.z
    quat = trans.transform.rotation
    quat_list = [quat.x, quat.y, quat.z, quat.w]
    rpy = euler_from_quaternion(quat_list)
    #rpy="0.006 -.084 0" xyz="0.255 -.125 .02"
    x = 0.252
    y = -0.12
    z = .0875
    rpy = [-0.01, -.07, 0.019] # -.075
    T_TargetSource = ang.poseToTransformationMatrix([x, y, z, rpy[0], rpy[1], rpy[2]])
    return T_TargetSource
    # return np.array([[-5.89262751e-03,  9.99438474e-01,  3.29850597e-02,
    #      7.09145721e+00],
    #    [ 9.90591319e-01,  1.32402451e-03,  1.36846944e-01,
    #      2.24357988e+00],
    #    [ 1.36726428e-01,  3.34811019e-02, -9.90042878e-01,
    #     -3.21620531e+01],
    #    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
    #      1.00000000e+00]])

if __name__ == '__main__':
    bot = InterbotixManipulatorXS("rx200", "arm", "gripper")
    bot.dxl.robot_set_motor_registers("single", "shoulder", "Position_P_Gain", 1500)
    bot.dxl.robot_set_motor_registers("single", "elbow", "Position_P_Gain", 1500)
    bot.dxl.robot_set_motor_registers("single", "wrist_angle", "Position_P_Gain", 1500)
    bot.dxl.robot_torque_enable("group", "arm", True)
    # bot.dxl.robot_reboot_motors("single", "wrist_angle", True)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #trans = TransformStamped()
    trans = get_transform(tfBuffer,'rx200/base_link', 'rx200/board')

    print(trans)
    print('Move Robot')

    bot.arm.go_to_home_pose()
    # time.sleep(1)
    # bot.gripper.close()
    # time.sleep(3)
    # bot.gripper.open()
    # time.sleep(2)
 
    position = np.array([0.015625 + (0.031250 * 0), 0.015625 + (0.031250 * 0) , .05, 1]) # pawn 0.033

    # # position = np.array([0.015625,0.23438 , .078, 1])
    # # position = np.array([0.23438 ,0.015625, .078, 1])

    # #position = np.array([0.23438 ,0.23438, .078, 1])

    am = np.matmul(trans, position)
    print(am)
    bot.arm.set_ee_pose_components(x=am[0] * 1.02, y=am[1] * 1.03, z=am[2])

    # time.sleep(2)
    # position = np.array([0, .25, .03, 1])
    # am = np.matmul(trans, position)
    # bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2] + .05)

    # time.sleep(2)
    # position = np.array([.25, .25, .03, 1])
    # am = np.matmul(trans, position)
    # bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2] + .05)

    # time.sleep(2)
    # position = np.array([.25, 0, .03, 1])
    # am = np.matmul(trans, position)
    # bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2] + .05)


    # bot.arm.set_ee_pose_components(x=am[0] + .03, y=am[1] - .01, z=am[2] + .05)
#    bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2] + .05)

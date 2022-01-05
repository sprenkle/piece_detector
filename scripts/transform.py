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
    T_TargetSource = ang.poseToTransformationMatrix([x, y, z, rpy[0], rpy[1], rpy[2]])
    return T_TargetSource

if __name__ == '__main__':
    bot = InterbotixManipulatorXS("rx200", "arm", "gripper")

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    # trans = TransformStamped()
    trans = get_transform(tfBuffer,'rx200/base_link', 'rx200/board')


    # listener = tf2_ros.TransformListener(tfBuffer)
    # print(rospy.get_published_topics())

    # # rate = rospy.Rate(10.0)
    # # while not rospy.is_shutdown():
    # # try:
    # trans = tfBuffer.lookup_transform('rx200/board','rx200/base_link', rospy.Time())
    #     # break                        rx200/base_link
    # # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     # rate.sleep()
    #     # continue

    print(trans)


    print('Move Robot')

    bot.arm.go_to_home_pose()
    
    position = np.array([0, 0, 0, 1])
    am = np.matmul(trans, position)
    # bot.arm.set_ee_pose_components(x=am[0] + .03, y=am[1] - .01, z=am[2] + .05)
    bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2] + .05)

    time.sleep(2)
    position = np.array([0, .25, 0, 1])
    am = np.matmul(trans, position)
    bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2] + .05)

    time.sleep(2)
    position = np.array([.25, .25, 0, 1])
    am = np.matmul(trans, position)
    bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2] + .05)

    time.sleep(2)
    position = np.array([.25, 0, 0, 1])
    am = np.matmul(trans, position)
    bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2] + .05)


    # bot.arm.set_ee_pose_components(x=am[0] + .03, y=am[1] - .01, z=am[2] + .05)
#    bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2] + .05)

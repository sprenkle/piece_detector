#!/usr/bin/env python3  

from interbotix_xs_modules.arm import InterbotixManipulatorXS
from apriltag_ros.msg import AprilTagDetectionArray

import numpy as np
import time
from pyquaternion import Quaternion
import math
import rospy


bot = InterbotixManipulatorXS("rx200", "arm", "gripper")

bot.dxl.robot_set_motor_registers("single", "shoulder", "Position_P_Gain", 1500)
bot.dxl.robot_set_motor_registers("single", "elbow", "Position_P_Gain", 1500)
bot.dxl.robot_set_motor_registers("single", "wrist_angle", "Position_P_Gain", 1500)

bot.gripper.release()
time.sleep(1)
bot.gripper.attract()
time.sleep(1)
bot.gripper.release()
time.sleep(1)
bot.gripper.attract()
time.sleep(1)
bot.gripper.release()
time.sleep(1)
bot.gripper.attract()
time.sleep(1)
bot.gripper.release()
time.sleep(1)
bot.gripper.attract()
time.sleep(1)
bot.gripper.release()
time.sleep(1)
bot.gripper.attract()
time.sleep(1)


# time.sleep(1)

# m = bot.arm.get_ee_pose()

# cx = m[0,3]
# cy = m[1,3]
# cz = m[2,3]


# cx, cy, cz = adjust_arm(cx, cy, cz, gx, gy, gz)
# time.sleep(2)
# cx, cy, cz = adjust_arm(cx, cy, cz,gx, gy, gz)
# time.sleep(2)
# adjust_arm(cx, cy, cz,gx, gy, gz)


# print('MOved')
# m = bot.arm.get_ee_pose()

# cx = m[0,3]
# cy = m[1,3]
# cz = m[2,3]


# msg = rospy.wait_for_message('tag_detections', AprilTagDetectionArray,1)

# tx = msg.detections[0].pose.pose.pose.position.x
# ty = msg.detections[0].pose.pose.pose.position.y
# tz = msg.detections[0].pose.pose.pose.position.z

# nx = cx + (gx - tx) 
# ny = cy + (gy - ty)
# nz = cz - (gz - tz)

# print(f'Current position from arm {cx}, {cy}, {cz}')
# print(f'Current Tag Location      {tx}, {ty}, {tz}')
# print(f'Goal    Tag Location      {gx}, {gy}, {gz}')
# print(f'New position              {nx}, {ny}, {nz}')
# bot.arm.set_ee_pose_components(x=nx, y=ny, z=nz)

#print(msg)
# rospy.init_node('listener', anonymous=True)
# rospy.Subscriber("tag_detections", AprilTagDetectionArray, callback)
# rospy.spin()

# bot.dxl.robot_set_motor_registers("single", "shoulder", "Position_P_Gain", 1500)
# bot.dxl.robot_set_motor_registers("single", "elbow", "Position_P_Gain", 1500)
# bot.dxl.robot_set_motor_registers("single", "wrist_angle", "Position_P_Gain", 1500)
# bot.dxl.robot_torque_enable("group", "arm", True)
# bot.arm.go_to_home_pose()

# #time.sleep(2)
# #_, success = bot.arm.set_ee_pose_matrix(test, bot.arm.get_joint_commands(), True, 1, 0.3, False)


# # f = open('/home/david/stuff/points.txt', 'w')
# # f.write('')
# # f.close()

# # for i in range(64):
# #     input("Press Enter UN Torque")
# #     bot.dxl.robot_torque_enable("group", "arm", False)
# #     input("Press Enter Torque")
# #     bot.dxl.robot_torque_enable("group", "arm", True)
# #     f = open('/home/david/stuff/points.txt', 'a')
# #     m = repr(bot.arm.get_ee_pose_command()).replace(" ","").replace("\n","")
# #     f.write(f'{i};{m}\n')
# #     f.close() 
# #     print(f'{i};{m}')
#am = move(.125, .125, .028)
# bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
# time.sleep(3)

# am = move(.25, 0, .028)
# bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
# time.sleep(3)

# am = move(.25, .25, .028)
# bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
# time.sleep(3)

# am = move(0, .25, .028)
# bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])

#tag location
#  x: -0.03195832913508378
#  y: -0.04016573881274465
#  z: 0.26726727827055735



# time.sleep(3)
#bot.arm.go_to_home_pose()
#time.sleep(1)

#[ 0.411 -0.138  0.118  1.   ]
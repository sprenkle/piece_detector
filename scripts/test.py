from interbotix_xs_modules.arm import InterbotixManipulatorXS

import numpy as np
import time
import rospy
from game_msgs.msg import ChessPieces
from game_msgs.msg import ChessPiece
from apriltag_ros.msg import AprilTagDetection
from apriltag_ros.msg import AprilTagDetectionArray

def adjust(x, y, z, actual_tag_x, actual_tag_y, actual_tag_z, desired_tag_x, desired_tag_y, desired_tag_z):
    cx = actual_tag_x
    cy = actual_tag_y
    cz = actual_tag_z

    x = (desired_tag_x - cy) + x
    y = (desired_tag_y - cx) + y
    z = (desired_tag_z - cz) + z
    return x, y, z  

def move(fx, fy, fz, tx, ty, tz):
    z=0.215
    bot.arm.go_to_home_pose()
    bot.gripper.open() 
    bot.arm.set_ee_pose_components(x=fx, y=fy, z=z)
    # x=0.5300000000000001 
    # y=-0.11999999999999998 
    z=0.125 #13
    bot.arm.set_ee_pose_components(x=fx, y=fy, z=fz)
    bot.gripper.close() 
    #time.sleep(3)
    # x=0.5300000000000001 
    # y=-0.11999999999999998 
    z=0.215
    bot.arm.set_ee_pose_components(x=fx, y=fy, z=z)
    #bot.gripper.open() 

    x=0.511 
    y=0.125 
    z=0.215

    bot.arm.go_to_home_pose()
    bot.arm.set_ee_pose_components(x=tx, y=ty, z=z)
    # x=0.5300000000000001 
    # y=-0.11999999999999998 
    z=0.125 #13
    bot.arm.set_ee_pose_components(x=tx, y=ty, z=tz)
    bot.gripper.open()
    #time.sleep(3)
    # x=0.5300000000000001 
    # y=-0.11999999999999998 
    z=0.215
    bot.arm.set_ee_pose_components(x=tx, y=ty, z=z)


bot = InterbotixManipulatorXS("rx200", "arm", "gripper")

# x=0.41
# y=0.0131 
# z=.12


# bottom left
#x=0.52 
#y=-0.115 
#z=0.2

# bottom right  0.145
# x=0.511 
# y=0.125 
# z=0.2

# # top left
# x=0.295 
# y=-0.128 
# z=0.2

# # top right
# x=0.29 
# y=0.125
# z=0.2
#chess_msg = rospy.wait_for_message('chess_piece_detector', ChessPieces, timeout=100)

#print(f'from chess_msg x={chess_msg.ChessPieces[0].X} {chess_msg.ChessPieces[0].Y}   actual= .52  -0.115')

#move(.52, -0.115, 0.125, 0.511, 0.125, 0.125)
#move(0.511, 0.125, 0.125, .52, -0.115, 0.125)

#move(.52, -0.115, 0.125, 0.295, -0.128, 0.1075)
#move(0.295, -0.128, 0.1075, .52, -0.115, 0.125)

#move(.52, -0.115, 0.125, 0.285, 0.125, 0.1030)
#move(0.285, 0.125, 0.1030, .52, -0.115, 0.125)

move(.52, -0.115, 0.125, 0.5155, 0.005, 0.125)
move(0.5155, 0.005, 0.125, .52, -0.115, 0.125)



# bot.arm.go_to_home_pose()
# bot.gripper.open() 
# bot.arm.set_ee_pose_components(x=x, y=y, z=z)
# # x=0.5300000000000001 
# # y=-0.11999999999999998 
# z=0.125 #13
# bot.arm.set_ee_pose_components(x=x, y=y, z=z)
# bot.gripper.close() 
# #time.sleep(3)
# # x=0.5300000000000001 
# # y=-0.11999999999999998 
# z=0.215
# bot.arm.set_ee_pose_components(x=x, y=y, z=z)
# #bot.gripper.open() 



# x=0.511 
# y=0.125 
# z=0.2

# bot.arm.go_to_home_pose()
# bot.arm.set_ee_pose_components(x=x, y=y, z=z)
# # x=0.5300000000000001 
# # y=-0.11999999999999998 
# z=0.125 #13
# bot.arm.set_ee_pose_components(x=x, y=y, z=z)
# bot.gripper.open()
# #time.sleep(3)
# # x=0.5300000000000001 
# # y=-0.11999999999999998 
# z=0.215
# bot.arm.set_ee_pose_components(x=x, y=y, z=z)



# goalX=-0.10733944795930432 
# goalY=0.08968576197726681 
# goalZ=0.31001173284890916 



# chess_msg = rospy.wait_for_message('chess_piece_detector', ChessPieces, timeout=10)
# goalX= chess_msg.ChessPieces[0].X 
# goalY= chess_msg.ChessPieces[0].Y
# goalZ=0.31001173284890916 



# # First Set arm to reference point
# bot.arm.set_ee_pose_components(x=x, y=y, z=z)

# print('Made Move')

# #rospy.init_node('test')
# msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout=10)

# print(msg)
# x, y, z = adjust(x, y, z, msg.detections[0].pose.pose.pose.position.x, msg.detections[0].pose.pose.pose.position.y, msg.detections[0].pose.pose.pose.position.z,
#                   goalX, goalY, goalZ              )


# # Move robot to actual point
# bot.arm.set_ee_pose_components(x=x, y=y, z=z)


# #bot.arm.set_ee_pose_components(x=goalX, y=goalY, z=goalZ)
# print(msg.detections[0].pose.pose.pose.position.x, msg.detections[0].pose.pose.pose.position.y, msg.detections[0].pose.pose.pose.position.z)

# time.sleep(1)

# msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout=10)
# #msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout=10)

# goalZ=0.275

# x, y, z = adjust(x, y, z, msg.detections[0].pose.pose.pose.position.x, msg.detections[0].pose.pose.pose.position.y, msg.detections[0].pose.pose.pose.position.z,
#                   goalX, goalY, goalZ              )

# print(f'Final position = {x} {y} {z}')
# # Adjust robot to actual point
# bot.arm.set_ee_pose_components(x=x, y=y, z=.115)

#time.sleep(5)


bot.arm.go_to_sleep_pose()
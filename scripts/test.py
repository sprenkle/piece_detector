from interbotix_xs_modules.arm import InterbotixManipulatorXS
from apriltag_ros.msg import AprilTagDetectionArray

import numpy as np
import time
from pyquaternion import Quaternion
import math
import rospy

x = 0.286 # DON'T Touch
y = -.138 # DON'T Touch
z = 0.09 # DON'T Touch

#pawn hight = 28mm

tr = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[x,y,z,1]])

roll = 0.09 # about x axis
pitch = -0.09 # about y axis  -.09
yaw = -0.05 # about z axis  -.05



def move(x, y, z):
    m = np.array([x,y,z,1])
    rm1 = np.array([
        [1              , 0                 , 0                 , 0],
        [0              ,math.cos(roll)     , -math.sin(roll)   , 0],
        [0              ,math.sin(roll)     , math.cos(roll)    , 0],
        [0              ,0                  , 0                 , 1]])

    rm2 = np.array([
        [math.cos(pitch) ,0                  , math.sin(pitch)   , 0],
        [0               , 1                 , 0                 , 0],
        [-math.sin(pitch), 0                 , math.cos(pitch)  , 0],
        [0               , 0                 , 0                  , 1]])

    rm3 = np.array([
        [math.cos(yaw), -math.sin(yaw) , 0, 0],
        [math.sin(yaw), math.cos(yaw) , 0, 0],
        [0            , 0              , 1, 0],
        [0            , 0              , 0, 1]])

    temp = np.matmul(rm2, rm3)
    temp = np.matmul(rm1, temp)

    am = np.matmul(m, temp)
    #am = np.append(am, 1)
    am = np.matmul(am, tr)
    return am

# cr = np.array([[ -0.0551730,  0.9981529,  0.0254306],
#    [0.9976436,  0.0540687,  0.0422353],
#    [0.0407823,  0.0277009, -0.9987840 ]])

# rm1 = np.array([
#     [math.cos(roll),-math.sin(roll),0],
#     [math.sin(roll),math.cos(roll),0],
#     [0, 0, 1]])

# rm2 = np.array([
#     [math.cos(pitch),0,math.sin(pitch)],
#     [0, 1, 0],
#     [-math.sin(pitch), 0, math.cos(pitch)]])

# rm3 = np.array([
#     [1, 0, 0],
#     [0, math.cos(yaw), -math.sin(yaw)],
#     [0, math.sin(yaw), math.cos(yaw)]])



# yaw = 0.1 # about z axis  
#pitch = 0.11 # about y axis  
#roll = 0.0 # about x axis
# [ 0.28292621 -0.01084604  0.10521256  1.        ]

#tr = np.append(cr,1)
# am2 = np.matmul(m[:3], cr)
# am = np.matmul(am, tr)

#print(f'am2 = {am}')
#q = Quaternion(axis=[0,0,1], angle=0.05) fits it left right
# q = Quaternion(axis=[0,1,0], angle=-0.04)

# am = q.rotate(m[:3])
# print(f'after rotate = {am}')
# am = np.append(am, 1)ls
# am = np.matmul(am, tr)
# print(am)

# [0.295, -.12, 0.11] # 0, 0
# [0.299, -.084, 0.11] # 1, 0

#rospy.init_node('listener22', anonymous=True)


test = np.array([[0.95732931,0.01029463,0.28881589,0.14383422],[-0.01028007,0.99994593,-0.00156732,-0.00154453],[-0.2888164,-0.00146861,0.95738338,0.08559261],[0.,0.,0.,1.]])

bot = InterbotixManipulatorXS("rx200", "arm", "gripper")

bot.dxl.robot_set_motor_registers("single", "shoulder", "Position_P_Gain", 1500)
bot.dxl.robot_set_motor_registers("single", "elbow", "Position_P_Gain", 1500)
bot.dxl.robot_set_motor_registers("single", "wrist_angle", "Position_P_Gain", 1500)



# print(rospy.get_published_topics())
# print("Looking for message")

def adjust_arm(cx, cy, cz, gx, gy, gz):

    msg = rospy.wait_for_message('tag_detections', AprilTagDetectionArray,1)

    tx = msg.detections[0].pose.pose.pose.position.x
    ty = msg.detections[0].pose.pose.pose.position.y
    tz = msg.detections[0].pose.pose.pose.position.z

    nx = cx + (gy - ty) 
    ny = cy + (gx - tx)
    nz = cz - (gz - tz)

    print(f'Current position from arm {cx}, {cy}, {cz}')
    print(f'Current Tag Location      {tx}, {ty}, {tz}')
    print(f'Goal    Tag Location      {gx}, {gy}, {gz}')
    print(f'                          {nx}, {ny}, {nz}')
    bot.arm.set_ee_pose_components(x=nx, y=ny, z=nz)
    return nx, ny, nz




gx = -0.03825733406356586
gy = -0.09190973429704273
gz =  0.2850922070153255

bot.arm.go_to_home_pose()

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
am = move(.125, .125, .028)
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



time.sleep(3)
bot.arm.go_to_home_pose()
#time.sleep(1)

#[ 0.411 -0.138  0.118  1.   ]
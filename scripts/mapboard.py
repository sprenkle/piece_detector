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

def move(x, y, z, roll, pitch, yaw, tr):
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

    temp = np.array([[-0.00928433,  0.86396638,  0.50346389],
       [-0.50885515, -0.43750219,  0.74138942],
       [ 0.86080209, -0.24930689,  0.44369566]])

    # temp = np.array([[-0.00928433, -0.50885515, -0.86080209],
    #    [ 0.86396638, -0.43750219,  0.24930689],
    #    [-0.50346389, -0.74138942,  0.44369566]])

    #am = np.matmul(m, temp)
    #am = np.append(am, 1)
    #am = np.matmul(am, tr)
    am = np.matmul(m, tr)
    
    am = np.matmul(am[:3], temp)
    return am

x = 0.286 # DON'T Touch
y = -.138 # DON'T Touch
z = 0.09 # DON'T Touch

x = 0.1949123792094417
y = 0.44149402513241925
z = -0.17835945650801005

tr = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[x,y,z,1]])

roll = 0.02 # about x axis
pitch = 0.03 # about y axis  -.09
yaw = -0.06 # LEAVE about z axis  -.05

bot.arm.go_to_home_pose()
am = move(.125, .125, .028, roll, pitch, yaw, tr)
#am = move(.125, .125, .028, roll, pitch, yaw, tr)
print(am)
bot.arm.set_ee_pose_components(x=am[0], y=am[1], z=am[2])
# time.sleep(3)


# bot.arm.go_to_home_pose()

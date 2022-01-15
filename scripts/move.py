#!/usr/bin/env python3  

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion, Point
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from interbotix_common_modules import angle_manipulation as ang
import numpy as np
import numpy as np
import time

class ChessBoard:

    def __init__(self):
        self.set_robot_black()
        self.piece_height = {1:0.033, 2:0.035, 3:.023, 4:0.03, 5:0.03, 6:.024}
        self.piece_pickup = {1:1.5, 2:2.5, 3:3.5, 4:4.5, 5:5.5, 6:6.5}
        self.bot = InterbotixManipulatorXS("rx200", "arm", "gripper", gripper_pressure=1)
        self.bot.dxl.robot_set_motor_registers("single", "shoulder", "Position_P_Gain", 1500)
        self.bot.dxl.robot_set_motor_registers("single", "elbow", "Position_P_Gain", 1500)
        self.bot.dxl.robot_set_motor_registers("single", "wrist_angle", "Position_P_Gain", 1500)
        self.bot.dxl.robot_set_motor_registers("single", "gripper", "Position_P_Gain", 1500)
        self.bot.dxl.robot_torque_enable("group", "arm", True)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        self.trans = self.get_transform(tfBuffer,'rx200/base_link', 'rx200/board')
        self.bot.arm.go_to_home_pose()
        self.pickupHeight = .11


    def get_transform(self, tfBuffer, target_frame, source_frame):
        # try:
        #     trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     rospy.logerr("Failed to look up the transform from '%s' to '%s'." % (target_frame, source_frame))
        #     return np.identity(4)
        # x = trans.transform.translation.x
        # y = trans.transform.translation.y
        # z = trans.transform.translation.z
        # quat = trans.transform.rotation
        # quat_list = [quat.x, quat.y, quat.z, quat.w]
        # rpy = euler_from_quaternion(quat_list)
        x = 0.252
        y = -0.12
        z = .0875
        rpy = [-0.01, -.07, 0.019] # -.075
        T_TargetSource = ang.poseToTransformationMatrix([x, y, z, rpy[0], rpy[1], rpy[2]])
        return T_TargetSource


    def set_robot_white(self):
        self.board = np.array([ 
                    [3, 5, 4, 1, 2, 4, 5, 3],
                    [6, 6, 6, 6, 6, 6, 6, 6],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [6, 6, 6, 6, 6, 6, 6, 6],
                    [3, 5, 4, 1, 2, 4, 5, 3],
                ])

    def set_robot_black(self):
        self.board = np.array([ 
                    [3, 5, 4, 2, 1, 4, 5, 3],
                    [6, 6, 6, 6, 6, 6, 6, 6],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0],
                    [6, 6, 6, 6, 6, 6, 6, 6],
                    [3, 5, 4, 2, 1, 4, 5, 3],
                ])

    def set_pose(self, ar):
        #print(ar)
        #print('Start of Move')
        self.bot.arm.set_ee_pose_components(x=ar[0] * 1.02, y=ar[1] * 1.03, z=ar[2], blocking=True)
        #print('End of Move')

    def attract(self):
        #self.bot.gripper.open(delay=2)
        self.bot.gripper.attract()

    def release(self):
        #self.bot.gripper.close()
        self.bot.gripper.release()

    def get_board_coordinates(self, x, y):
        board_x = 0.015625 + (.03125 * x)
        board_y = 0.015625 + (.03125 * y)
        return board_x, board_y


    def pick_up(self, x, y):
        bx, by = self.get_board_coordinates(x, y)
        fromHigh = np.matmul(self.trans, np.array([bx, by, self.pickupHeight, 1]))
        fromPickup = np.matmul(self.trans, np.array([bx, by, self.piece_height[self.board[x, y]], 1]))

        self.attract()
        self.set_pose(fromHigh)
        self.set_pose(fromPickup)
        self.set_pose(fromHigh)


    def put_down(self, x, y):
        bx, by = self.get_board_coordinates(x, y)
        toHigh = np.matmul(self.trans, np.array([bx, by, self.pickupHeight, 1]))
        toPickup = np.matmul(self.trans, np.array([bx, by, self.piece_height[self.board[x, y]]+.005, 1]))
        
        self.set_pose(toHigh)
        self.set_pose(toPickup)
        self.release()
        self.set_pose(toHigh)

    def remove_piece(self, x, y):
        self.pick_up(x, y)
        self.bot.arm.set_ee_pose_components(x=0.3, y=-.2,z=0.25)
        self.release()

    def move(self, fromX, fromY, toX, toY):
        if self.board[toX, toY] != 0:
            self.remove_piece(toX, toY)

        self.pick_up(fromX, fromY)
        self.put_down(toX , toY)

        self.board[toX, toY] = self.board[fromX, fromY] 
        self.board[fromX, fromY] = 0
        #self.bot.arm.go_to_sleep_pose()
    
    def sleep(self):
        self.bot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    print("move.py")
    chessBoard2 = ChessBoard()
    # chessBoard2.move(1,0,6,0)
    # chessBoard2.move(6,0,1,0)
    for i in range(8):
        chessBoard2.move(0, i, 7 ,i)
        chessBoard2.move(1, i, 6 ,i)
    chessBoard2.sleep()
    # chessBoard2.move(0,3,7,3)
    # chessBoard2.move(7,3,0,3)
    #chessBoard2.move(0,4,7,4)
    #chessBoard2.move(7,4,0,4)
    #chessBoard2.bot.arm.set_ee_pose_components(x=0.3, y=-.2,z=0.25)
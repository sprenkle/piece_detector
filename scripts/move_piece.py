#!/usr/bin/env python

from game_msgs.srv import MovePieceCmd, MovePieceCmdResponse
import rospy
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import time


def handle_move_piece(req):
    print(f'from_x {req.from_x} from_y {req.from_y} to_x {req.to_x} to_y {req.to_y} height {req.height}')
    
    bot.arm.go_to_home_pose()
    #bot.arm.set_ee_pose_components(x=req.from_y + .273, y = req.from_x - .1205, z=.12)
    bot.arm.set_ee_pose_components(x=req.from_x, y =req.from_y , z=.11)
    time.sleep(5)
    bot.arm.go_to_sleep_pose()

    return MovePieceCmdResponse(result = 'OK')

def move_piece_server():
    ##rospy.init_node('move_piece')
    s = rospy.Service('move_piece', MovePieceCmd, handle_move_piece)
    print("Ready to move")
    rospy.spin()

if __name__ == "__main__":
    bot = InterbotixManipulatorXS("rx200", "arm", "gripper")

    move_piece_server()




#request = TwoIntsRequest()
#   4 response = TwoIntsResponse()
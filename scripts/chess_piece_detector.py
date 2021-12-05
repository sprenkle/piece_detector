import rospy
from game_msgs.msg import Piecedetected
from game_msgs.msg import Gamefield
from game_msgs.msg import PieceMove
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os 
import sys
from chessbot import board 
from game_msgs.srv import MovePieceCmd, MovePieceCmdRequest
from std_msgs.msg import String
import numpy as np


def imageCallback(data):
    try:
        #chessPieces = ChessPieces()
        current_piece_array = np.full((8,8),-1, np.int32)
        
        if len(data.piece_data) == 0:
            return

        for piece in data.piece_data:
            #print(f'-- {piece.x}, {piece.y}, {piece.piece} --')
            px, py = board.get_position(44.6 , piece.x, piece.y)
            if px != -1 and py != -1 and px < 8 and py < 8:
                current_piece_array[px, py] = piece.piece

        msg = process_move(current_piece_array)
        if msg :
            pub.publish(msg)         

    except CvBridgeError as e:
        print(e)
        return
    except ValueError as e:
        return

def process_move(new_pos):
    # Find Empty board
    from_x = -1
    from_y = -1
    to_x = -1
    to_y = -1

    capture = False

    # Find where it came from
    for i in range(8):
        for j in range(8):
            if old_pos[i, j, 0] > 0 and new_pos[i, j] == -1:
                from_x = i
                from_y = j

    if from_x < 0 or from_y < 0:
        return 

    # Check capture
    for i in range(8):
        for j in range(8):
            if old_pos[i, j, 0] > 0 and new_pos[i, j] > 0 and old_pos[i, j, 1] != new_pos[i, j]:
                to_x = i
                to_y = j
                capture = True

    if not capture:
        # check move
        for i in range(8):
            for j in range(8):
                if old_pos[i, j, 0] == 0 and new_pos[i, j] > -1:
                    to_x = i
                    to_y = j

    if to_x < 0 or to_y < 0:
        return 

    captured = 0
    if capture:
        captured = old_pos[to_x, to_y, 0] 
    
    name = piece_name[old_pos[to_x, to_y, 0]]
    captured_piece_name = piece_name[old_pos[to_x, to_y,0]]

    old_pos[to_x, to_y,0] = old_pos[from_x, from_y,0] 
    old_pos[to_x, to_y,1] = old_pos[from_x, from_y,1] 
    old_pos[from_x, from_y, 0] = 0
    old_pos[from_x, from_y, 1] = 0

    return PieceMove(from_x = from_x, from_y = from_y, to_x = to_x, to_y = to_y, 
                        name=name, color = piece_color[old_pos[to_x, to_y,1]], captured=captured_piece_name)

def talker():
    while not rospy.is_shutdown():
        
        data = rospy.wait_for_message('piece_detector', Gamefield)
        imageCallback(data)


        #pub.publish(hello_str)
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        piece_name = ["Empty", "Pawn", "Knight", "Bishop", "Rook", "Queen", "King"]
        piece_color = ["White", "Black"]

        first_image = False
        old_pos = np.zeros((8,8,2),np.int32)

        # TODO put this in a service 

        old_pos[0,0,0] = 4
        old_pos[1,0,0] = 2
        old_pos[2,0,0] = 3
        old_pos[3,0,0] = 5
        old_pos[4,0,0] = 6
        old_pos[5,0,0] = 3
        old_pos[6,0,0] = 2
        old_pos[7,0,0] = 4
        old_pos[0,0,1] = 1
        old_pos[1,0,1] = 1
        old_pos[2,0,1] = 1
        old_pos[3,0,1] = 1
        old_pos[4,0,1] = 1
        old_pos[5,0,1] = 1
        old_pos[6,0,1] = 1
        old_pos[7,0,1] = 1

        old_pos[0,1,0] = 1
        old_pos[1,1,0] = 1
        old_pos[2,1,0] = 1
        old_pos[3,1,0] = 1
        old_pos[4,1,0] = 1
        old_pos[5,1,0] = 1
        old_pos[6,1,0] = 1
        old_pos[7,1,0] = 1
        old_pos[0,1,1] = 1
        old_pos[1,1,1] = 1
        old_pos[2,1,1] = 1
        old_pos[3,1,1] = 1
        old_pos[4,1,1] = 1
        old_pos[5,1,1] = 1
        old_pos[6,1,1] = 1
        old_pos[7,1,1] = 1

        old_pos[0,6,0] = 1
        old_pos[1,6,0] = 1
        old_pos[2,6,0] = 1
        old_pos[3,6,0] = 1
        old_pos[4,6,0] = 1
        old_pos[5,6,0] = 1
        old_pos[6,6,0] = 1
        old_pos[7,6,0] = 1
        old_pos[0,6,1] = 0
        old_pos[1,6,1] = 0
        old_pos[2,6,1] = 0
        old_pos[3,6,1] = 0
        old_pos[4,6,1] = 0
        old_pos[5,6,1] = 0
        old_pos[6,6,1] = 0
        old_pos[7,6,1] = 0

        old_pos[0,7,0] = 4
        old_pos[1,7,0] = 2
        old_pos[2,7,0] = 3
        old_pos[3,7,0] = 5
        old_pos[4,7,0] = 6
        old_pos[5,7,0] = 3
        old_pos[6,7,0] = 2
        old_pos[7,7,0] = 4
        old_pos[0,7,1] = 0
        old_pos[1,7,1] = 0
        old_pos[2,7,1] = 0
        old_pos[3,7,1] = 0
        old_pos[4,7,1] = 0
        old_pos[5,7,1] = 0
        old_pos[6,7,1] = 0
        old_pos[7,7,1] = 0


        pub = rospy.Publisher('chess_piece_detector', PieceMove, queue_size=10)
        rospy.init_node('chessPieceDetector', anonymous=True)
        rate = rospy.Rate(1)
        board = board.Board(423, 640, 480, 137, 499, 102, 466)
        rospy.sleep(3)
        talker()
    except rospy.ROSInterruptException:
        pass
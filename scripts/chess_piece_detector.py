import rospy
from game_msgs.msg import Whatboard
from game_msgs.msg import Piecedetected
from game_msgs.msg import Gamefield
from game_msgs.msg import ChessPieces
from game_msgs.msg import ChessPiece
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os 
import sys
from chessbot import board 
from chessbot import chess_board 
from game_msgs.srv import MovePieceCmd, MovePieceCmdRequest
from std_msgs.msg import String

#The purpose of this node is to publish chess piece locations, does not validate
def imageCallback(data):
    try:
        chessPieces = ChessPieces()
        for piece in data.piece_data:
            print(piece.x, piece.y, piece.piece)
            px, py = chess.image_to_absolute(44.6 , piece.x, piece.y)
            x, y = chess.get_position_mm(px, py)
            #print(f'X={x} Y={y}')
            ty = 1.0 * x + .35
            tx = 1.0 * y - .232
            #print(f'TX={tx} TY={ty}')
            chessPieces.ChessPieces.append(ChessPiece(X=tx, Y=ty, PixelX=int(px), PixelY=int(py), Height=46.5, Name='King'))

        pub.publish(chessPieces)         
        #print(gamefield)

    except CvBridgeError as e:
        print(e)
        return
    except ValueError as e:
        return



def talker():
    while not rospy.is_shutdown():
        
        data = rospy.wait_for_message('piece_detector', Gamefield)
        imageCallback(data)


        #pub.publish(hello_str)
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('chess_piece_detector', ChessPieces, queue_size=10)
        rospy.init_node('chessPieceDetector', anonymous=True)
        rate = rospy.Rate(1)
        chess = board.Board(423, 320, 240, 1.4558, 83, 445, 100, 464, 8, 8)
        talker()
    except rospy.ROSInterruptException:
        pass
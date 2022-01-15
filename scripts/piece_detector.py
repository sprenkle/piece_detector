#!/usr/bin/env python

import rospy
from game_msgs.msg import Piecedetected
from game_msgs.msg import Gamefield
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os 
import sys
#from chessbot import board 
from chessbot import nn_chess_detector 
#from chessbot import chess_board 
from game_msgs.srv import MovePieceCmd, MovePieceCmdRequest
from std_msgs.msg import String

def imageCallback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

        pieces = piece_finder.list_detected_pieces(cv_image)
        gamefield = Gamefield()
        for piece in pieces:
            gamefield.piece_data.append(Piecedetected(x = piece[2], y = piece[1], piece = piece[0]))
        pub.publish(gamefield)         

    except CvBridgeError as e:
        print(e)
        return
    except ValueError as e:
        return

def talker():
    while not rospy.is_shutdown():
        
        data = rospy.wait_for_message('/camera/color/image_raw', msg_Image, timeout=100)
        imageCallback(data)


        #pub.publish(hello_str)
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        bridge = CvBridge()
        piece_finder = nn_chess_detector.NnChessDetector()
        pub = rospy.Publisher('piece_detector', Gamefield, queue_size=10)
        rospy.init_node('pieceDetector', anonymous=True)
        rate = rospy.Rate(1)
        talker()
    except rospy.ROSInterruptException:
        pass
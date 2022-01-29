#!/usr/bin/env python3  

from tkinter import W
import rospy
from game_msgs.msg import Gamefield
from game_msgs.msg import PieceMove
from sensor_msgs.msg import Image as msg_Image
from game_msgs.msg import ChessMove, GameStatus
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os 
import sys
from chessbot import board 
import numpy as np
from chessbot import fen

blank_image = np.zeros((480, 640,3), np.uint8)

name_dict = {"Pawn":1, "Knight":2, "Bishop":3, "Rook":4, "Queen":5, "King":6}

last_update = []
num_updates = 0
color = "black"
mfen = {}

def image_pos(value):
    return value * 20 + 50

def pieces_detected(data):
    global last_update, num_updates, color
    rospy.loginfo("pieces_detected", logger_name="chess")
    try:
        blank_image = np.zeros((480, 640,3), np.uint8)
        current_piece_array = np.full((8,8),-1, np.int32)

        if len(data.piece_data) == 0:
            return

        for piece in data.piece_data:
            px, py = board.get_position(44.6 , piece.x, piece.y)
            if px != -1 and py != -1 and px < 8 and py < 8:
                current_piece_array[px, py] = piece.piece
                if piece.piece == 0:
                    color_w = (255, 0, 0)
                else:
                    color_w = (0, 255, 0)
                blank_image = cv2.circle(blank_image, (image_pos(px), image_pos(py)), 3, color_w, 2)

        if np.array_equal(last_update, current_piece_array):
            if num_updates < 0:
                num_updates = num_updates + 1
                return
        else:
            last_update = current_piece_array
            num_updates = 0
            return
        
        rospy.loginfo(f'calling process_move', logger_name="chess")
        msg = process_move(current_piece_array)
        rospy.loginfo(f'msg = {msg}', logger_name="chess")

        if msg: # and color == msg.color:
            pub.publish(msg)
        
        cv2.imshow('image',blank_image)
        cv2.waitKey(1)          

    except CvBridgeError as e:
        print(e)
        return
    except ValueError as e:
        return

def diff_square(new_pos, x, y):
    new_value =  new_pos[x, y]
    old_value_rank = old_pos[x, y, 0]
    old_value_color = old_pos[x, y, 1]
    if old_value_rank == 0:
        if new_value != -1:
            print(f'new piece at {x} {y}')
            return True
        return False
    if new_value != old_value_color:
        print(f'new piece at 2 {x} {y}  new_value {new_value}  old_value_color {old_value_color}')
        return True
    return False

def castle_right(new_pos, x, y):
    global old_pos
    #rospy.loginfo(f'{old_pos[x, y, 0]} {old_pos[x+1, y, 0]} {old_pos[x+2, y, 0]} {old_pos[x+3, y, 0]} {new_pos[x, 0]} {new_pos[x + 1, 0]} {new_pos[x + 2, 0]} {new_pos[x + 3, 0]}', logger_name="chess")
    return old_pos[x, y, 0] == 6 and old_pos[x + 1, y, 0] == 0 and old_pos[x + 2, y, 0] == 0 and ( old_pos[x + 3, y, 0] == 4  or ( old_pos[x + 3, y, 0] == 0 and old_pos[x + 4, y, 0] == 4 ) ) and new_pos[x, y] == -1 and new_pos[x + 1, y] > -1 and new_pos[x + 2, y] > -1 and new_pos[x + 3 ,y] == -1 

def castle_left(new_pos, x, y):
    global old_pos
    return old_pos[x, y, 0] == 6 and old_pos[x - 1, y, 0] == 0 and old_pos[x - 2, y, 0] == 0 and ( old_pos[x - 3, y, 0] == 4  or ( old_pos[x - 3, y, 0] == 0 and old_pos[x - 4, y, 0] == 4 ) )and new_pos[x, y] == -1 and new_pos[x - 1, y] > -1 and new_pos[x - 2, y] > -1 and new_pos[x - 3 ,y] == -1 

def process_move(new_pos):
    global old_pos, mfen
    if not mfen:
        return
    # Find Empty board
    from_x = -1
    from_y = -1
    to_x = -1
    to_y = -1
    capture = False

    rospy.loginfo(f'Process_move', logger_name="chess")

    # Black King Side top 
    if castle_right(new_pos, 4,0):
        return PieceMove(from_x = 4, from_y = 0, to_x = 6, to_y = 0, name='King', color = piece_color[old_pos[from_x, from_y,1]])

 
    # Black King Side bottom
    if castle_left(new_pos, 3,7):
        return PieceMove(from_x = 3, from_y = 7, to_x = 1, to_y = 7, name='King', color = piece_color[old_pos[from_x, from_y,1]])
 
    # Black Queen Side top
    if castle_left(new_pos, 4,0):
        return PieceMove(from_x = 4, from_y = 0, to_x = 2, to_y = 0, name='King', color = piece_color[old_pos[from_x, from_y,1]])

    # Black Queen Side bottom
    if castle_right(new_pos, 3,7):
        return PieceMove(from_x = 3, from_y = 7, to_x = 5, to_y = 7, name='King', color = piece_color[old_pos[from_x, from_y,1]])

    # White King Side Top
    if castle_left(new_pos, 3,0):
        return PieceMove(from_x = 3, from_y = 0, to_x = 1, to_y = 0, name='King', color = piece_color[old_pos[from_x, from_y,1]])
 
    # White King Side Bottom
    if castle_right(new_pos, 4,7):
        return PieceMove(from_x = 4, from_y = 7, to_x = 6, to_y = 7, name='King', color = piece_color[old_pos[from_x, from_y,1]])

    # White Queen Side Top
    if castle_right(new_pos, 3,0):
        return PieceMove(from_x = 3, from_y = 0, to_x = 5, to_y = 0, name='King', color = piece_color[old_pos[from_x, from_y,1]])

    # White Queen Side Bottom
    if castle_left(new_pos, 4,7):
        return PieceMove(from_x = 4, from_y = 7, to_x = 2, to_y = 7, name='King', color = piece_color[old_pos[from_x, from_y,1]])


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
            if i == from_x and j == from_y:
                continue
            if diff_square(new_pos, i, j) :
                to_x = i
                to_y = j
                if old_pos[i, j, 0] > 0:
                    capture = True

    if to_x < 0 or to_y < 0:
        return 

    captured = 0
    if capture:
        captured = old_pos[to_x, to_y, 0] 
    
    name = piece_name[old_pos[from_x, from_y, 0]]
    captured_piece_name = piece_name[old_pos[to_x, to_y,0]]

    # old_pos[to_x, to_y,0] = old_pos[from_x, from_y,0] 
    # old_pos[to_x, to_y,1] = old_pos[from_x, from_y,1] 
    # old_pos[from_x, from_y, 0] = 0
    # old_pos[from_x, from_y, 1] = 0

    return PieceMove(from_x = from_x, from_y = from_y, to_x = to_x, to_y = to_y, 
                        name=name, color = piece_color[old_pos[to_x, to_y,1]], captured=captured_piece_name)


def chess_status(data):
    global old_pos, color, mfen

    mfen = fen.Fen(data.board, data.white_player == "human")

    if data.white_player == "human":
        color = "white"
    
    if data.black_player == "human":
        color = "black"

    col = 0
    row = 0
    old_pos = np.zeros((8,8,2),np.int32)
    b = 1
    w = 0
    for i in range(100):
        c = data.board[i:i+1]
        if c == '/':
            row = row + 1
            col = 0
        elif c.isnumeric():
            n = int(c)
            for pos in range(n):
                old_pos[col,row, 0] = 0
                col = col + 1
        elif c == 'p':
            old_pos[col, row, 0] = 1
            old_pos[col, row, 1] = b
            col += 1
        elif c == 'n':
            old_pos[col, row, 0] = 2
            old_pos[col, row, 1] = b
            col += 1
        elif c == 'b':
            old_pos[col, row, 0] = 3
            old_pos[col, row, 1] = b
            col += 1
        elif c == 'r':
            old_pos[col, row, 0] = 4
            old_pos[col, row, 1] = b
            col += 1
        elif c == 'q':
            old_pos[col, row, 0] = 5
            old_pos[col, row, 1] = b
            col += 1
        elif c == 'k':
            old_pos[col, row, 0] = 6
            old_pos[col, row, 1] = b
            col += 1
        elif c == 'P':
            old_pos[col, row, 0] = 1
            old_pos[col, row, 1] = w
            col += 1
        elif c == 'N':
            old_pos[col, row, 0] = 2
            old_pos[col, row, 1] = w
            col += 1
        elif c == 'B':
            old_pos[col, row, 0] = 3
            old_pos[col, row, 1] = w
            col += 1
        elif c == 'R':
            old_pos[col, row, 0] = 4
            old_pos[col, row, 1] = w
            col += 1
        elif c == 'Q':
            old_pos[col, row, 0] = 5
            old_pos[col, row, 1] = w
            col += 1
        elif c == 'K':
            old_pos[col, row, 0] = 6
            col += 1
            old_pos[col, row, 1] = w
        if row >= 7 and col > 7:
            break 



if __name__ == '__main__':
    try:
        piece_name = ["Empty", "Pawn", "Knight", "Bishop", "Rook", "Queen", "King"]
        piece_color = ["white", "black"]

        first_image = False
        old_pos = np.zeros((8,8,2),np.int32)
        print('Made it here 1')
        move = 0
        pub = rospy.Publisher('chess_piece_detector', PieceMove, queue_size=10)
        rospy.init_node('chessPieceDetector', anonymous=True)
        rospy.Subscriber('/chess_status', GameStatus, chess_status)
        rospy.Subscriber('/piece_detector', Gamefield, pieces_detected)

        board = board.Board(423, 640, 480, 93, 435, 103, 445)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass


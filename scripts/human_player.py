#!/usr/bin/env python3  

from curses.ascii import NUL
import rospy
from game_msgs.msg import GameStatus, PieceMove, ChessMove
import sys
from game_msgs.srv import IsValidMove
from chessbot import fen


class HumanPlayer:

    def __init__(self, to_move='white'):
        rospy.loginfo("Init ChessCommand", logger_name="chess")

        self.pub_move = rospy.Publisher(f'human_move', ChessMove, queue_size=10)

        rospy.Subscriber('/chess_status', GameStatus,self.chess_status_callback)
        rospy.Subscriber('/chess_move', ChessMove,self.chess_move_callback)
        
        rospy.Subscriber('/chess_piece_detector', PieceMove, self.make_move)

        rospy.loginfo("Subscribed to chess_start", logger_name="chess")
        self.plays_white = False
        self.plays_black = False
        self.color = ""
        self.your_move = False
        rospy.wait_for_service('is_valid_move')
        self.is_valid_move = rospy.ServiceProxy('is_valid_move', IsValidMove)
        rospy.loginfo("Done Init ChessCommand", logger_name="chess")
        

    def chess_move_callback(self, data):
        self.move_number = data.move_number
        if data.color == self.color:
            self.your_move = False
        else:
            self.your_move = True
        
    def chess_status_callback(self, data):
        rospy.loginfo('callback', logger_name="chess")
        rospy.loginfo(data, logger_name="chess")

        my_fen = fen.Fen(data.board, data.white_player == "human") # TODO This could be wrong if human playing both sides
        self.move_number = my_fen.move_num()
        if (data.white_player == "robot" and my_fen.to_move() == 'white') or (data.black_player == "robot" and my_fen.to_move() == 'black'):
            self.your_move = False
            return

        if my_fen.to_move() == "white":
            self.row_alg_to_num = {0:'a', 1:'b', 2:'c', 3:'d', 4:'e', 5:'f', 6:'g', 7:'h'}
            self.col_alg_to_num = {0:'8', 1:'7', 2:'6', 3:'5', 4:'4', 5:'3', 6:'2', 7:'1'}
        else:
            self.row_alg_to_num = {7:'a', 6:'b', 5:'c', 4:'d', 3:'e', 2:'f', 1:'g', 0:'h'}
            self.col_alg_to_num = {0:'1', 1:'2', 2:'3', 3:'4', 4:'5', 5:'6', 6:'7', 7:'8'}

        self.your_move = True

        rospy.loginfo(f'your move = {self.your_move}', logger_name="chess")


    def make_move(self, data):
        if not self.your_move:
            return

        print(f'{data.from_x} {data.from_y} {data.to_x} {data.to_y}')
        alg_move = self.row_alg_to_num[data.from_x] + self.col_alg_to_num[data.from_y] + self.row_alg_to_num[data.to_x] + self.col_alg_to_num[data.to_y] 
        print(f'{alg_move}  {data.from_x} {data.from_y} {data.to_x} {data.to_y}')

        something = self.is_valid_move(alg_move) 
        print(f'Is it a valid move  {something}')
        if not self.is_valid_move(alg_move):
            print(f'Human move is not valid  {alg_move}')
            return

        chess_move = ChessMove(color = self.color, move = alg_move, move_number = self.move_number, official = True, valid = True)
        self.pub_move.publish(chess_move)



# string color
# string move
# int16  move_number
# bool   official
# bool   valid

def main(args):
    rospy.init_node('human_player', anonymous=True)
    robot_player = HumanPlayer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    rospy.loginfo("Starting Robot", logger_name="chess")
    main(sys.argv)        


#!/usr/bin/env python3  

from curses.ascii import NUL
import rospy
from game_msgs.msg import GameStatus, PlayerStatus, PieceMove, ChessMove
from chessbot import move_board
import sys
from game_msgs.srv import BestMove
from chessbot import fen


class RobotPlayer:

    def __init__(self, to_move='white'):
        self.move_board = move_board.MoveBoard()
        rospy.loginfo("Init ChessCommand", logger_name="chess")
        self.sub = rospy.Subscriber('/chess_status', GameStatus, self.chess_status_callback)
        rospy.loginfo("Subscribed to chess_start", logger_name="chess")
        self.plays_white = False
        self.plays_black = False
        self.color = ""
        self.move_number = 0
        rospy.wait_for_service('best_move')
        self.pub_move = rospy.Publisher(f'/robot_move', ChessMove, queue_size=10)
        
        
    def chess_status_callback(self, data):
        rospy.loginfo('callback', logger_name="chess")
        rospy.loginfo(data.board, logger_name="chess")
        my_fen = fen.Fen(data.board, data.white_player == "human")
        if (data.white_player == "human" and my_fen.to_move() == 'white') or (data.black_player == "human" and my_fen.to_move() == 'black'):
            self.your_move = False
            return
        self.color = my_fen.to_move()
        rospy.loginfo(f'self.move_number = {self.move_number} Fen={my_fen.move_num()}  Color = {self.color}', logger_name="chess")
        if self.move_number < my_fen.move_num():
            self.move_number = my_fen.move_num()
            self.make_move(self.move_number)
        else:
            rospy.loginfo(f'robot not making move {self.move_number}  {my_fen.move_num()}', logger_name="chess")



    def make_move(self, move_number):
        print(f'make_move {self.move_number}  {move_number}')
        if self.move_number >  move_number:
            return
        print(f'moveing Robot  {self.move_number} {move_number}-----------------------------------------------')

        get_move = rospy.ServiceProxy('best_move', BestMove)
        move = get_move("")
        print(f'found move {move}')
        self.move_board.move_alg(move.str)
        chess_move = ChessMove(color = self.color, move = move.str, move_number = self.move_number, official = True, valid = True)
        self.pub_move.publish(chess_move)
        rospy.loginfo(chess_move, logger_name="chess")

# string color
# string move
# int16  move_number
# bool   official
# bool   valid

def main(args):
    rospy.init_node('robot_player', anonymous=True)
    rospy.loginfo("Starting Robot", logger_name="chess")
    robot_player = RobotPlayer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)        


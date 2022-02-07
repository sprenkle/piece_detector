#!/usr/bin/env python3  

import rospy
from game_msgs.msg import GameStart, PlayerStatus, ChessMove, GameStatus
import sys
from stockfish import Stockfish
from game_msgs.srv import GetFEN, BestMove, IsValidMove
#"r2qkb1r/p4ppp/b1p1p3/3pn3/Np6/1B5P/PPPP1PP1/R1BQ1RK1 w kq - 2 12"

class ChessCommand:

    def __init__(self, to_move='white'):
        self.move_number = 0
        self.color_to_move = to_move
        self.pub_move = rospy.Publisher('chess_status', GameStatus, queue_size=10)
        self.sub_black_move = rospy.Subscriber("human_move",ChessMove,self.piece_move_callback)
        self.sub_white_move = rospy.Subscriber("robot_move",ChessMove,self.piece_move_callback)
        self.first_move = False
        rospy.wait_for_service('is_valid_move')
        self.set_fen = rospy.ServiceProxy('set_fen', GetFEN)
        self.get_fen = rospy.ServiceProxy('get_fen', GetFEN)
        self.set_move = rospy.ServiceProxy('set_move', BestMove)
        self.is_valid_move = rospy.ServiceProxy('is_valid_move', IsValidMove)


    def piece_move_callback(self, data):
        rospy.loginfo('----------------------------------------------------piece_move_callback', logger_name="chess")
        self.first_move = True
        if not self.is_valid_move(data.move):
            rospy.loginfo(f'Invalid Move {data.move}', logger_name="chess")
            return
        rospy.loginfo(data.move, logger_name="chess")
        self.set_move(data.move)
        fen = self.get_fen("")
        #rospy.loginfo(f'FEN = {fen.str}', logger_name="chess")
        self.game_status = GameStatus(board = fen.str, white_player = self.play_white, black_player = self.play_black)

    def start(self, robot_color):
        rospy.loginfo('Starting loop', logger_name="chess")
        self.game_status = self.setup_board(robot_color)

        while True: 
            rospy.loginfo(self.game_status, logger_name="chess")
            self.pub_move.publish(self.game_status)
            rospy.sleep(5)         

    def setup_board(self, robot_color):
        game_status = GameStatus()
        game_status.board = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1" # stardard start
        #game_status.board = "r1bqkb1r/ppp2ppp/2N2n2/3p4/3Pp3/6P1/PPP1PPBP/RNBQ1RK1 b kq - 0 7"
        if robot_color == 'white':
            game_status.white_player = "robot"
            game_status.black_player = "human"
            self.play_white = "robot"
            self.play_black = "human"
        else:
            game_status.white_player = "human"
            game_status.black_player = "robot"
            self.play_white = "human"
            self.play_black = "robot"

        return game_status



def main(args):
    rospy.init_node('chess_commander', anonymous=True)
    rospy.loginfo("Init ChessCommand", logger_name="chess")
    chess_command = ChessCommand()
    chess_command.start("black")
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("Shutting down")


if __name__ == '__main__':
    rospy.loginfo("Starting ChessCommand", logger_name="chess")
    main(sys.argv)
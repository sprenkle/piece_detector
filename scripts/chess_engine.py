#!/usr/bin/env python3  

import rospy
from game_msgs.msg import GameStatus, PlayerStatus, ChessMove
import sys
from stockfish import Stockfish
from game_msgs.srv import BestMove, IsValidMove, GetFEN

class ChessEngine:

    def __init__(self, to_move='white'):
        self.engine = Stockfish()
        self.engine.set_skill_level(0)
        rospy.Service('best_move', BestMove, self.best_move)
        rospy.Service('is_valid_move', IsValidMove, self.is_valid_move)
        rospy.Service('get_fen', GetFEN, self.get_FEN)
        #rospy.Service('set_fen', GetFEN, self.set_FEN)
        rospy.Service('set_move', BestMove, self.set_move)
        rospy.Subscriber('/chess_status', GameStatus, self.chess_status_callback)

        self.move_number = 0

    def chess_status_callback(self, data):
        rospy.loginfo(data, logger_name="chess")
        self.engine.set_fen_position(data.board)
        print(self.engine.get_board_visual())

    def set_move(self, data):
        rospy.loginfo(f'set_move {data.str}', logger_name="chess")
        print(self.engine.get_board_visual())
        self.engine.make_moves_from_current_position([data.str])
        print(self.engine.get_board_visual())
        return ""

    def best_move(self, data):
        rv = self.engine.get_best_move()
        rospy.loginfo(f'Best Move = {rv}', logger_name="chess")
        return rv

    def is_valid_move(self, data):
        print(f'is_valid_move {data.str}')
        return self.engine.is_move_correct(data.str)

    def get_FEN(self, data):
        return self.engine.get_fen_position()

    # def set_FEN(self, data):
    #     print(f'set_FEN  {data}')
    #     self.engine.set_fen_position(data)
    #     print(self.engine.get_board_visual())
    #     return "Ok" 

    def chess_move_callback(self, data):
        if data.valid and data.official:
            print(f'Move = {data.move}')
            self.engine.make_moves_from_current_position([data.move])
            print(self.engine.get_board_visual())

    # def piece_move_callback(self, data):
    #     rospy.loginfo(data, logger_name="chess")
    #     if self.engine.is_move_correct(data.move):
    #         data.official = True
    #         data.valid = True
    #         self.engine.make_moves_from_current_position(data.move)
    #     else:
    #         data.official = True
    #         data.valid = False



def main(args):
    rospy.init_node('ChessEngine', anonymous=True)
    rospy.loginfo("Init ChessEngine", logger_name="chess")
    chess_command = ChessEngine()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    rospy.loginfo("Starting ChessEngine", logger_name="chess")
    main(sys.argv)
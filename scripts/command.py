#!/usr/bin/env python3  

import rospy
from game_msgs.msg import GameStart, PlayerStatus, PieceMove
import sys

class ChessCommand:

    def __init__(self, to_move='white'):
        self.move_number = 1
        self.color_to_move = to_move
        self.pub_start = rospy.Publisher('chess_command', GameStart, queue_size=10)
        self.pub_move = rospy.Publisher('chess_move', PlayerStatus, queue_size=10)

        self.sub = rospy.Subscriber("/PieceMove",PieceMove,self.piece_move_callback)
        
    def piece_move_callback(self, data):
        rospy.loginfo(data)

    def start(self):
        game_start = GameStart()
        game_start.white_player = "robot"
        game_start.black_player = "human"
        game_start.board = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        game_start.to_move = "white"
        self.pub_start.publish(game_start)         
        rospy.loginfo(game_start)

        player_status = PlayerStatus()
        self.pub_move.publish(player_status)
        rospy.loginfo(player_status)



def main(args):
    rospy.init_node('chess_commander', anonymous=True)
    chess_command = ChessCommand()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
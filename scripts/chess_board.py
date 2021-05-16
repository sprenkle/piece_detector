class ChessBoard:

    def __init__(self, topic_piece_detector):
        rospy.Subscriber(topic_piece_detector, msg_Image, self.callback)
        self.board = board.Board(423, 320, 240, 1, 83, 445, 100, 464)
        self.starting_position = [Chesspiece('King', 'White', 2, 4)]

    def callback(self, data):
        # check if missing exactly one piece
        # check if exactly one piece moved
        pass


    def is_piece_missing(self, pieces):
        pass

if __name__ == '__main__':
    print('Hello World')

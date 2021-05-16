#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials providedcd
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import tensorflow as tf
from tensorflow import keras
import rospy
from std_msgs.msg import String
from game_msgs.msg import Piecedetected
from game_msgs.msg import Gamefield
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import message_filters
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
import struct
import pyrealsense2 as rs2
import os 
import sys
from chessbot import board 
from chessbot import nn_chess_detector 


class PieceDetector:
    
    def __init__(self, piece_finder, image_topic):
        self.bridge = CvBridge()
        self.model = tf.keras.models.load_model("/home/david/interbotix_ws/src/piece_detector/scripts/loc_to_servos/test1.h5")
        self.scale = 1/256
        self.conf_threshold = 0.5
        self.weights = "/home/david/interbotix_ws/src/piece_detector/scripts/loc_to_servos/yolov3-tiny_last.weights"
        self.cfg = "/home/david/interbotix_ws/src/piece_detector/scripts/loc_to_servos/yolov3-tiny.cfg"
        self.classes = ['Chesspiece']
        self.net = cv2.dnn.readNet(self.weights, self.cfg)

        rospy.Subscriber(image_topic, msg_Image, self.imageCallback)
        self.board = board.Board(423, 320, 240, 1, 83, 445, 100, 464)
        self.pub = rospy.Publisher('piece_detector', Gamefield, queue_size=1)

    def imageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            self.Height = 480
            self.Width = 640
            # print(f'Width={self.Width}  Hieght={self.Height} = {result}')
            
            blob = cv2.dnn.blobFromImage(cv_image, self.scale, (self.Width, self.Height), (0, 0, 0), True, crop=False)
            self.net.setInput(blob)
            outs = self.net.forward(get_output_layers(self.net))
            max_confidence = self.conf_threshold
            card = None
            cards_found = 0
            gamefield = Gamefield()
            for out in outs:

                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[0]

                    if confidence >  self.conf_threshold:
                        center_x = int(detection[0] * self.Width)
                        center_y = int(detection[1] * self.Height)
                        gamefield.piece_data.append(Piecedetected(x = center_x, y = center_y))
            self.pub.publish(gamefield)         


        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

def get_output_layers(net):
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    return output_layers


def main():
    image_topic = '/camera/color/image_raw'
    piece_finder = NnChessDetector()
    listener = PieceDetector(image_topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
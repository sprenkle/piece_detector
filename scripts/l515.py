#!/usr/bin/env python3  

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image as Img
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image

i = 0
avg = np.zeros(5)
root='/home/david'

def convert_depth_image(ros_image):
    bridge = CvBridge()
    global i
    depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
    depth_array = np.array(depth_image, dtype=np.float32)
    im = Image.fromarray(depth_array)
    im = im.convert("L")
    idx = str(i).zfill(4)
    #im.save(root+"/depth/frame{index}.png".format(index = idx))
    
    min_distance = 10000000
    max_distance = 0

    sub = depth_image[101:525,76:370]
    for x in range(sub.shape[0]):
        for y in range(sub.shape[1]):
            if sub[x, y] < min_distance:
                min_distance = sub[x, y]
            if sub[x, y] > max_distance:
                max_distance = sub[x, y]

    avg[i] = max_distance

    i += 1

    if i == 5:
        max_distance = avg.min()
        print(f'{min_distance} {max_distance}')
        i = 0
        if max_distance < 1100:
            print("Danger")
        else:
            print("Clear")

    

def callback(data):
    imgW = 640 * 2
    imgH = 480
    #print(rospy.get_caller_id() + "I heard %s", data.data)
    # imgD = np.reshape(data.data(1:2:end),imgW,imgH))
    # imgC = np.reshape(data.data(2:2:end),imgW,imgH))
    # print(len(data.data))
    # arry = np.array(data.data)
    # imgD = np.reshape(arry, (imgW, imgH))
    # print(imgD.shape)
    convert_depth_image(data)

    exit()
    #imgCombined = imgCx2^8 + imgD;
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/camera/depth/image_rect_raw", Img, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Startup")
    listener()
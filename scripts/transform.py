#!/usr/bin/env python3  
import rospy

import math
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('rx200/base_link', 'board', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        print(trans)

        # msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        # msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)


        rate.sleep()


# header: 
#   seq: 0
#   stamp: 
#     secs: 0
#     nsecs:         0
#   frame_id: "board"
# child_frame_id: "rx200/base_link"
# transform: 
#   translation: 
#     x: 0.1949123792094417
#     y: 0.44149402513241925
#     z: -0.17835945650801005
#   rotation: 
#     x: -0.49922668471591575
#     y: 0.49611545599441786
#     z: 0.17894585863451237
#     w: -0.6874740266920893

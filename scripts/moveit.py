import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()
print('Here')
group_name = "interbotix_arm"
group = moveit_commander.MoveGroupCommander(group_name)
print('Here 2')

# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
#                                                moveit_msgs.msg.DisplayTrajectory,
#                                                queue_size=20)

# # We can get the name of the reference frame for this robot:
# planning_frame = group.get_planning_frame()
# print ("============ Reference frame: %s" % planning_frame)

# # We can also print the name of the end-effector link for this group:
# eef_link = group.get_end_effector_link()
# print ("============ End effector: %s" % eef_link)

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print ("============ Robot Groups:", robot.get_group_names())

# # Sometimes for debugging it is useful to print the entire state of the
# # robot:
# print ("============ Printing robot state")
# print (robot.get_current_state())
# print ("")

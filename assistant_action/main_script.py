#!/usr/bin/env python

import rospkg
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

rospy.init_node('the_web_server_node', disable_signals=True)
rospy.loginfo('Inside web_server script')
rospack = rospkg.RosPack()
rate = rospy.Rate(10.0)



def move_base(x, y, client):
    print("base is moving")
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()
rospy.loginfo("Before movebase")
coords = {'x': -2.16564114295, 'y': -5.48124138424}

result = move_base(coords['x'], coords['y'], client)
rospy.loginfo(result)

#!/usr/bin/env python
import rospkg
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


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

def _move_base_init():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    return client


if __name__ == "__main__":
    client = _move_base_init()
    coords = {'x': 0.6564114295, 'y': 1.48124138424}
    result = move_base(coords['x'], coords['y'], client)
    rospy.loginfo(result)
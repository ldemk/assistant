import argparse

# parser = argparse.ArgumentParser(description='generate qr-code')
# parser.add_argument('--host', metavar='h', type=str, default=["localhost"], nargs=1)
# parser.add_argument('--port', metavar='p', type=int, default=[8080], nargs=1)
# parser.add_argument('--filename', metavar='f', type=str, default=["qr_code.png"], nargs=1)
# args = parser.parse_args()
# x = args.x[0]


import rospkg
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

rospy.init_node('the_web_server_node', disable_signals=True)
rospy.loginfo('Inside web_server script')
rospack = rospkg.RosPack()
rate = rospy.Rate(10.0)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()


def move_base(x, y, client):
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

coords = {'x': -2.16564114295, 'y': -5.48124138424}

result = move_base(coords['x'], coords['y'], client)
rospy.loginfo(result)

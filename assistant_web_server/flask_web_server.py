from flask import Flask, render_template, request, url_for
from auth import auth_error
import os
import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


app = Flask(__name__)
app.jinja_env.auto_reload = True
app.config['TEMPLATES_AUTO_RELOAD'] = True
app.secret_key = "super&key*secret"

rospy.init_node('the_web_server_node', anonymous=True)
rospy.loginfo('Inside web_server script')
rospack = rospkg.RosPack()
rate = rospy.Rate(10.0)

client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
client.wait_for_server()

def move_base(x, y):
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


@app.route('/', methods=["GET", "POST"])
def start():
    return render_template("main.html")


@app.route('/login', methods=["GET", "POST"])
def login():

    if request.method == "POST":
        if not auth_error(request.form.get("user_name"), request.form.get("password")):
            succ_msg = request.form.get("user_name")
            return render_template("search.html")
        else:
            fail_msg = "Wrong passwod or userame, try again."
            return render_template('main.html')

    init_msg = "Welcome!"
    return render_template('main.html')


@app.route('/search', methods=["GET", "POST"])
def search():
    if request.method == "POST":
        if request.form.get("person"):
            params = {'x': 3.72576482438, 'y': -5.62893755925}
            result = move_base(params['x'], params['y'])
            rospy.loginfo(result)
            return render_template("assistant.html")

    return render_template("search.html")


# @app.route('/assistant', methods=["GET", "POST"])
# def search():
#     params = {'x': 1.0, 'y': 1.0}
#     result = move_base(params['x'], params['y'])
#     rospy.loginfo(result)
#     result_msg = "Successful*"


if __name__ == '__main__':
    app.run(debug = True)



    #ToDo:
    # move base
    # db
    # cookie username,


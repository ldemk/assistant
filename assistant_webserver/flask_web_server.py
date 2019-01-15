#!/usr/bin/env python

from flask import Flask, render_template, request
from auth import auth_error, get_coords
import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


app = Flask(__name__)
app.jinja_env.auto_reload = True
# app.config['TEMPLATES_AUTO_RELOAD'] = False
app.secret_key = "super&key*secret"

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

@app.route('/', methods=["GET", "POST"])
def start():
    if request.method == "POST":
        if not auth_error(request.form.get("user_name"), request.form.get("password")):
            # succ_msg = request.form.get("user_name")
            return render_template("search.html")
        else:
            # fail_msg = "Wrong passwod or userame, try again."
            return render_template('main.html')
    # init_msg = "Welcome!"
    return render_template('main.html')


@app.route('/search', methods=["GET", "POST"])
def search():
    if request.method == "POST":
        worker = request.form.get("person")
        if worker:
            execfile('main_script.py')
            return render_template("assistant.html")

    return render_template("search.html")

app.run(debug=True, use_reloader=False)
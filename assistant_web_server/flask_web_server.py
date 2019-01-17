#!/usr/bin/env python

from flask import Flask, render_template, request, send_from_directory
from auth import auth_error, get_coords
import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


app = Flask(__name__, static_folder='templates')
app.jinja_env.auto_reload = True
# app.config['TEMPLATES_AUTO_RELOAD'] = False
app.secret_key = "super&key*secret"


@app.route('/', methods=["GET", "POST"])
def start():
    if request.method == "POST":
        if not auth_error(request.form.get("user_name"), request.form.get("password")):
            return  send_from_directory(app.static_folder, "search.html")
            # return render_template("search.html")
        else:
            return send_from_directory(app.static_folder, "main.html")
            # return render_template('main.html')

    return send_from_directory(app.static_folder, "main.html")
    # return render_template('main.html')


@app.route('/search', methods=["GET", "POST"])
def search():
    if request.method == "POST":
        worker = request.form.get("person")
        if worker:
            execfile('main_script.py')
            return send_from_directory(app.static_folder, "assistant.html")
            # return render_template("assistant.html")

    return send_from_directory(app.static_folder, "search.html")
    # return render_template("search.html")



app.run(debug=True, use_reloader=False)

#ToDo:
# add custom messages into templates
    # succ_msg = request.form.get("user_name")
    # fail_msg = "Wrong passwod or userame, try again."
    # init_msg = "Welcome!"



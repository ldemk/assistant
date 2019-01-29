#!/usr/bin/env python
from flask import Flask, render_template, request, send_from_directory
import rospy
import rospkg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_script import move_base, _move_base_init
from communication_db import Database
from auth import auth_error, get_coords
print("START")

# Configurations & Settings

app = Flask(__name__, static_folder='../templates')
app.jinja_env.auto_reload = True
app.secret_key = "super&key*secret"
app.config['TEMPLATES_AUTO_RELOAD'] = False

my_email = 'lazorenko@ucu.edu.ua'         # only for registered users
my_id = 'Q4LaAPTNL4cHNsZJO24IoTvlh2I2'
db = Database(my_email, my_id)   # Instance of database-wrappig class for communication with the firebase.

rospy.loginfo("move_base_init")
rospy.init_node('web_server_node')   # disable_signals=True
rospy.loginfo("web_server_node initialized")
rospack = rospkg.RosPack()
rate = rospy.Rate(10.0)


# Main Web-App

@app.route('/', methods=["GET", "POST"])
def start():
    if request.method == "POST":
        if not auth_error(request.form.get("user_name"), request.form.get("password")):
            return  send_from_directory(app.static_folder, "search.html")
        else:
            return send_from_directory(app.static_folder, "main.html")
    return send_from_directory(app.static_folder, "main.html")


@app.route('/search', methods=["GET", "POST"])
def search():
    if request.method == "POST":
        worker = request.form.get("person")
        if worker:
            first_name, last_name = tuple(worker.split(' '))     # e.g.: ("Branden", "Ciesla")
            if db.employee_is_available(first_name, last_name):
                x_coord, y_coord = db.coordinate_x(first_name, last_name), db.coordinate_y(first_name, last_name)
                client = _move_base_init()
                client.wait_for_server()
                move_base(x_coord * 8, y_coord * 20, client)
                return send_from_directory(app.static_folder, "assistant.html")
            else:
                return send_from_directory(app.static_folder, "busy.html")
    return send_from_directory(app.static_folder, "search.html")


app.run(debug=True,
        use_reloader=False)



#ToDo:
# add custom messages into templates
    # succ_msg = request.form.get("user_name")
    # fail_msg = "Wrong passwod or userame, try again."
    # init_msg = "Welcome!"

# parametrise execfile('move_script.py')

# subprocess.call(['./abc.py', arg1, arg2])
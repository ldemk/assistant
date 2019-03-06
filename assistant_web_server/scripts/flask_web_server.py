#!/usr/bin/env python
from flask import Flask, request, send_from_directory
import rospy
import rospkg
from roslaunch.loader import rosparam

from move_script import move_base, move_base_init
from assistant_database.db_employees import EmployeesDB
from assistant_database import config
from auth import auth_check
rospy.loginfo("START")


#  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Configurations & Settings ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#  ~~~~~~~~~~~~~FLASK APP~~~~~~~~~~~~~~
app = Flask(__name__, static_folder='../templates')
app.jinja_env.auto_reload = True
app.secret_key = "super&key*secret"
app.config['TEMPLATES_AUTO_RELOAD'] = False

#  ~~~~~~~~~~~~~FIREBASE~~~~~~~~~~~~~~
db = EmployeesDB(config.my_email, config.my_key)   # Instance of database-wrappig class for communication with the firebase.

#  ~~~~~~~~~~~~~WEB-SERVER-NODE~~~~~~~~~~~~~~
rospy.init_node('web_server_node')  # Initialize a node for the web server.
rospy.loginfo("web_server_node initialized")

rospack = rospkg.RosPack()
rate = rospy.Rate(10.0)
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


@app.route('/', methods=["GET", "POST"])        # Log in for more secure using.
def start():
    if request.method == "POST":
        try:
            if not auth_check(request.form.get("user_name"), request.form.get("password")):
                return send_from_directory(app.static_folder, "search.html")
        except ():
            return send_from_directory(app.static_folder, "main.html")
    return send_from_directory(app.static_folder, "main.html")


@app.route('/search', methods=["GET", "POST"])   # Search for worker, get coords, start Assistant.
def search():
    if request.method == "POST":
        worker = request.form.get("person")
        if worker:
            first_name, last_name = tuple(worker.split(' '))     # e.g.: ("Branden", "Ciesla")
            if db.employee_is_available(first_name, last_name):

                # change goal tolerance params to
                if rospy.has_param('yaw_goal_tolerance') and rospy.has_param('xy_goal_tolerance'):
                    rosparam.set_param('yaw_goal_tolerance', 3.14)
                    rosparam.set_parem('xy_goal_tolerance', 0.55)

                x_coord, y_coord = db.coordinate_x(first_name, last_name), db.coordinate_y(first_name, last_name)
                client = move_base_init()
                client.wait_for_server()
                move_base(x_coord, y_coord, client)
                return send_from_directory(app.static_folder, "ask_next.html")
            else:
                return send_from_directory(app.static_folder, "busy.html")
    return send_from_directory(app.static_folder, "search.html")


@app.route('/end', methods=["GET", "POST"])     # End session, send Assistant home.
def end():
    x_home_coord, y__home_coord = db.coordinate_x("Home", "Divanchiki"), db.coordinate_y("Home", "Divanchiki")
    if rospy.has_param('yaw_goal_tolerance') and rospy.has_param('xy_goal_tolerance'):
        rosparam.set_param('yaw_goal_tolerance', 0.3)
        rosparam.set_parem('xy_goal_tolerance', 0.05)
    client = move_base_init()
    client.wait_for_server()
    move_base(x_home_coord, y__home_coord, client)
    return send_from_directory(app.static_folder, "end.html")


app.run(use_reloader=False,
        debug=True)


# ToDo:
# add custom messages into templates
# succ_msg = request.form.get("user_name")
# fail_msg = "Wrong passwod or userame, try again."
# init_msg = "Welcome!"

# ? subprocess.call(['./abc.py', arg1, arg2])

# rospy.init_node('web_server_node', disable_signals=True)

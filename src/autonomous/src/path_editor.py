#! /usr/bin/env python

from flask import Flask, render_template, url_for, request, redirect
from datetime import datetime
import json
import threading
import rospy
import tf
from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular
from path import AutoGoal, AutoPath, Autons

app = Flask(__name__)


# This is the main data structure which holds the data and will be uploaded to a json file for saving
data = []

# Current auto selected data (Note: Paths and goals are done in JavaScript)
selected_auto_data = None
selected_auto_index = None

# This checks if you are in the planner url with no auton selected
is_alert = False

# For the robot pose
x = None
y = None
th = None

# Home Page
@app.route('/')
def index():
    global is_alert
    if (is_alert):
        alert = True
        is_alert = False
    else:
        alert = False

    return render_template('home.html', is_alert=alert)

# Selecting Auton Page
@app.route('/select')
def select():
    rev_data = list(reversed(data))
    return render_template('select_auto.html', autons=rev_data)

# Gets the id of the selected auto
@app.route('/select/auto/<int:id>')
def select_auto(id):
    global selected_auto_data
    global selected_auto_index

    selected_auto_data = data[id]
    selected_auto_index = id
    return redirect('/planner')

# Deletes the auton with the id from the data
@app.route('/select/delete/<int:id>')
def select_delete(id):
    for d in data:
        if d.id > id:
            d.id -= 1
    del data[id]
    write_json()
    return redirect('/select')

# Update Page for the selected auton
@app.route('/select/update/<int:id>', methods=['GET','POST'])
def select_update(id):
    if request.method == 'POST':
        data[id].title = request.form['title_content']
        if data[id].title == "":
            data[id].title = "N/A"
        data[id].num_balls = request.form['num_ball_content']
        if data[id].num_balls == "":
            data[id].num_balls = "N/A"
        data[id].start_pose = request.form['start_pose_content']
        if data[id].start_pose == "":
            data[id].start_pose = "N/A"
        data[id].description = request.form['description_content']
        if data[id].description == "":
            data[id].description = "N/A"
        write_json()
        return redirect('/select')

    return render_template('select_update.html', auton=data[id])

# Create New Auton Page
@app.route('/create')
def create():
    return render_template('create_auto.html')

# Submits the contents of the create page and updates the data
@app.route('/create/submit', methods=['POST', 'GET'])
def create_submit():
    global selected_auto_data
    global selected_auto_index

    title_content = request.form['title_content']
    if title_content == "":
        title_content = "N/A"
    num_ball_content = request.form['num_ball_content']
    if num_ball_content == "":
        num_ball_content = "N/A"
    start_pose_content = request.form['start_pose_content']
    if start_pose_content == "":
        start_pose_content = "N/A"
    description_content = request.form['description_content']
    if description_content == "":
        description_content = "N/A"
    data.append(Autons(len(data), title=title_content, num_balls=num_ball_content, start_pose=start_pose_content, description=description_content))
    write_json()

    selected_auto_data = data[len(data)-1]
    selected_auto_index = len(data)-1

    return redirect('/planner')

# The main page of the path editor
@app.route('/planner')
def planner():
    global selected_auto_data
    global selected_auto_index
    global is_alert

    if (selected_auto_data == None or selected_auto_index == None):
        is_alert = True
        return redirect('/')
    
    return render_template("planner.html", auton=selected_auto_data)

# API Json communication with JavaScript
@app.route('/planner/api/auton', methods=['POST', 'GET'])
def api_auton():
    global selected_auto_data
    global selected_auto_index
    global is_alert

    if request.method == 'GET':
        if (selected_auto_data == None or selected_auto_index == None):
            d = {}
        else:
            d = selected_auto_data.serialize_json()
        
        return d
    
    if request.method == 'POST':
        if (not selected_auto_data == None or not selected_auto_index == None):
            selected_auto_data.deserialize_json(request.get_json())
            write_json()
        return ('', 204)

# API Json communication with JavaScript
@app.route('/planner/api/robot_pose', methods=['POST', 'GET'])
def api_robot_pose():
    global x
    global y
    global th

    # Updates the robot pose
    if request.method == 'GET':
        d = {"x":x, "y":y, "th":th}
        return d
    
    # New Robot Pose
    if request.method == 'POST':
        new_pose = request.get_json()
        return ('', 204)
    
def write_json():
    global data

    with open('data.txt', 'w') as outfile:
        a = []
        for auto in data:
            b = auto.serialize_json()
            a.append(b)
        json.dump(a, outfile, indent=4)

def read_json():
    global data
    with open('data.txt') as json_file:
        json_data = json.load(json_file)
        
        for d in json_data:
            a = Autons(len(data))
            a.deserialize_json(d)
            data.append(a)

def robot_pose_callback(msg):
    global x
    global y
    global th

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    z = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w

    quat = [0,0,z,w]
    euler = tf.transformations.euler_from_quaternion(quat)

    th = euler[2]

if __name__ == "__main__":
    threading.Thread(target=lambda: rospy.init_node('path_editor', disable_signals=True)).start()
    rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, robot_pose_callback)
    read_json()
    app.run(debug=True)
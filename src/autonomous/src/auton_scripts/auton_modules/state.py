from datetime import datetime
import json
import rospy
from std_msgs.msg import Float32, Float64, Bool, String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular
from path import AutoGoal, AutoPath, Autons
import time
import rospkg 

global data
data = []

def read_json():
    """ This reads the auton data and saves it to a list to be used """
    rospack = rospkg.RosPack()
    file_path = rospack.get_path('autonomous') + "/src/data.txt"
    with open(file_path) as json_file:
        json_data = json.load(json_file)
        
        new_data = []
        for d in json_data:
            a = Autons(len(new_data))
            a.deserialize_json(d)
            new_data.append(a)

        global data
        data = new_data

class State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.initialize()
        self.action_executed = False

        self.start_time = time.time()

    def log_state(self):
        """ Logs the name of the State """
        rospy.loginfo("STATE: %s   [%s]" %(self.__class__.__name__, 15 - self.ros_node.get_time()))
    
    # Counts the amount of time 
    def start_timer(self):
        """ Used to start a timer that counts time within a state """
        self.start_time = time.time()

    def check_timer(self, time):
        """ Checks if the amount of time given has passed """
        if time.time() - self.start_time >= time:
            return True
        return False
    
    # Checks if you passed the waypoint
    def passed_waypoint(self, waypoint_num):
        """ Checks if you passed a given waypoint in a path. Starts at 1 """
        bools = self.ros_node.get_data('/diff_drive/waypoints_achieved', simple_data = False)
        # Waits for the data
        if bools is not None:
            if len(bools.bools) >= waypoint_num:
                return bools.bools[waypoint_num -1]
            
            rospy.logerr_throttle(15, "Checking Waypoint Failed. Did not find a waypoint with the number '%s' in the path" %(waypoint_num))
            return False
        else:
            return False

    # This runs in the child class when created
    def initialize(self):
        pass

    # This runs once in the child class
    def execute_action(self):
        pass

    # This runs in a loop in the child class
    def tick(self):
        pass

    # This makes it so that the functions created in the child class act as they should
    def update(self):
        if not self.action_executed:
            self.execute_action()
            self.action_executed = True
        return self.tick()

class StartPath(State):

    # Conditions
    def started_path(self):
        """ This checks if the robot has recieved the path """
        if self.ros_node.get_data('/diff_drive/path_achieved') is None:
            return False
        return not self.ros_node.get_data('/diff_drive/path_achieved')
    
    # Actions
    def publish_path(self, name):
        """ This gets the path data from the json file and publishes to diff_drive """
        # Checks for updated data
        read_json()

        global data

        # Loops through the specified auton and checks for the path name
        for auton in data:
            if (auton.title == self.ros_node.auton_title):
                for path in auton.paths:
                    if path.name == name:
                        a = []
                        for goal in path.goals:
                            a.append(goal.get_goal())
                        self.ros_node.publish('/diff_drive/goal_path', GoalPath, path.get_path(a), latching = True)
                        rospy.loginfo("Published Path, with the name '%s'", name)
                        return
                rospy.logerr("Publishing Path Failed. Did not find a path named '%s' in the data file", name)
                return
        rospy.logerr("Publishing Path Failed. Did not find auton titled '%s' in the data file", self.ros_node.auton_title)

# This is ROBOT SPECIFIC
class Intake(State):

    # Actions
    def deploy_intake(self):
        """ This publishes a msg to deploy the intake """
        intake_state = String()
        intake_state.data = "deploy"

        self.ros_node.publish("/auto/intake/state", String, intake_state, latching = True)

    def retract_intake(self):
        """ This publishes a msg to retract the intake """
        intake_state = String()
        intake_state.data = "retract"

        self.ros_node.publish("/auto/intake/state", String, intake_state, latching = True)

class Shooter(State):

    # This puts the shooter in idle mode and allows other sub systems to do specific functions
    def idle(self):
        """ This puts the shooter in idle mode """
        shooter_state = String()
        shooter_state.data = "idle"

        self.ros_node.publish("/auto/shooter/state", String, shooter_state, latching = True)
    
    # Overrides the other states because it needs to control all three subsystems 
    def start_prime(self):
        """ This starts the turret tracking, adjusting rpm, and hood angle """
        shooter_state = String()
        shooter_state.data = "prime"

        self.ros_node.publish("/auto/shooter/state", String, shooter_state, latching = True)

    def start_shoot(self):
        """ This makes the turret begin to shoot """
        shooter_state = String()
        shooter_state.data = "shoot"

        self.ros_node.publish("/auto/shooter/state", String, shooter_state, latching = True)

class Turret(Shooter):

    # Conditions
    def reached_angle(self, angle):
        """ Checks if the turret has reached the given angle """
        if self.ros_node.get_data("/auto/turret/current/angle") == angle:
            return True
        return False

    # Actions (Only works if Shooter is in idle)
    def idle_turret(self):
        """ This puts the turret into idle mode """
        turret_state = String()
        turret_state.data = "idle"

        self.ros_node.publish("/auto/turret/state", String, turret_state, latching = True)

    def rotate_turret(self, angle):
        """ This rotates the turret to a specified angle """
        turret_state = String()
        turret_state.data = "rotate_turret"

        turret_angle = Float32()
        turret_angle.data = angle

        self.ros_node.publish("/auto/turret/state", String, turret_state, latching = True)
        self.ros_node.publish("/auto/turret/wanted/angle", Float32, turret_angle, latching = True)

class Flywheel(Shooter):

    # Conditions
    def reached_rpm(self, rpm):
        """ Checks if the fly wheel has reached the wanted rpm """
        if self.ros_node.get_data("/auto/flywheel/current/rpm") == rpm:
            return True
        return False

    # Actions (Only works if Shooter is in idle)
    def idle_flywheel(self):
        """ This puts the shooter into idle mode """
        flywheel_state = String()
        flywheel_state.data = "idle"

        self.ros_node.publish("/auto/flywheel/state", String, flywheel_state, latching = True)

    def start_spin_up(self, rpm):
        """ This starts the robot's spin up to a specific rpm """
        flywheel_state = String()
        flywheel_state.data = "spin_up"

        flywheel_rpm = Float32()
        flywheel_rpm.data = rpm

        self.ros_node.publish("/auto/flywheel/state", String, flywheel_state, latching = True)
        self.ros_node.publish("/auto/flywheel/wanted/rpm", Float32, flywheel_rpm, latching = True)

class Hood(Shooter):

    # Conditions
    def reached_angle(self, angle):
        """ Checks if the turret has reached the given angle """
        if self.ros_node.get_data("/auto/hood/current/angle") == angle:
            return True
        return False

    # Actions (Only works if Shooter is in idle)
    def idle_hood(self):
        """ This puts the hood into idle mode """
        hood_state = String()
        hood_state.data = "idle"

        self.ros_node.publish("/auto/hood/state", String, hood_state, latching = True)

    def rotate_hood(self, angle):
        """ This adjusts the hood to the given angle """
        hood_state = String()
        hood_state.data = "rotate_hood"

        hood_angle = Float32()
        hood_angle.data = angle

        self.ros_node.publish("/auto/hood/state", String, hood_state, latching = True)
        self.ros_node.publish("/auto/hood/wanted/angle", Float32, hood_angle, latching = True)
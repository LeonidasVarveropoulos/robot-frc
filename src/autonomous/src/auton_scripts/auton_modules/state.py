from datetime import datetime
import json
import rospy
from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular
from path import AutoGoal, AutoPath, Autons
import time
import rospkg 

global data
data = []

def read_json():
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
        """
        Returns the name of the State.
        """
        rospy.loginfo("STATE: %s   [%s]" %(self.__class__.__name__, 15 - self.ros_node.get_time()))
    
    # Counts the amount of time 
    def start_timer(self):
        self.start_time = time.time()

    def check_timer(self, time):
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

    # This runs in a loop
    def tick(self):
        pass

    def update(self):
        if not self.action_executed:
            self.execute_action()
            self.action_executed = True
        return self.tick()

class StartAuton(State):
    """
    We define a state object which provides some utility functions 
    for starting the auton
    """

class StartPath(State):

    # Conditions
    def started_path(self):
        if self.ros_node.get_data('/diff_drive/path_achieved') is None:
            return False
        return not self.ros_node.get_data('/diff_drive/path_achieved')
    
    # Actions
    def publish_path(self, name):
        # Checks for updated data for testing
        read_json()

        global data

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

class Intake(State):

    # Actions
    def deploy_intake(self):
        pass

    def retract_intake(self):
        pass

class Turret(State):

    # Actions
    def rotate(self, angle):
        pass

    def start_spin_up(self, rpm):
        pass

    def stop_spin_up(self):
        pass

    def start_tracking(self):
        pass

    def stop_tracking(self):
        pass

    def start_shoot(self):
        pass

    def stop_shoot(self):
        pass


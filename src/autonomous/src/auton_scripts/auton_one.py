
import rospy
from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Twist
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
import time
from auton_modules.path import AutoPath, AutoGoal
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular, BoolArray

from auton_modules.state import State, StartAuton, StartPath

# The id of the auton, used for picking auton
auton_id = 2
auton_title = "Auton 1"

# Start of our states
class Idle(StartAuton):
    """
    The state which waits for autonomous to start
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        pass

    def tick(self):
        return StartFirstPath(self.ros_node)


class StartFirstPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.publish_path("First Path")

    def tick(self):
        if self.started_path():
            return PassedSecondWaypoint(self.ros_node)

        return self

class PassedSecondWaypoint(State):
    """
    The state which waits for the second waypoint of the path.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        pass

    def tick(self):
        if self.passed_waypoint(2):
            return Final(self.ros_node)

        return self

class Final(State):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        pass

    def tick(self):
        return self

class Shutdown(State):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def initialize(self):
        pass

    def execute_action(self):
        pass

    def tick(self):
        return self

# End of our states.
def start(ros_node):
    # Pick which topics to subscribe to
    ros_node.subscribe("/diff_drive_go_to_goal/distance_to_goal", Float32)
    ros_node.subscribe("/diff_drive/waypoints_achieved", BoolArray)
    ros_node.subscribe("/diff_drive/path_achieved", Bool)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown
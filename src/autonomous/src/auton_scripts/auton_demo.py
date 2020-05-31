import rospy
from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Twist
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
import time
from auton_modules.path import AutoPath, AutoGoal
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular, BoolArray

from auton_modules.state import State, StartIdle, StartPath, Intake, Shooter, Turret, Hood, Flywheel

# The id of the auton, used for picking auton
auton_id = 1
auton_title = "Auton Demo"

# Start of our states
class Idle(StartIdle):
    """
    The state which waits for autonomous to start
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.setRobotPose()

    def tick(self):
        return StartFirstPath(self.ros_node)


class StartFirstPath(StartPath):
    """
    The state which publishes the first path to follow
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.publish_path("Path 1")

    def tick(self):
        if self.started_path():
            return DeployIntake(self.ros_node)

        return self

class DeployIntake(Intake):
    """
    The state which waits for the second waypoint of the path.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.deploy_intake()

    def tick(self):
        if self.passed_waypoint(2):
            return Prime(self.ros_node)

        return self

class Prime(Shooter):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_prime()

    def tick(self):
        if self.ros_node.get_data("/diff_drive/path_achieved"):
            return StartShoot(self.ros_node)
        return self

class StartShoot(Shooter):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def initialize(self):
        self.log_state()

    def execute_action(self):
        self.start_shoot()
        self.start_timer()

    def tick(self):
        if self.check_timer(3.0):
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
        rospy.loginfo("END OF AUTON")

    def tick(self):
        return self

class Shutdown(Shooter):
    """
    The state which indicates that there are no limitations on device
    capabilities.
    """

    def initialize(self):
        pass

    def execute_action(self):
        self.idle()

    def tick(self):
        return self

def start(ros_node):
    # Pick which topics to subscribe to
    ros_node.subscribe("/diff_drive_go_to_goal/distance_to_goal", Float32)
    ros_node.subscribe("/diff_drive/waypoints_achieved", BoolArray)
    ros_node.subscribe("/diff_drive/path_achieved", Bool)

    ros_node.subscribe("/auto/turret/current/angle", Float32)
    ros_node.subscribe("/auto/flywheel/current/rpm", Float32)
    ros_node.subscribe("/auto/hood/current/angle", Float32)

    # Return the wanted Start and Shutdown state
    return Idle, Shutdown
#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64, Bool, Float32MultiArray
from geometry_msgs.msg import Twist
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry, Path
import time
from auton_scripts.auton_modules.path import AutoPath, AutoGoal
from diff_drive.msg import Goal, GoalPath, Constants, Linear, Angular, BoolArray
import os
from auton_scripts.auton_modules.state_machine import StateMachine
import rospkg
import importlib
import auton_scripts.auton_modules.state as state

# Imports all autons
rospack = rospkg.RosPack()
path = rospack.get_path('autonomous') + "/src/auton_scripts"
file_root = rospy.get_param('file_root', "auton_")
autons = []
for i in os.listdir(path):
    if file_root in i and i[len(i)-3: len(i)] == ".py":
        autons.append(importlib.import_module("auton_scripts." + i[0: len(i) -3]))

class ROSNode:

    def __init__(self):
        # Create ROS node
        rospy.init_node('autonomous')
        self.rate = float(rospy.get_param('rate', 50))
        self.auto_state_topic = rospy.get_param('auto_state_topic', "/auto/state")
        self.auto_select_topic = rospy.get_param('auto_select_topic', "/auto/select")

        self._data = {}
        self._publishers = {}
        self._subscribers = []

        self.state_machine = None
        self.auton_title = None

        # Needed for determining when to start auton
        self.subscribe(self.auto_state_topic, Bool)
        self.subscribe(self.auto_select_topic, Float32)

        # Used for timing events
        self.start_time = time.time()
        self.over_time = False
    
    # Counts the amount of time since the beginning of autonomous
    def start_timer(self):
        """ Starts the timer, called at the beginning of auton """
        self.start_time = time.time()
        self.over_time = False

    def get_time(self):
        """ Return the amount of time passed since the start of auton """
        passed_time = time.time() - self.start_time
        if passed_time >= 15 and not self.over_time:
            rospy.logwarn("15 seconds have passed since the start of auton")
            self.over_time = True
        return passed_time

    def stop_diff_drive(self):
        msg = Bool()
        msg.data = True
        self.publish("/stop_diff_drive", Bool, msg, latching=True)

    def subscribe(self, topic_name, data_type):
        """ This sets up the ros subscribers for incoming data """
        # Checks if subscriber exists, if not create one
        if topic_name in self._subscribers:
            return
        rospy.Subscriber(topic_name, data_type, self._on_new_data)
        self._subscribers.append(topic_name)

    def publish(self, topic_name, data_type, data, queue = 10, latching = False):
        """ This publishes ros data """
        # Check if publisher exists, if not create and publish data
        if not topic_name in self._publishers:
            self._publishers[topic_name] = rospy.Publisher(topic_name, data_type, queue_size=queue, latch=latching)
        self._publishers[topic_name].publish(data)

    def get_data(self, topic_name, simple_data = True):
        """ This gets the subscribed ros data """
        # Get the data from the dict if it exists
        if topic_name in self._data:
            if simple_data:
                return self._data[topic_name].data
            else:
                return self._data[topic_name]
        return None

    def _on_new_data(self, msg):
        """ This is the callback function for the subscribers """
        self._data[str(msg._connection_header["topic"])] = msg

    # Main loop
    def main(self):
        """ This is the main loop for autonomous """
        rospy.loginfo("{0} started".format(rospy.get_name()))

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():   
            # State machine is running and auton disabled
            if self.state_machine is not None and not self.get_data(self.auto_state_topic):
                rospy.logwarn("State Machine Reset")
                self.state_machine.shutdown()
                self.stop_diff_drive()
                self.state_machine = None

            # State machine was not running and auton was enabled
            elif self.state_machine is None and self.get_data(self.auto_state_topic):
                rospy.loginfo_throttle(10, "Waiting for auton selector")
                for auton in autons:
                    if (self.get_data(self.auto_select_topic) == auton.auton_id):
                        self.start_timer()
                        self.state_machine = StateMachine()
                        start_state, shutdown_state = auton.start(self)
                        self.state_machine.set_initial(start_state(self))
                        self.state_machine.set_shutdown(shutdown_state(self))
                        self.state_machine.start()

                        self.auton_title = auton.auton_title
                        break
            
            # State machine was not running and auton is disabled (Sets starting pose)
            elif self.state_machine is None and not self.get_data(self.auto_state_topic):
                rospy.loginfo_throttle(10, "Waiting for auto mode")
                for auton in autons:
                    if (self.get_data(self.auto_select_topic) == auton.auton_id):
                        msg = Float32MultiArray()
                        for auto in state.data:
                            if auto.title == auton.auton_title:
                                state.read_json()
                                msg.data = auto.start_pose
                                self.publish('/robot_set_pose', Float32MultiArray, msg, latching = True)
                                rospy.loginfo_throttle(10, "Reset Robot Pose")
                                break

            # Ticker state machine if one then sleep until next loop
            if self.state_machine is not None:
                self.state_machine.tick()
                self.get_time()
            rate.sleep()


if __name__ == '__main__':
    ros_node = ROSNode()
    ros_node.main()





    

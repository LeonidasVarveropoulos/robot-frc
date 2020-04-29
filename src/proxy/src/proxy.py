#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Bool, String
import threading
from geometry_msgs.msg import Twist

# Creates proxy node
rospy.init_node('proxy')

# For simulating the robot's server
if not rospy.get_param("~sim_server", False):
    from networktables import NetworkTables
else:
    from sim_server import SimServer
    NetworkTables = SimServer()

cond = threading.Condition()
notified = [False]

# Checks if connected to networktables
def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

if not rospy.get_param("~sim_server", False):
    server_ip = rospy.get_param('~server_ip', "10.06.24.2")

    NetworkTables.initialize(server=server_ip)
    NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

    with cond:
        print("Waiting")
        if not notified[0]:
            cond.wait()
else:
    rospy.loginfo("Simulation Server Started!")

class Proxy:

    def __init__(self):
        # Runs code below if connected to the server
        rospy.loginfo("NetworkTables Connected!")

        table_name = rospy.get_param('~table_name', "SmartDashboard")
        self.table = NetworkTables.getTable(table_name)

        self.update_rate = rospy.get_param('~rate', 15)

        # A Dictionary of wanted topic name and data type
        self.input_data = rospy.get_param('~input_data', [])
        self.output_data = rospy.get_param('~output_data', [])

        # Collection of ROS subscribers and actual data
        self._data = {}

        # List of ROS publishers and subscribers to avoid calling one twice
        self._publishers = {}
        self._subscribers = []

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

        if topic_name in self._data:
            if simple_data:
                return self._data[topic_name].data
            else:
                return self._data[topic_name]
        return None

    def _on_new_data(self, msg):
        """ This is the callback function for the subscribers """
        self._data[str(msg._connection_header["topic"])] = msg

    def main(self):
        # Set how many times this should run per second
        r = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():

            # Input
            for data in self.input_data:
                if data["type"] == "number":
                    input_data = self.table.getNumber(data["name"], data["default"])
                    self.publish(data["name"], Float32, input_data)

                elif data["type"] == "boolean":
                    input_data = self.table.getBoolean(data["name"], data["default"])
                    self.publish(data["name"], Bool, input_data)
                    
                elif data["type"] == "string":
                    input_data = self.table.getString(data["name"], data["default"])
                    self.publish(data["name"], String, input_data)

                else:
                    rospy.logerr("Proxy could not find data type of " + data["type"])

            # Output
            for data in self.output_data:
                # Data from the subscribers
                new_data = self.get_data(data["name"])

                if data["type"] == "number":
                    self.subscribe(data["name"], Float32)
                    if new_data != None:
                        self.table.putNumber(data["name"], new_data)
                    else:
                        self.table.putNumber(data["name"], data["default"])

                elif data["type"] == "boolean":
                    self.subscribe(data["name"], Bool)
                    if new_data != None:
                        self.table.putBoolean(data["name"], new_data)
                    else:
                        self.table.putBoolean(data["name"], data["default"])

                elif data["type"] == "string":
                    self.subscribe(data["name"], String)
                    if new_data != None:
                        self.table.putBoolean(data["name"], new_data)
                    else:
                        self.table.putBoolean(data["name"], data["default"])

                else:
                    rospy.logerr("Proxy could not find data type of " + data["type"])

            # Displays simulated networktables data in sim_data.txt
            if rospy.get_param("~sim_server", False):
                self.table.display_data()

            # Sleeps to meet specified rate
            r.sleep()

proxy_node = Proxy()
proxy_node.main()

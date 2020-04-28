import rospy
import rospkg
import json

class SimServer:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path('proxy') + "/src/sim_data.txt"

        self.prev_data = {}
        self.data = {"/auto/state":True, "/auto/select": rospy.get_param("/auto/select", 0.0)}

    def getTable(self, table_name):
        return self

    # Input
    def getBoolean(self, name, default):
        if name in self.data:
            return self.data[name]
        return default

    def getNumber(self, name, default):
        if name in self.data:
            return self.data[name]
        return default

    def getString(self, name, default):
        if name in self.data:
            return self.data[name]
        return default

    # Output
    def putBoolean(self, name, value):
        self.data[name] = value

    def putNumber(self, name, value):
        self.data[name] = value

    def putString(self, name, value):
        self.data[name] = value

    def display_data(self):
        with open(self.file_path, 'w') as outfile:
            json.dump(self.data, outfile, indent=4)

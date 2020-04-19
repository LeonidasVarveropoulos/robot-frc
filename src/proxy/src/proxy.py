#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32, Float64, Bool
import threading
from networktables import NetworkTables
from geometry_msgs.msg import Twist

# Creates proxy node
rospy.init_node('proxy')

cond = threading.Condition()
notified = [False]

# Checks if connected to networktables
def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server='10.06.24.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

class Proxy:

    def __init__(self):
        # Runs code below if connected to the server
        print("Connected!")
        self.table = NetworkTables.getTable('SmartDashboard')

        self.update_rate = 15
        
        # AUTON from roboRIO
        self.auto_enabled_pub = rospy.Publisher("auto/state", Bool, queue_size=50)
        self.auto_selector_pub = rospy.Publisher("auto/select", Float32, queue_size=50)

        # AUTON to roboRIO
        self.left_vel = 0.0
        self.right_vel = 0.0

        # AUTON and TELEOP to roboRIO
        self.turret_vel = 0.0
        self.turret_distance = 0.0
        #self.turret_tar_loc = False
        #self.turret_rot_ready = False

        # AUTON to roboRIO
        self.turret_state = 0.0
        self.intake_state = 0.0

        self.move_state = 0

    def left_vel_callback(self,msg): 
        self.left_vel = msg.data

    def right_vel_callback(self,msg): 
        self.right_vel = msg.data

    def turret_vel_callback(self,msg): 
        self.turret_vel = msg.data
    
    def turret_distance_callback(self,msg): 
        self.turret_distance = msg.data

    def turret_state_callback(self,msg): 
        self.turret_state = msg.data
    
    def intake_state_callback(self,msg): 
        self.intake_state = msg.data
	
    def move_state_callback(self,msg):
        self.move_state = msg.data
    """
    def turret_tar_loc_callback(self,msg): 
            self.turret_tar_loc.data = msg.data
        
    def turret_rot_ready_callback(self,msg): 
        self.turret_rot_ready.data = msg.data
    """

    def main(self):
         # Done under pathfinding
        left_vel_sub = rospy.Subscriber('cmd_vel/left', Float64, self.left_vel_callback)
        right_vel_sub = rospy.Subscriber('cmd_vel/right', Float64, self.right_vel_callback)

        # Done by turret rotation
        turret_vel_sub = rospy.Subscriber('turret/cmd_vel', Float32, self.turret_vel_callback)
        turret_distance_sub = rospy.Subscriber('turret/distance', Float32, self.turret_distance_callback)

        # Done by turret rotation
        #turret_tar_loc_sub = rospy.Subscriber('turret/target_loc', Bool, self.turret_tar_loc_callback)
        #turret_rot_ready_sub = rospy.Subscriber('turret/rot_ready', Bool, self.turret_rot_ready_callback)

        # Done by autonomous
        turret_state_sub = rospy.Subscriber('auto/turret/state', Float32, self.turret_state_callback)
        intake_state_sub = rospy.Subscriber('auto/intake/state', Float32, self.intake_state_callback)
        move_base_sub = rospy.Subscriber("auto/move/state", Float32, self.move_state_callback)
        # Set how many times this should run per second
        r = rospy.Rate(self.update_rate)

        while not rospy.is_shutdown():
            # Output

            # Sends Turret aiming info to the roboRio (rpm and meters)

            # Sends Turret state booleans
            #self.table.putBoolean('Target Locked?', self.turret_tar_loc.data)
            #self.table.putBoolean('Rotation Ready?', self.turret_rot_ready.data)
		
	    # Input
            auto_enabled = Bool()
            auto_selector = Float32()

            # Autonomous modes
            auto_enabled.data = self.table.getBoolean('Autonomous Enabled?', False)
            auto_selector.data = self.table.getNumber('Auton Selector', 0)

            # Sends State modes for autonomous
            if (self.turret_state == 0 and auto_enabled.data):
                print("Sending Turret Idle")
                self.table.putBoolean('Shoot Prime Close?', False)
                self.table.putBoolean('Shoot Prime Far?', False)
                self.table.putBoolean('Shoot Shoot?', False)
                self.table.putBoolean("Spin Up?", False)
                #self.table.putBoolean('Frame Rotation?', False)
                self.table.putNumber('Shooter Turn Velocity',self.turret_vel)
                self.table.putNumber('Dist From Target', self.turret_distance)
            elif (self.turret_state == 1 and auto_enabled.data):
                print("Sending Turret Prime Close")
                self.table.putBoolean('Shoot Prime Close?', True)
                self.table.putBoolean('Shoot Prime Far?', False)
                self.table.putBoolean('Shoot Shoot?', False)
                self.table.putBoolean("Spin Up?", False)
                #self.table.putBoolean('Frame Rotation?', False)
                self.table.putNumber('Shooter Turn Velocity',self.turret_vel)
                self.table.putNumber('Dist From Target', self.turret_distance)
            elif (self.turret_state == 2 and auto_enabled.data):
                print("Sending Turret Prime Far")
                self.table.putBoolean('Shoot Prime Close?', False)
                self.table.putBoolean('Shoot Prime Far?', True)
                self.table.putBoolean('Shoot Shoot?', False)
                self.table.putBoolean("Spin Up?", False)
                #self.table.putBoolean('Frame Rotation?', False)
                self.table.putNumber('Shooter Turn Velocity',self.turret_vel)
                self.table.putNumber('Dist From Target', self.turret_distance)
            elif (self.turret_state == 3 and auto_enabled.data):
                print("Sending Turret Shoot Close")
                self.table.putBoolean('Shoot Prime Close?', True)
                self.table.putBoolean('Shoot Prime Far?', False)
                self.table.putBoolean('Shoot Shoot?', True)
                self.table.putBoolean("Spin Up?", False)
                #self.table.putBoolean('Frame Rotation?', False)
                self.table.putNumber('Shooter Turn Velocity',self.turret_vel)
                self.table.putNumber('Dist From Target', self.turret_distance)
            elif (self.turret_state == 4 and auto_enabled.data):
                print("Sending Turret Shoot Far")
                self.table.putBoolean('Shoot Prime Close?', False)
                self.table.putBoolean('Shoot Prime Far?', True)
                self.table.putBoolean('Shoot Shoot?', True)
                self.table.putBoolean("Spin Up?", False)
                #self.table.putBoolean('Frame Rotation?', False)
                self.table.putNumber('Shooter Turn Velocity',self.turret_vel)
                self.table.putNumber('Dist From Target', self.turret_distance)
            elif (self.turret_state == 5 and auto_enabled.data):
                print("Sending Turret rot out of frame")
                self.table.putBoolean('Shoot Prime Close?', True)
                self.table.putBoolean('Shoot Prime Far?', False)
                self.table.putBoolean('Shoot Shoot?', False)
                self.table.putBoolean("Spin Up?", False)
                #self.table.putBoolean('Frame Rotation?', False)
                self.table.putNumber('Shooter Turn Velocity',-10)
                self.table.putNumber('Dist From Target', 10)
            elif (self.turret_state == 5.5 and auto_enabled.data):
                print("Sending Turret Spin Up and out of frame")
                self.table.putBoolean('Shoot Prime Close?', False)
                self.table.putBoolean('Shoot Prime Far?', False)
                self.table.putBoolean('Shoot Shoot?', False)
                self.table.putBoolean("Spin Up?", True)
                #self.table.putBoolean('Frame Rotation?', False)
                self.table.putNumber('Shooter Turn Velocity',-10)
                self.table.putNumber('Dist From Target', 10)
            elif (self.turret_state == 10 and auto_enabled.data):
                print("Sending Turret spin up ignore")
                self.table.putBoolean('Shoot Prime Close?', False)
                self.table.putBoolean('Shoot Prime Far?', False)
                self.table.putBoolean('Shoot Shoot?', False)
                self.table.putBoolean("Spin Up?", True)
                #self.table.putBoolean('Frame Rotation?', True)
                self.table.putNumber('Shooter Turn Velocity', 0)
                self.table.putNumber('Dist From Target', 10)
            elif (self.turret_state == 12 and auto_enabled.data):
                print("Sending Turret rot out of frame")
                self.table.putBoolean('Shoot Prime Close?', True)
                self.table.putBoolean('Shoot Prime Far?', False)
                self.table.putBoolean('Shoot Shoot?', False)
                self.table.putBoolean("Spin Up?", False)
                #self.table.putBoolean('Frame Rotation?', False)
                self.table.putNumber('Shooter Turn Velocity',10)
                self.table.putNumber('Dist From Target', 10)
            else:
                self.table.putNumber('Shooter Turn Velocity',self.turret_vel)
                self.table.putNumber('Dist From Target', self.turret_distance)

            if (self.intake_state == 0):
                print("Sending Intake Idle")
                self.table.putBoolean('Intake Action?', False)
            elif (self.intake_state == 1):
                print("Sending Intake Action")
                self.table.putBoolean('Intake Action?', True)

            print("Getting Data: ",auto_enabled.data,auto_selector.data)

            if (auto_selector.data == 0):
                self.table.putNumber('Left DT Setpoint', 0)
                self.table.putNumber('Right DT Setpoint', 0)
                self.table.putBoolean('Intake Action?', False)
                self.table.putNumber('Shooter Turn Velocity', 0)
                self.table.putNumber('Dist From Target', 0)
                self.table.putBoolean('Shoot Prime Far?', False)
                self.table.putBoolean('Shoot Prime Close?', False)
                self.table.putBoolean("Spin Up?", False)
                self.table.putBoolean('Shoot Shoot?', False)
            elif (self.move_state == 0):
                self.table.putNumber('Left DT Setpoint', 0)
                self.table.putNumber('Right DT Setpoint', 0)
            else:
                self.table.putNumber('Left DT Setpoint', self.left_vel)
                self.table.putNumber('Right DT Setpoint', self.right_vel)

            # Publish data to ROS
            self.auto_enabled_pub.publish(auto_enabled)
            self.auto_selector_pub.publish(auto_selector)

            # Sleeps to meet specified rate
            r.sleep()

proxy_node = Proxy()
proxy_node.main()

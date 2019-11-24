#!/usr/bin/python
#ROS imports

import rospy
from osuf1_common.msg import MPC_metadata, MPC_trajectory, MPC_prediction
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#local imports
from nl_dynamics import F1Dynamics
import simulator

#python lib imports
from functools import partial
import math
import time

class MPC_Sim:
    def __init__(self):

        rospy.init_node("mpc_sim_node")
        
        self.nlDynamics = F1Dynamics()
        self.stepFunc = partial(self.nlDynamics.frontStep, self.nlDynamics)
        self.dt = rospy.get_param("dt", .1)
        self.totalTime = rospy.get_param("total_time", 1)

        self.currentState = [0, 0, 0]
        self.inputFunc = lambda et, t : [ 4, -1 * math.cos(2*t+et)/4]

#        self.simulator = simulator.ModelSimulator(self.dt, self.totalTime, self.currentState, self.stepFunc, self.inputFunc, True)

        self.simulation_period = 100 #ms between publish events
        self.prediction_pub_topic = rospy.get_param("mpc_prediction_topic", "mpc_prediction")
        self.meta_pub_topic = rospy.get_param("mpc_metadata_topic", "mpc_metadata")
        self.position_topic = rospy.get_param("localization_topic", "pf_pose")

        self.prediction_pub = rospy.Publisher(self.prediction_pub_topic, MPC_trajectory, queue_size=1)
        self.meta_pub = rospy.Publisher(self.meta_pub_topic, MPC_metadata, queue_size=1)
        self.position_sub = rospy.Subscriber(self.position_topic, PoseStamped, self.parseState)

        self.elapsedTime = 0
        

    def start(self):
        rate = rospy.Rate(1000/self.simulation_period)
        time.sleep(2)
        while not rospy.is_shutdown():
            results = self.simulate()
            self.currentState = results[0][0][:-1]

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()

            #metadata message
            meta = MPC_metadata()
            meta.header = header
            meta.dt = self.dt
            meta.horizon = self.totalTime
            self.meta_pub.publish(meta)

            #simulation results message
            trajectory = MPC_trajectory()
            trajectory.header = header
            trajectory.trajectory = [MPC_prediction(pred[0], pred[1]) for pred in results]
            self.prediction_pub.publish(trajectory)
            rospy.loginfo("Published trajectory")

            # wait remainder to attain desired 'simulation_period'
            rate.sleep()

    def parseState(self, data):
        qt = data.pose.orientation
        _, _, yaw = euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])
        self.currentState = [
            data.pose.position.x,
            data.pose.position.y,
            yaw
        ]

    def simulate(self):
        etInputFunc = partial(self.inputFunc, self.elapsedTime)
        sim = simulator.ModelSimulator(self.dt, self.totalTime, 
            self.currentState, self.stepFunc, etInputFunc, True)
        predictions = sim.simulate()
        return predictions

if __name__ == "__main__":
    sim = MPC_Sim()
    sim.start()

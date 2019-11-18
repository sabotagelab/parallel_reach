
#ROS imports
import rospy
from osuf1_common import MPC_metadata, MPC_trajectory, MPC_prediction
import std_msgs.msg
from geometry_msgs import Pose
from tf.transformation import euler_from_quaternion

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
        self.totalTime = rospy.get_param("total_time", 2)

        self.currentState = [0, 0, 0]
        self.inputFunc = lambda t : [ 6, -1 * math.cos(2*t)/4]

        self.simulator = simulator.ModelSimulator(self.dt, self.totalTime, self.currentState, self.stepFunc, self.inputFunc, True)

        self.simulation_period = 100 #ms between publish events
        self.prediction_pub_topic = rospy.get_param("mpc_prediction_topic", "mpc_prediction")
        self.meta_pub_topic = rospy.get_param("mpc_metadata_topic", "mpc_metadata")
        self.position_topic = rospy.get_param("localization_topic", "pf_pose")

        self.prediction_pub = rospy.Publisher(self.prediction_pub_topic, MPC_trajectory, queue_size=1)
        self.meta_pub = rospy.Publisher(self.meta_pub_topic, MPC_metadata, queue_size=1)
        self.position_sub = rospy.Subscriber(self.position_topic, Pose, self.parseState)
        

    def start(self):
        rate = rospy.rate(1000/self.simulation_period)
        while not rospy.is_shutdown():
            rospy.spinOnce()

            results = self.simulate()

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()

            #metadata message
            meta = MPC_metadata()
            meta.header = header
            meta.dt = self.dt
            meta.horizon = self.totalTime
            self.meta_pub.Publish(meta)

            #simulation results message
            trajectory = MPC_trajectory()
            trajectory.header = header
            trajectory.trajectory = MPC_trajectory([MPC_prediction(pred[0], pred[1]) for pred in results])
            self.prediction_pub.Publish(trajectory)

            # wait remainder to attain desired 'simulation_period'
            rate.sleep()

    def parseState(self, data):
        qt = data.orientation
        _, _, yaw = euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])
        self.initalState = [
            data.position.x,
            data.position.y,
            yaw
        ]

    def simulate(self):
        sim = simulator.ModelSimulator(self.dt, self.totalTime, 
            self.currentState, self.stepFunc, self.inputFunc, True)
        predictions = sim.simulate()
        return predictions
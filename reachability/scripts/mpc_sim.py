
#ROS imports
import rospy
from osuf1_common import MPC_metadata, MPC_trajectory, MPC_prediction
import std_msgs.msg

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

        self.simulation_period = 100 #ms between publish events
        self.prediction_pub_topic = rospy.get_param("mpc_prediction_topic", "mpc_prediction")
        self.meta_pub_topic = rospy.get_param("mpc_metadata_topic", "mpc_metadata")

        self.prediction_pub = rospy.Publisher(self.prediction_pub_topic, MPC_trajectory, queue_size=1)
        self.meta_pub = rospy.Publisher(self.meta_pub_topic, MPC_metadata, queue_size=1)
        
        self.currentState = [0, 0, 0]
        self.inputFunc = lambda t : [ 6, -1 * math.cos(2*t)/4]

    def start(self):
        current_ms = lambda: int(round(time.time() * 1000))
        while not rospy.is_shutdown():
            frameStart_ms = current_ms()
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

            #compute time to simulate and publish
            # wait remainder to attain desired 'simulation_period'
            frameEnd_ms = current_ms()
            frametime = frameEnd_ms - frameStart_ms
            if frametime > 0:
                time.sleep(frametime * .001)

    def simulate(self):
        sim = simulator.ModelSimulator(self.dt, self.totalTime, 
            self.currentState, self.stepFunc, self.inputFunc, True)
        predictions = sim.simulate()
        return predictions
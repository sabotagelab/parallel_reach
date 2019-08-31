''' Created by Nathan, 8/27/2019 '''

import rospy
from nav_msgs.msg import Path
from osuf1_common.msg import StampedFloat2d, MPC_metadata
from F1Hylaa import F1Hylaa

class Reachability:
    def __init__(self):
        rospy.init_node('reachability_node', anonymous=True)

        #define parameters
        self.run_frequency = rospy.get_param("run_frequency", 10)
        self.runtime_limitation = rospy.get_param("runtime_limitation", .5)
        self.hylaa = F1Hylaa()


        #define subscribers
        trajectory_topic = rospy.get_param("trajectory_topic")
        inputs_topic = rospy.get_param("inputs_topic")
        metadata_topic = rospy.get_param("metadata_topic")
        rospy.Subscriber(trajectory_topic, Path, self.storeTrajectory, queue_size=1)
        rospy.Subscriber(inputs_topic, StampedFloat2d, self.storeInputs, queue_size=1)
        rospy.Subscriber(metadata_topic, MPC_metadata, self.storeMetadata, queue_size=1)

        #define publisher(s)
        self.reach_set_pub = rospy.Publisher('/reach_set', Path, queue_size=1)

        #predicted trajectory and inputs stored from last computation
        self.predictedTrajectory = []
        self.predictedInputs = []

    def start(self):
        rospy.spin()

    def storeTrajectory(self, msg):
        self.predictedTrajectory = msg.data

    def storeInputs(self, msg):
        self.predictedInputs = msg.data
    
    def storeMetadata(self, msg):
        self.control_meta = msg

    def doReachability(self, msg):
        self.hylaa.initializeModel({
            "dt" : self.control_meta.dt,
            "total" : self.control_meta.simtime
            },
            self.predictedTrajectory, self.predictedInputs)
        self.hylaa.runModel()

    
if __name__ == "__main__":
    node = Reachability()
    node.start()
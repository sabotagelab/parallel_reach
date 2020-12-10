#!/usr/bin/python3.6
# library imports
from functools import partial

#local imports
from F1Hylaa import F1Hylaa

#ROS imports
import rospy
from std_msgs.msg import Header
from osuf1_common.msg import MPC_trajectory, MPC_metadata, MPC_prediction, ReachSets, NPointSet, NPoint

class HYLAA_node:
    def __init__(self):

        rospy.init_node("hylaa_node")

        #self.modeList = None    #store list of all modes so we can retrieve correct reachsets
        #self.initialBox = None  #we need to store the initial stateset
        #self.core = None        #core object for running hylaa with settings

        #---------------------------------------------------------
        #                   HYLAA Params
        #---------------------------------------------------------
        self.predictions = None #predictions from MPC
        self.reachability_horizon = rospy.get_param("hylaa_node/horizon", 0)
        self.dt = None          #dt from MPC metdata
        self.mpc_horizon = None #simulation horizon from MPC metadata

        #TODO automatic or otherwise validated variability
        self.state_uncertainty = [
            rospy.get_param("hylaa_node/state_uncertainty/px", 0),
            rospy.get_param("hylaa_node/state_uncertainty/py", 0),
            rospy.get_param("hylaa_node/state_uncertainty/psi", 0),
        ]

        self.input_uncertainty = [
            rospy.get_param("hylaa_node/input_uncertainty/velocity", 0),
            rospy.get_param("hylaa_node/input_uncertainty/delta", 0)
        ]


        #custom horizon or utilize all computed MPC steps
        self.hylaa_verbosity = rospy.get_param("hylaa_node/output_verbosity", "VERBOSE").upper() #TODO integrate with roslogger 
        self.graph_predictions = rospy.get_param("hylaa_node/graph_predictions", False) #wether predicted sim results should be overlayed on reach set graph (if displayed)
        self.displayType = rospy.get_param("hylaa_node/display_type", "NONE").upper()
        self.output = rospy.get_param("hylaa_node/display_filename", "hylaa_reach.png") #TODO put images in distinct folder and label with frame times


        #---------------------------------------------------------
        #                   HYLAA Calling Vars
        #---------------------------------------------------------
        self.hylaa = F1Hylaa()
        self.hylaa.set_model_params(self.state_uncertainty, self.input_uncertainty, "kinematics_model")

        #custom interval in ms or 0=maximum speed
        self.reachability_interval = rospy.get_param("hylaa_node/interval", 0)

        self.current_metadata = False
        self.current_trajectory = False
        #UNUSED self.hylaa_running = False #Uneeded unless hylaa calls become async

        #---------------------------------------------------------
        #                   NODE Params
        #---------------------------------------------------------
        self.prediction_topic = rospy.get_param("hylaa_node/mpc_prediction_topic", "mpc_prediction") #topic which publishes predicted trajectory
        self.metadata_topic = rospy.get_param("hylaa_node/mpc_metadata_topic", "mpc_metadata")
        self.reach_pub_topic = rospy.get_param("hylaa_node/reach_pub_topic", "hylaa_reach") #topic where reach sets are published

        self.prediction_sub = rospy.Subscriber(self.prediction_topic, MPC_trajectory, self.storePredictions)
        self.metadata_sub = rospy.Subscriber(self.metadata_topic, MPC_metadata, self.storeMetadata)
        self.reach_pub = rospy.Publisher(self.reach_pub_topic, ReachSets, queue_size=1)
        #self.reach_viz_pub = rospy.Publisher(self.reach_pub_topic+'_viz') #TODO implement vizualizations
        self.MPC_frame_id = None

    def run_hylaa(self):
        rospy.loginfo("Hylaa reachability computation finished.") #TODO add time to output

    def start(self):
	#TODO consider timing of run_hylaa calls more closely 
        #should it run ASAP on possibly old messages?
        #or should it run immediately after recieving mpc data to ensure largest useful horizon
        hz = 1/self.reachability_interval if self.reachability_interval else 0
        if hz:
            rate = rospy.Rate(hz)
        while not rospy.is_shutdown(): 
            if self.current_metadata and self.current_trajectory:
                rospy.loginfo("STARTING HYLAA !!!!!!")
                self.current_metadata = self.current_trajectory = False
                reach = self.hylaa.run_hylaa(self.predictions)

                #build message to publish reachset
                reachMessage = ReachSets()
                reachMessage.header = Header()
                reachMessage.header.stamp = rospy.Time.now()
                reachMessage.header.frame_id = self.MPC_frame_id
                reachMessage.sets = [
                    NPointSet(
                        [NPoint(p) for p in rs]
                    ) for rs in reach 
                ]

                self.reach_pub.publish(reachMessage)
            else:
                rospy.loginfo_throttle(15, "Waiting for updated mpc data")
            if hz:
                rate.sleep()

    def storePredictions(self, data):
        self.predictions = [[list(p.state),list( p.inputs)] for p in data.trajectory]
        self.MPC_frame_id = data.header.frame_id
        self.current_trajectory = True

    def storeMetadata(self, data):
        if self.dt != data.dt or self.mpc_horizon != data.horizon:
            self.dt = data.dt
            self.mpc_horizon = data.horizon

            if self.reachability_horizon > self.mpc_horizon:
                rospy.logerr("Reachability horizon exceeds mpc prediction horizon. Exiting Hylaa Node")
                rospy.logdebug("Reachability horizon: {}\nMPC horizon: {}".format(
                    self.reachability_horizon, self.mpc_horizon
                ))
                exit(1)

            horizon = self.reachability_horizon 
            if horizon == 0:
                horizon = self.mpc_horizon

            self.hylaa.make_settings(self.dt, horizon, self.displayType, self.hylaa_verbosity)

        self.current_metadata = True

if __name__ == "__main__":
    node = HYLAA_node()
    node.start()

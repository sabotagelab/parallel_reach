# library imports
from functools import partial
import subprocess
import xmlrpc.client

#local imports
from F1Hylaa import F1Hylaa

#ROS imports
import rospy
from osuf1_common import MPC_Prediction, MPC_metadata, Reach

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
        self.reachability_horizon = rospy.get_param("horizon", 2.0) #horizon in seconds
        self.dt = None          #dt from MPC metdata
        self.mpc_horizon = None #simulation horizon from MPC metadata

        #TODO automatic or otherwise validated variability
        self.state_uncertainty = [
            rospy.get_param("px", 0),
            rospy.get_param("py", 0),
            rospy.get_param("psi", 0),
        ]

        self.input_uncertainty = [
            rospy.get_param("velocity", 0),
            rospy.get_param("steer", 0),
            0,  #e0
            0   #d0
        ]


        #custom horizon or utilize all computed MPC steps
        self.reachability_horizon = rospy.get_param("horizon", 0)
        self.hylaa_verbosity = rospy.get_param("output_verbosity", "VERBOSE").upper() #TODO integrate with roslogger 
        self.graph_predictions = rospy.getparam("graph_predictions", False) #wether predicted sim results should be overlayed on reach set graph (if displayed)
        self.displayType = rospy.get_param("display_type", "NONE").upper()
        self.output = rospy.get_param("display_filename", "f1_kinematics.png") #TODO put images in distinct folder and label with frame times


        #---------------------------------------------------------
        #                   HYLAA Calling Vars
        #---------------------------------------------------------
        #hylaa_caller = {

            #"server_port" : 7890,
            #"server_host" : 'localhost',
            #"physical_core" : 1
        #}
        #subprocess.call("python ./hylaa_server.py {} {}".format(
            #hylaa_caller["host"],
            #hylaa_caller["port"]
        #))
        #self.hylaa_server = xmlrpc.client.ServerProxy(
            #"{}:{}".format(
                #hylaa_caller["server_host"],
                #hylaa_caller["server_port"]
            #)
        #)
        self.hylaa = F1Hylaa()

        #custom interval in ms or 0=maximum speed
        self.reachability_interval = rospy.get_param("interval", 0)

        self.current_metadata = False
        self.current_trajectory = False
        #UNUSED self.hylaa_running = False #Uneeded unless hylaa calls become async

        #---------------------------------------------------------
        #                   NODE Params
        #---------------------------------------------------------
        self.prediction_topic = rospy.get_param("mpc_prediction_topic", "mpc_prediction") #topic which publishes predicted trajectory
        self.metadata_topic = rospy.get_param("mpc_metadata_topic", "mpc_metadata")
        self.reach_pub_topic = "hylaa_reach" #topic where reach sets are published

        self.prediction_sub = rospy.Subscriber(self.prediction_topic, MPC_Prediction, self.storePredictions)
        self.metadata_sub = rospy.Subscriber(self.metadata_topic, MPC_metadata, self.storeMetadata)
        self.reach_pub = rospy.Publisher(self.reach_pub_topic, Reach, queue_size=1)
        #self.reach_viz_pub = rospy.Publisher(self.reach_pub_topic+'_viz') #TODO implement vizualizations



    def start(self):

        hz = 1/self.reachability_interval if self.reachability_interval else 0
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown(): 
            if self.current_metadata and self.current_trajectory:
                reach = self.hylaa.run_hylaa(self.predictions)
                self.reach_pub.Publish("Data")
                self.current_metadata = self.current_trajectory = False
            if hz:
                rate.sleep()

    def storePredictions(self, data):
        self.predictions = data
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

            self.hylaa.make_settings(self.dt, self.mpc_horizon, self.displayType, self.hylaa_verbosity)

        self.current_metadata = True

if __name__ == "__main__":
    node = HYLAA_node()
    node.start()

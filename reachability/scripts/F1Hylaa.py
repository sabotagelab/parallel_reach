# library imports
from functools import partial
from scipy.io import loadmat

#hylaa imports
from hylaa.hybrid_automaton import HybridAutomaton
from hylaa.settings import HylaaSettings, PlotSettings, LabelSettings
from hylaa.core import Core
from hylaa.stateset import StateSet
from hylaa import lputil

#local imports
import kinematics_model as model_gen 
from dynamics import F1Dynamics
import simulator

#ROS imports
import rospy
from osuf1_common import StampedFloat2d, MPC_metadata

#wrapper to run hylaa repeatedly with different inputs
class F1Hylaa:
    def __init__(self):

        rospy.init_node("hylaa_node")

        self.ha = None          #hybrid automaton
        self.modeList = None    #store list of all modes so we can retrieve correct reachsets
        self.initialBox = None  #we need to store the initial stateset
        self.core = None        #core object for running hylaa with settings
        self.predictions = None #predictions from MPC
        self.dt = None          #dt from MPC metdata
        self.mpc_horizon = None #simulation horizon from MPC metadata
        self.model = model_gen.getModel()

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

        #custom interval or 0=maximum speed
        self.reachability_interval = rospy.get_param("interval", 0)

        #custom horizon or utilize all computed MPC steps
        self.reachability_horizon = rospy.get_param("horizon", 0)

        self.hylaa_verbosity = rospy.get_param("output_verbosity", "VERBOSE").upper() #TODO integrate with roslogger 
        self.graph_predictions = rospy.getparam("graph_predictions", False) #wether predicted sim results should be overlayed on reach set graph (if displayed)
        self.prediction_topic = rospy.get_param("mpc_prediction_topic", "mpc_prediction") #topic which publishes predicted trajectory
        self.metadata_topic = rospy.get_param("mpc_metadata_topic", "mpc_metadata")
        self.reach_pub_topic = "hylaa_reach" #topic where reach sets are published

        self.prediction_sub = rospy.Subscriber(prediction_topic, StampedFloat2d, self.storePredictions)
        self.metadata_sub = rospy.Subscriber(metadata_topic, MPC_metadata, self.storeMetadata)
        self.reach_pub = rospy.Publisher(reach_pub_topic, String, queue_size=1)

        self.running_model = False 
        self.current_metadata = False

    def start(self):
        rospy.spin()

    def storePredictions(self, data):
        if not self.running_model:
            self.predictions = data
            if self.current_metadata:
                self.running_model = True
                self.initialize_hylaa()
                reach = self.run_hylaa()
                self.reach_pub.Publish("Data")
                self.current_metadata = False
                self.running_model = False

    def storeMetadata(self, data):
        if not self.running_model:
            self.dt = data.dt
            self.mpc_horizon = data.horizon
            self.current_metadata = True

            if self.reachability_horizon > self.mpc_horizon:
                rospy.logerr("Reachability horizon exceeds mpc prediction horizon. Exiting Hylaa Node")
                rospy.logdebug("Reachability horizon: {}\nMPC horizon: {}".format(
                    self.reachability_horizon, self.mpc_horizon
                ))
                exit(1)


    def initialize_hylaa(self, params):
        self.modeList = []
        ha = self.make_automaton(ha, zip(trajectory, inputs))
        ttime = self.reachability_horizon
        if ttime == 0:
            ttime = self.mpc_horizon

        settings = self.getSettings(self.dt, ttime)
        self.initialBox = self.boundState(trajectory[0])
        self.core = Core(ha, settings)

    #TODO refactor settings so only changes are modified between hylaa interations
    def getSettings(dt, total):
        'make the reachability settings object'

        # see hylaa.settings for a list of reachability settings
        settings = HylaaSettings(dt, total) # step size = 0.1, time bound 20.0
        settings.plot.filename = rospy.get_param("display_filename", "f1_kinematics.png") #TODO put images in distinct folder and label with frame times
        settings.optimize_tt_transitions = True

        #wwe want to store the reach set cuz that is the whole point of this
        settings.plot.store_plot_result = True 

        displayType = rospy.get_param("display_type", "NONE").upper()
        settings.plot.plot_mode = PlotSettings.PLOT_IMAGE
        settings.stdout = HylaaSettings.STDOUT_VERBOSE

        if displayType == "NONE":
            settings.plot.plot_mode = PlotSettings.PLOT_NONE
        elif displayType == "IMAGE":
            settings.plot.plot_mode = PlotSettings.PLOT_IMAGE
        elif displayType == "VIDEO":
            settings.plot.plot_mode = PlotSettings.PLOT_VIDEO
        elif: 
            print("WARNING: Invalid hylaa plot type specified, default=NONE used")
            settings.plot.plot_mode = PlotSettings.PLOT_NONE

        if verbosity == "NONE":
            settings.stdout = HylaaSettings.STDOUT_NONE
        elif verbosity == "VERBOSE":
            settings.stdout = HylaaSettings.STDOUT_VERBOSE
        elif:
            print("WARNING: Invalid hylaa output verbosity specified, default=VERBOSE used")
            settings.stdout = HylaaSettings.STDOUT_VERBOSE
        
        if displayType != "NONE":
            settings.plot.xdim_dir = [0, 3]
            settings.plot.ydim_dir = [1, 2]
            settings.plot.label = [LabelSettings(), LabelSettings()]

            xyplot = settings.plot.label[0]
            ytplot = settings.plot.label[1]
            #xy plot
            xyplot.big(size=26)
            xyplot.title = "Linearized Kinematics"
            xyplot.x_label = "X Position"
            xyplot.y_label = "Y Position"

            #yaw time plot
            ytplot.big(size=26)
            ytplot.title = "Yaw Vs Time"
            ytplot.x_label = "Time"
            ytplot.y_label = "Yaw"

            #plot the predicted trajectory (MPC output)
            nl_Circles = None
            if self.graph_predictions:
                nl_Centers = [(state[0], state[1]) for state, _ in self.predictions]
                nl_Patches = [Ellipse(center, .2, .2) for center in nl_Centers]
                nl_Circles = collections.PatchCollection(nl_Patches, facecolors='red', edgecolors='black', zorder=1000)    

                nl_yaw_centers = [(state[3], state[2]) for state, _ in self.precitions]
                nl_yaw_patches = [ellipse(center, dt/4 * 2, 3.14/32) for center in nl_yaw_Centers]
                nl_yaw_circles = collections.patchcollection(yawpatches, facecolors='red', edgecolors='black', zorder=1000)
            
                settings.plot_extra_collections = [[nl_Circles]]


        return settings

    def make_init(self, initialState):
        'make the initial states'

        # initial set has every variable as [-0.1, 0.1]
        mode = ha.modes['m0']

        dims = mode.a_csr.shape[0]
        init_box = [(0, 0)] * (dims-2)

        #initial state = state uncertainty + time uncertainty(0)
        #we assume the uncertainty is symettrical for x, y, yaw
        time_init = [(0.0, 0.0), (1.0, 1.0)]
        for s in range(len(init_box)):
            init_box[s] = ( 
                initialState[s] - self.uncertainty[s],
                initialState[s] + self.uncertainty[s]
            )
        init_box += time_init
        print(init_box)
        init_lpi = lputil.from_box(init_box, mode)
        
        init_list = [StateSet(init_lpi, mode)]

        return init_list

    #generated modes IN PLACE on given ha, filling with given inputs
    def make_automaton(self)
        ha = HybridAutomaton()

        modeLabelIncrement = 0
        lastMode = None
        prevState = self.initialState
        prevInputs = self.predictions[0][1] #the first inputs (at t=0)
        for state, inputs in self.predictions:
            rospy.logdebug(state)
            rospy.logdebug(inputs)

            #get the dynamics linearized around this state/input set
            e0d0 = [prevState[2], prevInputs[1]]
            dynamics = self.model.getTimeAugmentMatrices(state, inputs)
            prevState = state
            prevInputs = inputs

            modeName = 'm{}'.format(modeLabelIncrement)
            mode = ha.new_mode(modeName)
            self.modeList.push_back(modeName)

            a_matrix = dynamics['A']
            b_matrix = dynamics['B']
            mode.set_dynamics(a_matrix)

            bounds_mat = dynamics["bounds_mat"]
            bounds_rhs = dynamics["bounds_rhs"]
            mode.set_inputs(b_matrix, bounds_mat, bounds_rhs)

            criticalTime = state[-1]# + 1e-4 #using time offset frome rendevous example

            invariant_mat = [
                [0, 0, 0, 1, 0]
            ]

            #TODO add configurable ciritical time variance
            invariant_rhs = [
                criticalTime + 1e-4
            ]
            mode.set_invariant(invariant_mat, invariant_rhs)

            if lastMode:        
                t = ha.new_transition(lastMode, mode)
                guard_mat = [ [0.0, 0.0, 0.0, -1.0, 0.0] ]
                guard_rhs = [ -criticalTime + 1e-4] #time from last state is bound
                t.set_guard(guard_mat, guard_rhs)

            lastMode = mode
            modeLabelIncrement += 1

            return ha


    def run_hylaa(self):
        result = self.core.run(self.initialBox)
        reachset = result.plot_data.get_verts_list(self.modeList[-1])
        rospy.loginfo("Hylaa reachability computation finished.") #TODO add time to output

        return reachset #TODO export reach sets from all modes

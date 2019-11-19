# library imports
from functools import partial
from scipy.io import loadmat
import importlib

#hylaa imports
from hylaa.hybrid_automaton import HybridAutomaton
from hylaa.settings import HylaaSettings, PlotSettings, LabelSettings
from hylaa.core import Core
from hylaa.stateset import StateSet
from hylaa import lputil
from matplotlib.patches import Circle, Ellipse
from matplotlib import collections
from matplotlib import pyplot as plt

#local imports
from nl_dynamics import F1Dynamics
import simulator

##ROS imports
#import rospy
#from osuf1_common import StampedFloat2d, MPC_metadata

#wrapper to run hylaa repeatedly with different inputs
class F1Hylaa:
    def __init__(self):

        # ------------------ Semi-Static Configuration ----------------------
        self.settings = None #hylaa settings object
        self.dt = None          #dt from MPC metdata
        self.ttime = None

        self.input_uncertainty = []
        self.state_uncertainty = []

        self.model = None       #dynamics abstraction object

        #TODO automatic or otherwise validated variability
        # ------------------ Per-run Configuration ----------------------
        self.ha = None
        self.predictions = None #predictions from MPC 3d array : [ dt[ state[x,y,psi], inputs[d,v] ] ]
        self.stateList = []
        self.initialState = None


    def set_model_params(self, state_uncertainty, input_uncertainty, dynamics_module="kinematics_model"):
        model_gen = importlib.import_module(dynamics_module)
        self.modeList = []
        self.state_uncertainty = state_uncertainty
        self.input_uncertainty = input_uncertainty
        self.model = model_gen.getModel()
        self.model.setInputUncertainty(input_uncertainty)

        return 1
    
    def run_hylaa(self, predictions):
        self.ha = None
        self.predictions = None
        self.stateList = []
        self.initialState = None

        self.ha = HybridAutomaton()
        self.predictions = predictions
        self.graphPredictions()
        self.make_automaton()

        initialBox = self.make_init(self.predictions[0][0])
        core = Core(self.ha, self.settings)
        result = core.run(initialBox)
        reachset = result.plot_data.get_verts_list(self.modeList[-1])


        #return [(float(x), float(y)) for x, y in reachset] #TODO export reach sets from all modes
        return reachset

    def make_settings(self, dt, total, displayType, verbosity, fileName="hylaa_reach.png"):
        self.dt = dt
        self.ttime = total

        'make the reachability settings object'

        # see hylaa.settings for a list of reachability settings
        settings = HylaaSettings(dt, total)
        settings.optimize_tt_transitions = True
        settings.plot.filename = fileName

        #wwe want to store the reach set cuz that is the whole point of this
        settings.plot.store_plot_result = True 

        settings.plot.plot_mode = PlotSettings.PLOT_IMAGE
        settings.stdout = HylaaSettings.STDOUT_VERBOSE

        if displayType == "NONE":
            settings.plot.plot_mode = PlotSettings.PLOT_NONE
        elif displayType == "IMAGE":
            settings.plot.plot_mode = PlotSettings.PLOT_IMAGE
        elif displayType == "VIDEO":
            settings.plot.plot_mode = PlotSettings.PLOT_VIDEO
        else: 
            rospy.logwarn("Invalid hylaa plot type specified, default=NONE used")
            settings.plot.plot_mode = PlotSettings.PLOT_NONE

        if verbosity == "NONE":
            settings.stdout = HylaaSettings.STDOUT_NONE
        elif verbosity == "VERBOSE":
            settings.stdout = HylaaSettings.STDOUT_VERBOSE
        else:
            rospy.logwarn("WARNING: Invalid hylaa output verbosity specified, default=VERBOSE used")
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
        
        self.settings = settings
        return 1

    def graphPredictions(self):
        if True: #self.graph_predictions:
            nl_Centers = [(state[0], state[1]) for state, _ in self.predictions]
            nl_Patches = [Ellipse(center, .2, .2) for center in nl_Centers]
            nl_Circles = collections.PatchCollection(nl_Patches, facecolors='red', edgecolors='black', zorder=1000)    

            nl_yaw_centers = [(state[3], state[2]) for state, _ in self.predictions]
            nl_yaw_patches = [Ellipse(center, self.dt/4 * 2, 3.14/32) for center in nl_yaw_centers]
            nl_yaw_circles = collections.PatchCollection(nl_yaw_patches, facecolors='red', edgecolors='black', zorder=1000)
        
            self.settings.plot.extra_collections = [[nl_Circles],[nl_yaw_circles]]

    def make_init(self, initialState):
        'make the initial states'
        # initial set has every variable as [-0.1, 0.1]
        mode = self.ha.modes['m0']

        dims = mode.a_csr.shape[0]
        init_box = [(0, 0)] * (dims-2)

        #initial state = state uncertainty + time uncertainty(0)
        #we assume the uncertainty is symettrical for x, y, yaw
        time_init = [(0.0, 0.0), (1.0, 1.0)]
        for s in range(len(init_box)):
            init_box[s] = ( 
                initialState[s] - self.state_uncertainty[s],
                initialState[s] + self.state_uncertainty[s]
            )
        init_box += time_init
        init_lpi = lputil.from_box(init_box, mode)
        
        init_list = [StateSet(init_lpi, mode)]

        return init_list

    #generated modes IN PLACE on given ha, filling with given inputs
    def make_automaton(self):

        modeLabelIncrement = 0
        lastMode = None
        prevState = self.predictions[0][0]
        prevInputs = self.predictions[0][1] #the first inputs (at t=0)
        for state, inputs in self.predictions:

            #get the dynamics linearized around this state/input set
            e0d0 = [prevState[2], prevInputs[1]]
            dynamics = self.model.linearized_dynamics(state[:-1], inputs+e0d0)
            prevState = state
            prevInputs = inputs

            modeName = 'm{}'.format(modeLabelIncrement)
            mode = self.ha.new_mode(modeName)
            self.modeList.append(modeName)

            a_matrix = dynamics['A']
            b_matrix = dynamics['B']
            mode.set_dynamics(a_matrix)

            bounds_mat = dynamics["bounds_mat"]
            bounds_rhs = dynamics["bounds_rhs"]
            mode.set_inputs(b_matrix, bounds_mat, bounds_rhs, allow_constants=True)

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
                t = self.ha.new_transition(lastMode, mode)
                guard_mat = [ [0.0, 0.0, 0.0, -1.0, 0.0] ]
                guard_rhs = [ -criticalTime + 1e-4] #time from last state is bound
                t.set_guard(guard_mat, guard_rhs)

            lastMode = mode
            modeLabelIncrement += 1


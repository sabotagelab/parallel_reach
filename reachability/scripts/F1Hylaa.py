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
import model
from dynamics import F1Dynamics
import simulator

#wrapper to run hylaa repeatedly with different inputs
class F1Hylaa:
    def __init__(self):
        self.ha = None          #hybrid automaton
        self.modeList = None    #store list of all modes so we can retrieve correct reachsets
        self.initialBox = None  #we need to store the initial stateset
        self.core = None        #core object for running hylaa with settings

        self.uncertainty = [
            rospy.get_param("px_variation"),
            rospy.get_param("py_variation"),
            rospy.get_param("psi_variation")
        ]


    def initializeModel(self, params, trajectory, inputs):
        self.modeList = []
        ha = HybridAutomaton()
        generateModes(ha, zip(trajectory, inputs))
        settings = self.getSettings(params["dt"], params["total"])
        self.initialBox = self.boundState(trajectory[0])
        self.core = Core(ha, settings)

    #TODO refactor settings so only changes are modified between hylaa interations
    def getSettings(dt, total):
        'make the reachability settings object'

        # see hylaa.settings for a list of reachability settings
        settings = HylaaSettings(dt, total) # step size = 0.1, time bound 20.0
        settings.plot.filename = rospy.get_param("display_filename", "f1_kinematics.png")
        settings.optimize_tt_transitions = True
        #wwe want to store the reach set cuz that is the whole point of this
        settings.plot.store_plot_result = True 

        displayType = rospy.get_param("display_type", "NONE").upper()
        verbosity = rospy.get_param("output_verbosity", "VERBOSE").upper()


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

        return settings

    def boundState(self, initialState):
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
                initialState[s] - uncertainty[s],
                initialState[s] + uncertainty[s]
            )
        init_box += time_init
        print(init_box)
        init_lpi = lputil.from_box(init_box, mode)
        
        init_list = [StateSet(init_lpi, mode)]

        return init_list

    #generated modes IN PLACE on given ha, filling with given inputs
    def generateModes(self, ha, stateInputs)
        modeLabelIncrement = 0
        lastMode = None
        for state, inputs in predictions:
            #print(state)

            dynamics = model.getTimeAugmentMatrices(state, inputs)

            modeName = 'm{}'.format(modeLabelIncrement)
            mode = ha.new_mode()
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

            invariant_rhs = [
                criticalTime
            ]

            mode.set_invariant(invariant_mat, invariant_rhs)

            if lastMode:        
                t = ha.new_transition(lastMode, mode)
                guard_mat = [ [0.0, 0.0, 0.0, -1.0, 0.0] ]
                guard_rhs = [ -criticalTime + 1e-4] #time from last state is bound
                t.set_guard(guard_mat, guard_rhs)

            lastMode = mode
            modeLabelIncrement += 1


    def runModel(self):
        result = self.core.run(self.initialBox)
        reachset = result.plot_data.get_verts_list(self.modeList[-1])

        return reachset

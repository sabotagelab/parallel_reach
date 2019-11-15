# This is a 3 dimensional model with 2 inputs
# It represents the kinematics of the f110 car
# the state is augented with two rows where the first augment is time and the second is an accumulation buffer
# so the final dimensionality is 5

# State Variables are: 

# library imports
from functools import partial
from scipy.io import loadmat
from matplotlib.patches import Circle, Ellipse
from matplotlib import collections
from matplotlib import pyplot as plt
import math

#hylaa imports
from hylaa.hybrid_automaton import HybridAutomaton
from hylaa.settings import HylaaSettings, PlotSettings, LabelSettings
from hylaa.core import Core
from hylaa.stateset import StateSet
from hylaa import lputil

#local imports
import model_OLD as model
from nl_dynamics import F1Dynamics
from lin_dynamics import F1Dynamics_Linear
import simulator


def make_automaton(dt, total, initialState, inputFunc, headless):
    'make the hybrid automaton'

    ha = HybridAutomaton()



    nlDynamics = F1Dynamics()
    stepFunc = partial(nlDynamics.frontStep, nlDynamics)


    sim = simulator.ModelSimulator(dt, total, initialState, stepFunc, inputFunc, headless)
    predictions = sim.simulate()

    modeLabelIncrement = 0
    lastMode = None
    prevState = initialState
    prevInputs = inputFunc(0)
    for state, inputs in predictions:
        e0d0 = [prevState[2], prevInputs[1]]
        dynamics = model.getTimeAugmentMatrices(state, inputs + e0d0)
        prevState = state
        prevInputs = inputs

        mode = ha.new_mode('m{}'.format(modeLabelIncrement))

        a_matrix = dynamics['A']
        b_matrix = dynamics['B']

        mode.set_dynamics(a_matrix)

        bounds_mat = dynamics["bounds_mat"]
        bounds_rhs = dynamics["bounds_rhs"]
        mode.set_inputs(b_matrix, bounds_mat, bounds_rhs, allow_constants=True)

        criticalTime = state[-1]# + 1e-4 #using time offset frome rendevous example

        invariant_mat = [
            [0, 0, 0, 1, 0]
            #[0, 0, 0, 0, 0, 1, 0]
        ]

        invariant_rhs = [
            criticalTime + 1e-4
        ]

        mode.set_invariant(invariant_mat, invariant_rhs)

        if lastMode:        
            t = ha.new_transition(lastMode, mode)
            guard_mat = [ [0.0, 0.0, 0.0, -1.0, 0.0] ]
            #guard_mat = [ [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0] ]
            guard_rhs = [ -criticalTime + 1e-4] #time from last state is bound
            t.set_guard(guard_mat, guard_rhs)

        lastMode = mode
        modeLabelIncrement += 1

    #TODO error checking
    #error = ha.new_mode('error')

    ## the third output defines the unsafe condition
    #y3 = dynamics['C'][2]

    #limit = 0.0005
    ##limit = 0.0007

    ## Error condition: y3 * x <= -limit OR y3 >= limit
    #trans1 = ha.new_transition(mode, error)
    #trans1.set_guard(y3, [-limit])

    #trans2 = ha.new_transition(mode, error)
    #trans2.set_guard(-1 * y3, [-limit])

    return ha, predictions

def make_init(ha, uncertainty, initialState):
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

def make_settings(dt, total, predictions, headless, lin_predictions=None):
    'make the reachability settings object'

    # see hylaa.settings for a list of reachability settings
    settings = HylaaSettings(dt, total) # step size = 0.1, time bound 20.0
    settings.plot.filename = "f1_kinematics_sim0.png"
    settings.plot.plot_mode = PlotSettings.PLOT_IMAGE
    settings.stdout = HylaaSettings.STDOUT_VERBOSE
    settings.optimize_tt_transitions = True
    settings.plot.store_plot_result = True

    if headless:
        settings.plot.plot_mode = PlotSettings.PLOT_NONE
        settings.stdout = HylaaSettings.STDOUT_NONE
        settings.plot.store_plot_result = True

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
    xyplot.axes_limits = [-10, 10, -10, 10]

    xyCenters = [(state[0], state[1]) for state, _ in predictions]
    xyPatches = [Ellipse(center, .2, .2) for center in xyCenters]
    xyCircles = collections.PatchCollection(xyPatches, facecolors='red', edgecolors='black', zorder=1000)    

    #create visualization for test of linear dynamics simulation.
    lin_xyCircles = None
    if lin_predictions != None:
        lin_xyCenters = [(state[0], state[1]) for state, _ in lin_predictions]
        lin_xyPatches = [Ellipse(center, .2, .2) for center in lin_xyCenters]
        lin_xyCircles = collections.PatchCollection(lin_xyPatches, facecolors='blue', edgecolors='black', zorder=0)    

    #yaw time plot
    ytplot.big(size=26)
    ytplot.title = "Yaw Vs Time"
    ytplot.x_label = "Time"
    ytplot.y_label = "Yaw"

    yawCenters = [(state[3], state[2]) for state, _ in predictions]
    yawPatches = [Ellipse(center, dt/4 * 2, 3.14/32) for center in yawCenters]
    yawCircles = collections.PatchCollection(yawPatches, facecolors='red', edgecolors='black', zorder=1000)
    
    lin_yawCircles = None
    if lin_predictions != None:
        lin_yawCenters = [(state[3], state[2]) for state, _ in lin_predictions]
        lin_yawPatches = [Ellipse(center, dt/4 * 2, 3.14/32) for center in lin_yawCenters]
        lin_yawCircles = collections.PatchCollection(lin_yawPatches, facecolors='blue', edgecolors='black')
    
    #hacked in code for generating standalone non-linear simulation results
    #simfig = plt.figure()
    #ax = simfig.add_subplot(1, 1, 1)
    #ax.set_xlim(-3, 3)
    #ax.set_ylim(-3, 3)
    #ax.add_collection(xyCircles)
    #ax.set_title("Non-linear Kinematic Simulation")
    #ax.set_xlabel('x-position')
    #ax.set_ylabel('y-position')
    #ax.axis("equal")
    #simfig.savefig("nonlinear-kinematics-og.png")
    #simfig.clf()

    settings.plot.extra_collections = [[xyCircles, lin_xyCircles],[yawCircles, lin_yawCircles]]
    return settings

def run_hylaa( total = 2, dt = .1, headless=False, outfile="f110_reach.dat"):
    'main entry point'

    #state = [x, y, yaw]
    initialState = [-8, 0, 0]
    uncertainty = [.1, .1, .00]
    inputFunc = lambda t : [ 6, -1 * math.cos(2*t)/4]


    #initializing linear dynamics simulation for test
    lin_Dynamics = F1Dynamics_Linear(initialState)
    lin_stepFunc = partial(lin_Dynamics.frontStep, lin_Dynamics)
    lin_sim = simulator.ModelSimulator(dt, total, initialState, lin_stepFunc, inputFunc, headless)
    lin_predictions = lin_sim.simulate()

    ha, predictions = make_automaton(dt, total, initialState, inputFunc, headless)

    init_states = make_init(ha, uncertainty, initialState)

    settings = make_settings(dt, total, predictions, headless, lin_predictions=lin_predictions)

    result = Core(ha, settings).run(init_states)

    if not headless:
        from sympy import Matrix, pprint
        from contextlib import redirect_stdout
        with open(outfile, "w+") as reachSetFile:
            with redirect_stdout(reachSetFile):
                for m in range(int(total/dt)):
                    mode = "m{}".format(m)
                    reachSetFile.write("Vertex list from {} : \n".format(mode))
                    for r in result.plot_data.get_verts_list(mode):
                        print("S : \n")
                        pprint(Matrix(r[0]))
                        print("\n")

    return result.top_level_timer.total_secs

if __name__ == "__main__":
    run_hylaa()

# This is a 3 dimensional model with 2 inputs
# It represents the kinematics of the f110 car

# State Variables are: 
from functools import partial

from scipy.io import loadmat

from hylaa.hybrid_automaton import HybridAutomaton
from hylaa.settings import HylaaSettings, PlotSettings, LabelSettings
from hylaa.core import Core
from hylaa.stateset import StateSet
from hylaa import lputil

import model
from dynamics import F1Dynamics
import simulator


def make_automaton(dt, total, initialState, headless):
    'make the hybrid automaton'

    ha = HybridAutomaton()



    nlDynamics = F1Dynamics()
    stepFunc = partial(nlDynamics.frontStep, nlDynamics)
    inputFunc = lambda t : [ 1, 3.14/4]


    sim = simulator.ModelSimulator(dt, total, initialState, stepFunc, inputFunc, headless)
    predictions = sim.simulate()

    modeLabelIncrement = 0
    lastMode = None
    for state, inputs in predictions:
        #print(state)

        dynamics = model.getTimeAugmentMatrices(state, inputs)

        mode = ha.new_mode('m{}'.format(modeLabelIncrement))

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

    return ha

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

def make_settings(dt, total, headless):
    'make the reachability settings object'

    # see hylaa.settings for a list of reachability settings
    settings = HylaaSettings(dt, total) # step size = 0.1, time bound 20.0
    settings.plot.filename = "f1_kinematics.png"
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

    #yaw time plot
    ytplot.big(size=26)
    ytplot.title = "Yaw Vs Time"
    ytplot.x_label = "Time"
    ytplot.y_label = "Yaw"
    return settings

def run_hylaa( total = 1, dt = .1, headless=False, outfile="f110_reach.dat"):
    'main entry point'

    #state = [x, y, yaw]
    initialState = [0, 0, 0]
    uncertainty = [.1, .1, .00]

    ha = make_automaton(dt, total, initialState, headless)

    init_states = make_init(ha, uncertainty, initialState)

    settings = make_settings(dt, total, headless)

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

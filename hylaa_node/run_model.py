# This is a 3 dimensional model with 2 inputs
# It represents the kinematics of the f110 car

# State Variables are: 
from functools import partial

from scipy.io import loadmat

from hylaa.hybrid_automaton import HybridAutomaton
from hylaa.settings import HylaaSettings, PlotSettings
from hylaa.core import Core
from hylaa.stateset import StateSet
from hylaa import lputil

import model
import dynamics
import simulator


def make_automaton(dt, total):
    'make the hybrid automaton'

    ha = HybridAutomaton()

    #state = [x, y, yaw]
    initialState = [0, 0, 0]

    nlDynamics = dynamics.F1Dynamics()
    stepFunc = bind(nlDynamics, nlDynamics.frontStep)
    inputFunc = lambda t : [ 3.14/6, 1]


    sim = simulator.ModelSimulator(dt, total, initialState, stepFunc, inputFunc)
    stateEstimates = simulator.simulate()

    modeLabelIncrement = 0
    for state in stateEstimates:
        dynamics = model.getTimeAugmentMatrices(*state)

        mode = ha.new_mode('m{}'.format(modeLabelIncrement))

        a_matrix = dynamics['A']
        b_matrix = dynamics['B']

        mode.set_dynamics(a_matrix)

        bounds_mat = dynamics["bounds_mat"]
        bounds_rhs = dynamics["bounds_rhs"]
        mode.set_inputs(b_matrix, bounds_mat, bounds_rhs)

        if lastMode:        
            t = ha.new_transition(lastMode, mode)
            guard_mat = [
                
            ]


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

def make_init(ha):
    'make the initial states'

    # initial set has every variable as [-0.0001, 0.0001]
    mode = ha.modes['m0']

    dims = mode.a_csr.shape[0]
    init_box = dims * [[-0.1, 0.1]]
    init_lpi = lputil.from_box(init_box, mode)
    
    init_list = [StateSet(init_lpi, mode)]

    return init_list

def make_settings():
    'make the reachability settings object'

    # see hylaa.settings for a list of reachability settings
    settings = HylaaSettings(0.1, 10.0) # step size = 0.1, time bound 20.0
    settings.plot.plot_mode = PlotSettings.PLOT_IMAGE
    settings.stdout = HylaaSettings.STDOUT_VERBOSE
    settings.plot.filename = "f1_kinematics.png"


    outputs = dynamics["C"]
    settings.plot.xdim_dir = 0 #outputs[0][0] # x dimension will bethe car's x position 
    settings.plot.ydim_dir = 1 #outputs[1][1] # y dimension will be the car's y position 
    settings.plot.label.title = "Linearized Kinematics"
    settings.plot.label.x_label = "X Position"
    settings.plot.label.y_label = "Y Position"

    return settings

def run_hylaa():
    'main entry point'

    ha = make_automaton(dynamics)

    init_states = make_init(ha)

    settings = make_settings(dynamics)

    Core(ha, settings).run(init_states)

if __name__ == "__main__":
    run_hylaa()

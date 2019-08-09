#simplified to understand how this works.
# State Variables are: 
from functools import partial
from contextlib import redirect_stdout

from scipy.io import loadmat

from hylaa.hybrid_automaton import HybridAutomaton
from hylaa.settings import HylaaSettings, PlotSettings
from hylaa.core import Core
from hylaa.stateset import StateSet
from hylaa import lputil

import model
from dynamics import F1Dynamics
import simulator
from sympy import Matrix, pprint


def make_automaton(dt, total):
    'make the hybrid automaton'

    ha = HybridAutomaton()

    dynamics_a = {
        "A" : [[0, 0],[0, 0]],
        "B" : [[1, 0],[0,1]]
    }
    dynamics_b = {
        "A" : [[0, 0],[0, 0]],
        "B" : [[-1, 0],[0,-1]],
        "bounds_mat" : []
    }

    mode_a = ha.new_mode('first')
    mode_b = ha.new_mode('second')


    mode_a.set_dynamics(dynamics_a["A"])
    mode_b.set_dynamics(dynamics_b["A"])

    t = ha.new_transition(mode_a, mode_b)
    guard_mat = [ [0.0, -1.0] ]
    guard_rhs = [ .5 ] #time from last state is bound
    t.set_guard(guard_mat, guard_rhs)


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
    mode = ha.modes['first']

    dims = mode.a_csr.shape[0]
    init_box = dims * [[-0.1, 0.1]]
    init_lpi = lputil.from_box(init_box, mode)
    
    init_list = [StateSet(init_lpi, mode)]

    return init_list

def make_settings(dt, total):
    'make the reachability settings object'

    # see hylaa.settings for a list of reachability settings
    settings = HylaaSettings(dt, total) # step size = 0.1, time bound 20.0
    settings.plot.plot_mode = PlotSettings.PLOT_NONE
    settings.stdout = HylaaSettings.STDOUT_VERBOSE
    settings.plot.filename = "simplified.png"
    settings.plot.store_plot_result = True


    settings.plot.xdim_dir = 0 #outputs[0][0] # x dimension will bethe car's x position 
    settings.plot.ydim_dir = 1 #outputs[1][1] # y dimension will be the car's y position 
    settings.plot.label.title = "Simplified"
    settings.plot.label.x_label = "State Var 1"
    settings.plot.label.y_label = "State Var 2"

    return settings

def run_hylaa():
    'main entry point'

    dt = .5
    total = 1

    ha = make_automaton(dt, total)

    init_states = make_init(ha)

    settings = make_settings(dt, total)

    result = Core(ha, settings).run(init_states)

    with open("simplified_reach.dat", "w+") as reachSetFile:
        with redirect_stdout(reachSetFile):
            reachSetFile.write("Vertex list from first mode : \n")
            pprint(Matrix(result.plot_data.get_verts_list("first")[0]))
            reachSetFile.write("Vertex list from second mode : \n")
            pprint(Matrix(result.plot_data.get_verts_list("second")[0]))
        

if __name__ == "__main__":
    run_hylaa()

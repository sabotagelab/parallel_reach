#simplified to understand how hylaa TTT work.
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
        "A" : [[1, 0, 0],[0, 0, 1], [0, 0, 0]],
    }
    dynamics_b = {
        "A" : [[1, 0, 0],[0, 0, 1], [0, 0, 0]],
        "bounds_mat" : []
    }
    ttime = .5 - 1e-4

    mode_a = ha.new_mode('first')
    mode_b = ha.new_mode('second')

    mode_a.set_dynamics(dynamics_a["A"])
    mode_b.set_dynamics(dynamics_b["A"])

    # -x2 > -ttime
    mode_a.set_invariant([[0, 1.0, 0]], [ ttime])

    t = ha.new_transition(mode_a, mode_b)

    ## -x2 > -ttime
    guard_mat = [ [0.0, -1.0, 0.0] ]
    guard_rhs = [ -ttime ] #time from last state is bound
    t.set_guard(guard_mat, guard_rhs)

    return ha

def make_init(ha):
    'make the initial states'

    # initial set has every variable as [-0.0001, 0.0001]
    mode = ha.modes['first']

    init_box = [(-.1, .1), (0, 0), (1.0, 1.0)] #dim from -.1 to 1, time is fixed
    init_lpi = lputil.from_box(init_box, mode)
    
    init_list = [StateSet(init_lpi, mode)]

    return init_list

def make_settings(dt, total):
    'make the reachability settings object'

    # see hylaa.settings for a list of reachability settings
    settings = HylaaSettings(dt, total) 
    settings.plot.plot_mode = PlotSettings.PLOT_IMAGE
    settings.stdout = HylaaSettings.STDOUT_VERBOSE
    settings.plot.filename = "simplified.png"
    settings.plot.store_plot_result = True
    settings.optimize_tt_transitions = True


    settings.plot.xdim_dir = 0 #the first dimension
    settings.plot.ydim_dir = 1 #time
    settings.plot.label.title = "Simplified"
    settings.plot.label.x_label = "State Var 1"
    settings.plot.label.y_label = "State Var 2 (t)"

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
            for r in result.plot_data.get_verts_list("first"):
                print("S : \n")
                pprint(Matrix(r[0]))
                print("\n")
            reachSetFile.write("Vertex list from second mode : \n")
            for r in result.plot_data.get_verts_list("second"):
                print("S : \n")
                pprint(Matrix(r[0]))
                print("\n")
        

if __name__ == "__main__":
    run_hylaa()

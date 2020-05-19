#!/home/dev/cuda-venv/bin/python3

#barebones testing script to run hylaa without ROS
# also used by profiler

from F1QuickZono import F1QuickZono
#local imports
from nl_dynamics import F1Dynamics
import simulator

#python lib imports
from functools import partial
import math
import time

import matplotlib.pyplot as plt
import cProfile as profile

#from pycallgraph import PyCallGraph
#from pycallgraph.output import GraphvizOutput
from timeit import Timer

gen_callgraph = False
if gen_callgraph:
    from pycallgraph import PyCallGraph
    from pycallgraph import Config
    from pycallgraph.output import GraphvizOutput

state_uncertainty = [.1, .1, 0]
input_uncertainty = [.1, 3.14/90] # .1m/s , 2deg
nlDynamics = F1Dynamics()
stepFunc = partial(nlDynamics.frontStep, nlDynamics)
inputFunc = lambda t : [ 1+4, -1 * math.cos(2)/4]
headless = True
fy = F1QuickZono() 
fy.set_model_params(state_uncertainty, input_uncertainty, "model_hardcode")

def run_quickzono(dt, ttime, initialState, do_profile=False):
    sim = simulator.ModelSimulator(dt, ttime, initialState, stepFunc, inputFunc, headless)

    print("Simulating")
    global predictions
    predictions = sim.simulate()
    print("Simulation Finished, Initializing Reachability")

    fy.make_settings(dt, ttime)

    result = None
    print("Running quickzono")
    if do_profile:
        print("RUNNING PROFILER")
        timer = Timer("""fy.run(predictions)""", globals=globals())
        return timer.timeit(1)
        #profile.runctx('resultprof = fy.run(predictions)', globals(), locals(), filename="profiler/prof/out_tmp.prof")
        result = locals()['resultprof']
    else:
        #with PyCallGraph(output=GraphvizOutput()):
        result = fy.run(predictions)
    print("quickzono execution finished.")
    #print(result)
    #print("Stateset obj")
    #print(result[0][-3])
    return result


if __name__ == "__main__":
    print("Initializing Simulation")
    initialState = [0, 0, 0]
    dt = .05
    ttime = 1.4

    if gen_callgraph:
        config = Config(max_depth=10)
        graphviz = GraphvizOutput(output_file="quickzono_callgraph.png")
        with PyCallGraph(output=graphviz, config=config):
            zonos = run_quickzono(dt, ttime, initialState)
    else:
        zonos = run_quickzono(dt, ttime, initialState)


    xdim = 0
    ydim = 1

    plot = False
    if plot:
        filename="f1_zonos_noquick.png"
        plt.figure(figsize=(6, 6))
            
        zonos[0].plot(col='r-o', label='Init', xdim=xdim, ydim=ydim)

        for i, z in enumerate(zonos[1:]):
            print("Plotting")
            label = 'Reach Set' if i == 0 else None
            z.plot(label=label, xdim=xdim, ydim=ydim)

        plt.title('Quickzonoreach Output (run_f1zono.py)')
        plt.legend()
        plt.grid()
        plt.savefig(filename)



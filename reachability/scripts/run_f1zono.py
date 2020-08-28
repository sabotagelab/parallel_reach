#!/home/dev/cuda-venv/bin/python3

#barebones testing script to run reachability without ROS
# also used by profiler

from F1QuickZono import F1QuickZono
from quickzonoreach.zono_projection import ZP_TYPE

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

#modal wrappers for profiler
def run_quickzono_CPU(dt, ttime, initialState, profile=1):
    return run_quickzono(dt, ttime, initialState, profile, ZP_TYPE.CPU)

def run_quickzono_CPU_MP(dt, ttime, initialState, profile=1):
    return run_quickzono(dt, ttime, initialState, profile, ZP_TYPE.CPU_MP)

def run_quickzono_GPU_HYBRID(dt, ttime, initialState, profile=1):
    return run_quickzono(dt, ttime, initialState, profile, ZP_TYPE.GPU_Hybrid)

def run_quickzono(dt, ttime, initialState, do_profile=0, runtime_mode=ZP_TYPE.CPU):
    fy.set_model_params(state_uncertainty, input_uncertainty, "model_hardcode", runtime_mode=runtime_mode)
    sim = simulator.ModelSimulator(dt, ttime, initialState, stepFunc, inputFunc, headless)

    #print("Simulating")
    global predictions
    predictions = sim.simulate()
    #print("Simulation Finished, Initializing Reachability")

    fy.make_settings(dt, ttime)

    result = None
    #print("Running quickzono")
    if do_profile==1:
        timer = Timer("""fy.run(predictions)""", globals=globals())
        result = timer.timeit(1)
        #profile.runctx('resultprof = fy.run(predictions)', globals(), locals(), filename="profiler/prof/out_tmp.prof")
        #result = locals()['resultprof']
    elif do_profile==2:
        result = fy.run(predictions, profile=True)
    else:
        #with PyCallGraph(output=GraphvizOutput()):
        result = fy.run(predictions)
    #print("quickzono execution finished.\n")
    #print(result)
    #print("Stateset obj")
    #print(result[0][-3])
    return result


if __name__ == "__main__":
    
    try:
        import multiprocessing
        multiprocessing.set_start_method("spawn")
        print("Threads now spawn not fork.")
    except RuntimeError:
        pass
    print("Initializing Simulation")
    initialState = [0, 0, 0]
    dt = .05
    ttime = 1.0

    if gen_callgraph:
        config = Config(max_depth=15)
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



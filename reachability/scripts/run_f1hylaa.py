#barebones testing script to run hylaa without ROS
# also used by profiler

from F1Hylaa import F1Hylaa
#local imports
from nl_dynamics import F1Dynamics
import simulator

#python lib imports
from functools import partial
import math
import time
import xmlrpc.client
from timeit import Timer

state_uncertainty = [.1, .1, 0]
input_uncertainty = [.1, 3.14/90] # .1m/s , 2deg
nlDynamics = F1Dynamics()
stepFunc = partial(nlDynamics.frontStep, nlDynamics)
inputFunc = lambda t : [ 1+4, -1 * math.cos(2)/4]
headless = True
fy = F1Hylaa()

def run_hylaa_profile(dt, ttime, initialState = [0, 0, 0]):
    return run_hylaa(dt, ttime, initialState, True, False)

def run_hylaa(dt, ttime, initialState, do_profile=False, output=False):
    sim = simulator.ModelSimulator(dt, ttime, initialState, stepFunc, inputFunc, headless)

    print("Simulating")
    global predictions
    predictions = sim.simulate()
    print("Simulation Finished, Initializing Reachability")


    fy.set_model_params(state_uncertainty, input_uncertainty, "kinematics_model")
    fy.make_settings(dt, ttime, output, "VERBOSE", "hylaa.png")

    print("Running HYLAA")
    result = None
    if do_profile:
        timer = Timer("""fy.run_hylaa(predictions)""", globals=globals())
        result = timer.timeit(1)
    else:
        result = fy.run_hylaa(predictions)
    print("HYLAA execution finished.")
    #print(result)
    #print("Stateset obj")
    #print(result[0][-3])
    return result


if __name__ == "__main__":
    print("Initializing Simulation")
    initialState = [0, 0, 0]
    dt = .05
    ttime = 1
    output = "IMAGE"
    reach = run_hylaa(dt, ttime, initialState, output)


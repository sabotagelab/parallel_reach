from F1Hylaa import F1Hylaa
#local imports
from nl_dynamics import F1Dynamics
import simulator

#python lib imports
from functools import partial
import math
import time
import xmlrpc.client

state_uncertainty = [.1, .1, 0]
input_uncertainty = [.1, 3.14/90] # .1m/s , 2deg
nlDynamics = F1Dynamics()
stepFunc = partial(nlDynamics.frontStep, nlDynamics)
inputFunc = lambda t : [ 1+4*t, -1 * math.cos(2*t)/4]
headless = True

def run_hylaa(dt, ttime, initialState, output):
    sim = simulator.ModelSimulator(dt, ttime, initialState, stepFunc, inputFunc, headless)

    print("Simulating")
    predictions = sim.simulate()
    print("Simulation Finished, Initializing Reachability")

    fy = F1Hylaa()

    fy.set_model_params(state_uncertainty, input_uncertainty, "kinematics_model_new")
    fy.make_settings(dt, ttime, output, "NONE", "/home/nvidia/hylaa.png")

    print("Running HYLAA")
    result = fy.run_hylaa(predictions)
    print("HYLAA execution finished.")
    #print(result)
    #print("Stateset obj")
    #print(result[0][-3])


if __name__ == "__main__":
    print("Initializing Simulation")
    initialState = [0, 0, 0]
    dt = .05
    ttime = 1
    output = "IMAGE"
    run_hylaa(dt, ttime, initialState, output)


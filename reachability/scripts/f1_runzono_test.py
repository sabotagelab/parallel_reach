from F1QuickZonoTest import F1QuickZono
#local imports
from nl_dynamics import F1Dynamics
import simulator

#python lib imports
from functools import partial
import math
import time
import xmlrpc.client

import matplotlib.pyplot as plt

state_uncertainty = [.1, .1, 0]
input_uncertainty = [.1, 3.14/90] # .1m/s , 2deg
nlDynamics = F1Dynamics()
stepFunc = partial(nlDynamics.frontStep, nlDynamics)
inputFunc = lambda t : [ 1+4, -1 * math.cos(2)/4]
headless = True

def run_quickzono(dt, ttime, initialState):
    sim = simulator.ModelSimulator(dt, ttime, initialState, stepFunc, inputFunc, headless)

    print("Simulating")
    predictions = sim.simulate()
    print("Simulation Finished, Initializing Reachability")

    fy = F1QuickZono()

    fy.set_model_params(state_uncertainty, input_uncertainty, "kinematics_model")
    fy.make_settings(dt, ttime)

    print("Running quickzono")
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
    ttime = .5
    zonos = run_quickzono(dt, ttime, initialState)

    xdim = 0
    ydim = 1

    plot = True
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
from F1Hylaa import F1Hylaa
#local imports
from nl_dynamics import F1Dynamics
import simulator

#python lib imports
from functools import partial
import math
import time
import xmlrpc.client

hylaa_server = False#xmlrpc.client.ServerProxy("http://127.0.0.1:8970")
print("Initializing Simulation")
initialState = [0, 0, 0]
dt = .1
ttime = 1
headless = True
nlDynamics = F1Dynamics()
stepFunc = partial(nlDynamics.frontStep, nlDynamics)
currentState = [0, 0, 0]
inputFunc = lambda t : [ 3, -1 * math.cos(2*t)/4]
sim = simulator.ModelSimulator(dt, ttime, initialState, stepFunc, inputFunc, headless)

print("Simulating")
#predictions = [([float(fs) for fs in s], [float(fi) for fi in i]) for s,i in sim.simulate()]
predictions = sim.simulate()
print("Simulation Finished, Initializing Reachability")

state_uncertainty = [.1, .1, 0]
input_uncertainty = [.1, .1, 0, 0]

fy = F1Hylaa()
if hylaa_server:
    fy = hylaa_server


fy.set_model_params(state_uncertainty, input_uncertainty)
fy.make_settings(dt, ttime, "IMAGE", "NONE")
print(predictions)
print("Running HYLAA")
fy.run_hylaa(predictions)
print("HYLAA execution finished.")

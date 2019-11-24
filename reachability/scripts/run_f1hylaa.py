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

hylaa_server = False#xmlrpc.client.ServerProxy("http://127.0.0.1:8970")
print("Initializing Simulation")
initialState = [0, 0, 0]
dt = .05
ttime = 1

headless = True
nlDynamics = F1Dynamics()
stepFunc = partial(nlDynamics.frontStep, nlDynamics)
inputFunc = lambda t : [ 1+4*t, -1 * math.cos(2*t)/4]
sim = simulator.ModelSimulator(dt, ttime, initialState, stepFunc, inputFunc, headless)

print("Simulating")
#predictions = [([float(fs) for fs in s], [float(fi) for fi in i]) for s,i in sim.simulate()]
predictions = sim.simulate()
print("Simulation Finished, Initializing Reachability")

state_uncertainty = [.1, .1, 0]
input_uncertainty = [.1, 3.14/90] # .1m/s , 2deg

fy = F1Hylaa()
if hylaa_server:
    fy = hylaa_server


fy.set_model_params(state_uncertainty, input_uncertainty, "kinematics_model_new")
fy.make_settings(dt, ttime, "IMAGE", "NONE")

print("Running HYLAA")
result = fy.run_hylaa(predictions)
print("HYLAA execution finished.")
print(result)
print("Stateset obj")
print(result[0][-3])

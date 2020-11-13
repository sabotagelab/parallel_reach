from matplotlib import pyplot as plt
import numpy as np
from nl_dynamics import F1Dynamics
from functools import partial
import math
import time
from timeit import Timer
from run_f1zono import run_quickzono_CPU, run_quickzono_CPU_MP, run_quickzono_GPU_HYBRID
from run_f1hylaa import run_hylaa_profiler
#from run_f1hylaa import run_hylaa
from quickzonoreach.zono_projection import ZP_TYPE
import pstats
import statistics


min_steps = 5
max_steps = 45

steps_list = list(range(min_steps, 16)) + list(range(20, max_steps, 5))

#define and store model functions
#hylaa = run_hylaa_profile
qz_cpu = run_quickzono_CPU
qz_mp = run_quickzono_CPU_MP 
qz_hybrid = run_quickzono_GPU_HYBRID
qz_dummy = run_quickzono_GPU_DUMMY
hylaa = run_hylaa_profiler

all_modes = [qz_cpu, qz_mp, qz_hybrid, qz_dummy, hylaa]
all_modes_names = ["QZ-CPU", "QZ-MP", "QZ-HYBRID", "QZ-DUMMY", "HYLAA"]


trials_per_stepsmode = 200
full_maxtime = 0
verts_maxtime = 0
std_maxtime = 0
data = [] # (time, steps, mode, verts?)
for steps in steps_list:
    dt = 1
    ttime = steps
    print("Calling for steps={}".format(steps))
    for num, mode, name in zip(range(len(all_modes)), all_modes, all_modes_names):
        print("\tRunning {}".format(name))
        for trial in range(trials_per_stepsmode):
            full_time = mode(dt, ttime, [0, 0, 0])
            data.append((full_time, ttime, name, 0))
            verts_time = mode(dt, ttime, [0, 0, 0], profile=2)
            data.append((verts_time, ttime, name, 1))


with open("raw/offline.csv", "w+") as outputFile:
    for d in data:
        line = f"{d[0]},{d[1]},{d[2]},{d[3]}\n"
        outputFile.write(line)



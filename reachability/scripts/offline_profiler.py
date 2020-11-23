from matplotlib import pyplot as plt
import numpy as np
from nl_dynamics import F1Dynamics
from functools import partial
import math
import time
from timeit import Timer
from run_f1zono import run_quickzono_CPU, run_quickzono_CPU_MP, run_quickzono_GPU_HYBRID, run_quickzono_GPU_DUMMY
#from run_f1hylaa import run_hylaa_profile
#from run_f1hylaa import run_hylaa
from quickzonoreach.zono_projection import ZP_TYPE
import pstats
import statistics


min_steps = 5
max_steps = 45

steps_list = list(range(min_steps, 16)) + list(range(20, max_steps, 5))

start_steps = 9
end_steps = 20
steps_list = steps_list[steps_list.index(start_steps):steps_list.index(end_steps)]

#define and store model functions
#hylaa = run_hylaa_profile
qz_cpu = run_quickzono_CPU
qz_mp = run_quickzono_CPU_MP 
qz_hybrid = run_quickzono_GPU_HYBRID
qz_dummy = run_quickzono_GPU_DUMMY
#hylaa = run_hylaa_profile

all_modes = [qz_cpu, qz_mp, qz_hybrid, qz_dummy]#, hylaa]
all_modes_names = ["QZ_CPU", "QZ_MP", "QZ_HYBRID", "QZ_DUMMY"]#, "HYLAA"]
#all_modes_trials = [200, 200, 200, 200, 200, 30]
all_modes_trials = [20, 20, 20, 20, 20, 1]


trials_per_stepsmode = 200
full_maxtime = 0
verts_maxtime = 0
std_maxtime = 0
data = [] # (time, steps, mode, verts?)
cutoff_modes = set()
cutoff_time = 3 #we only run a few trials for modes over this
cutoff_trials = 3 #run way fewer trials once timout is reached


inline_out = open("raw/offline_tmp.csv", "w+")
for steps in steps_list:
    dt = 1
    ttime = steps
    print("Calling for steps={}".format(steps))
    for num, mode, name, trials in zip(range(len(all_modes)), all_modes, all_modes_names, all_modes_trials):
        print("\tRunning {}".format(name))
        run_trials = trials
        if name in cutoff_modes:
            run_trials = cutoff_trials
        for trial in range(run_trials):
            full_time = mode(dt, ttime, [0, 0, 0])
            d = (full_time, ttime, name, 0)
            data.append(d)
            inline_out.write(f"{d[0]},{d[1]},{d[2]},{d[3]}\n")

            if full_time > cutoff_time:
                cutoff_modes.add(name)
            if name == "HYLAA":
                continue

            verts_time = mode(dt, ttime, [0, 0, 0], profile=2)
            d = (verts_time, ttime, name, 1)
            data.append(d)
            inline_out.write(f"{d[0]},{d[1]},{d[2]},{d[3]}\n")
inline_out.close()


with open("raw/offline.csv", "w+") as outputFile:
    for d in data:
        line = f"{d[0]},{d[1]},{d[2]},{d[3]}\n"
        outputFile.write(line)



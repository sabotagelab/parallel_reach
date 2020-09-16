
from matplotlib import pyplot as plt
import numpy as np
from nl_dynamics import F1Dynamics
from functools import partial
import math
import time
from timeit import Timer
from run_f1zono import run_quickzono_CPU, run_quickzono_CPU_MP, run_quickzono_GPU_HYBRID
#from run_f1hylaa import run_hylaa
from quickzonoreach.zono_projection import ZP_TYPE
import pstats
import statistics


min_steps = 5
max_steps = 40

steps_list = list(range(min_steps, 16)) + list(range(20, max_steps, 5))

#define and store model functions
#hylaa = run_hylaa_profile
qz_cpu = run_quickzono_CPU
qz_mp = run_quickzono_CPU_MP 
qz_hybrid = run_quickzono_GPU_HYBRID

all_modes = [qz_cpu, qz_mp, qz_hybrid]
all_modes_names = ["QZ-CPU", "QZ-MP", "QZ-HYBRID"]


full_mode_results_avg = [[] for _ in range(len(all_modes))]
verts_mode_results_avg = [[] for _ in range(len(all_modes))]
full_mode_results_std = [[] for _ in range(len(all_modes))]
verts_mode_results_std = [[] for _ in range(len(all_modes))]


trials_per_stepsmode = 15
full_maxtime = 0
verts_maxtime = 0
for steps in steps_list:
    dt = 1
    ttime = steps
    print("Calling for steps={}".format(steps))
    for num, mode, name in zip(range(len(all_modes)), all_modes, all_modes_names):
        print("\tRunning {}".format(name))
        full_steps_results = []
        verts_steps_results = []
        for trial in range(trials_per_stepsmode):
            full_time = mode(dt, ttime, [0, 0, 0])
            verts_time = mode(dt, ttime, [0, 0, 0], profile=2)
            verts_time = verts_time/full_time
            if full_time > full_maxtime:
                full_maxtime = full_time
            if verts_time > verts_maxtime:
                verts_maxtime = verts_time
            full_steps_results.append(full_time)
            verts_steps_results.append(verts_time)
        full_mode_results_avg[num].append(np.mean(full_steps_results))
        verts_mode_results_avg[num].append(np.mean(verts_steps_results))
        full_mode_results_std[num].append(np.std(full_steps_results))
        verts_mode_results_std[num].append(np.std(verts_steps_results))



plt.title("Combined execution time by steps for runtime modes")
plt.ylabel("Execution Time (s)")
plt.xlabel("Steps")
for results, name in zip(full_mode_results_avg, all_modes_names_avg):
    plt.plot(steps_list, results, marker='o', markersize='5', linestyle='solid', label=name)
plt.axis([
    0, max_steps,
    0, full_maxtime
])

plt.legend(loc='best')
plt.savefig("profiler/avg_unified_singleaxis.png")

plt.clf()
plt.title("Standard Deviation between trial execution time by steps for runtime modes")
plt.ylabel("Standard Deviation (s)")
plt.xlabel("Steps")
for results, name in zip(full_mode_results_std, all_modes_names_std):
    plt.plot(steps_list, results, marker='o', markersize='5', linestyle='solid', label=name)
plt.axis([
    0, max_steps,
    0, full_maxtime
])

plt.legend(loc='best')
plt.savefig("profiler/std_unified_singleaxis.png")


plt.clf()
plt.title("Fraction of execution time in verts function by steps")
plt.ylabel("Fraction spent in 'verts'")
plt.xlabel("Steps")
for results, name in zip(verts_mode_results, all_modes_names):
    plt.plot(steps_list, results, marker='o', markersize='5', linestyle='solid', label=name)
plt.axis([
    0, max_steps,
    .6, 1,
])

plt.legend(loc='best')
plt.savefig("profiler/verts_unified_singleaxis.png")


        
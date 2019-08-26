from run_model import run_hylaa
from matplotlib import pyplot as plt
import numpy as np

linear = lambda mini, maxi, frac : mini + (maxi - mini) * frac

numDataTrials = 3

#dt config
dt_Min = .01
dt_Max = .05
dt_numSteps = 30
dt_StepFunc = linear
dt_Steps = [dt_StepFunc(dt_Min, dt_Max, step/dt_numSteps) for step in range(dt_numSteps)]
print("DT steps are:")
print(dt_Steps)

#total time config
ttime_Min = .4
ttime_Max = .4
ttime_numSteps = 1
ttime_StepFunc = linear
ttime_Steps = [ttime_StepFunc(ttime_Min, ttime_Max, step/ttime_numSteps) for step in range(ttime_numSteps)]
print("Total Time Steps are")
print(ttime_Steps)

#tuple result pairs (ttime, dt, runtime)
def printResult(r):
    print("\033[0;32;40m Run complete with: runtime avg={}\n".format(r[2]))

results = []

maxSimSteps = 100
#testing on dt and total
for totalTime in ttime_Steps:
    step_results = []
    for dt in dt_Steps:
        if totalTime / dt < maxSimSteps:
            print("\033[0;32;40m Running model with : ttime={}, dt={}\033[0;37;40m".format(totalTime, dt))
            stamp = "{}{}".format(totalTime, dt).replace('.','d')
            runTimeTrials = []
            for t in range(numDataTrials):
                runTimeTrials.append(run_hylaa(totalTime, dt, True, stamp))
            runTime = np.mean(runTimeTrials)
            result = (totalTime, dt, runTime)
            printResult(result)
            step_results.append(result)
        else:
            print("\033[2;33;40m SKIPPING model with : ttime={}, dt={}\n".format(totalTime, dt))
    results.append(step_results)
        
for ttime_set in results:
    ttime, dt, runtime = zip(*ttime_set)
    print("\033[0;37;40m Plotting with ttime={}".format(ttime))
    plt.title("DT Series (T={})".format(ttime[0]))
    plt.ylabel("runtime avg ({} trials)".format(numDataTrials))
    plt.xlabel("dt")
    plt.plot(dt, runtime, '--bo')
    plt.axis([0, dt_Max, 0, max(runtime)])
    plt.savefig("profiler/{}-g.png".format(ttime[0]))
    plt.clf()

print(results)

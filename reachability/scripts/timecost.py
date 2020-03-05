import pstats

from run_f1hylaa import run_hylaa


if __name__ == "__main__":
    print("Initializing Simulation")
    initialState = [0, 0, 0]
    dtList = [.02, .05, .1, .2, .5]
    ttime = 1
    output = "NONE"
    numTrials = 1

    top = {}
    #run profiling for each dt
    for dt in dtList:
        print("With {} timesteps".format(int(ttime/dt)))

        for x in range(numTrials):
            reach = run_hylaa(dt, ttime, initialState, output)
            print("Trial {}".format(x+1))
            with open('profiler/raw/profile_out_{}_{}.txt'.format(dt*1000,x), 'w+') as stream:
                p = pstats.Stats('profiler/prof/out_tmp.prof', stream=stream)
                p = p.sort_stats("tottime").print_stats()
        
        with open('profiler/raw/profile_out_{}_{}.txt'.format(dt*1000,0), 'r') as tmp:
            lines = tmp.readlines()
            top[str(dt)] = lines[7:9]

    for k,v in top.items():
        print("{} Steps:".format(int(ttime/float(k))))
        for func in v:
            print("\t{}".format(func))
            
        



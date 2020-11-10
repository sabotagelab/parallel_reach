import pandas as pd
import scipy.stats
import matplotlib.pyplot as plt
import numpy as np

import scipy.stats as st
def get_best_distribution(data):
    dist_names = ["norm", "exponweib", "weibull_max", "weibull_min", "pareto", "genextreme"]
    dist_results = []
    params = {}
    for dist_name in dist_names:
        dist = getattr(st, dist_name)
        param = dist.fit(data)

        params[dist_name] = param
        # Applying the Kolmogorov-Smirnov test
        D, p = st.kstest(data, dist_name, args=param)
        dist_results.append((dist_name, p))

    # select the best fitted distribution
    best_dist, best_p = (max(dist_results, key=lambda item: item[1]))
    # store the name of the best fit and its p value

    print("Best fitting distribution: "+str(best_dist))
    print("Best p value: "+ str(best_p))
    #print("Parameters for the best fit: "+ str(params[best_dist]))

    return best_dist, best_p, params[best_dist]

upstatus = "Offline"
csv_input = "raw/offline.csv"


columns = ["time", "steps", "mode", "verts"]
timing_data = pd.read_csv(csv_input, sep=",", names=columns)


print(timing_data[:6])

modes = timing_data['mode'].unique()
steps = timing_data['steps'].unique()
conf_interval_prop = .95

timing_stats = pd.DataFrame(
    columns=['steps', 'mode', 'full_avg', 'verts_avg', 'full_std', 'verts_std']
    )
 #steps, mode, full_avg, verts_avg, full_std, verts_std
import pdb
best_dists = []
for m in modes:
    dists = []
    timing_data_avg = []
    rows = timing_data.loc[timing_data['mode'] == m]
    for s in steps:
        full_data = rows.loc[(rows['steps'] == s) & (rows['verts'] == 0)]
        verts_data = rows.loc[(rows['steps'] == s) & (rows['verts'] == 1)]
        full_avg = full_data['time'].mean()
        full_std = full_data['time'].std()
        verts_avg = verts_data['time'].mean()
        verts_std = verts_data['time'].std()
        print(f"{full_std}")
        full_isnormal = scipy.stats.normaltest(full_data['time'])
        verts_isnormal = scipy.stats.normaltest(verts_data['time'])
        #dsma = np.asarray([np.mean(np.random.choice(full_data['time'], size=5)) for x in range(5000)])
        #dsmb = np.asarray([np.mean(np.random.choice(full_data['time'], size=15)) for x in range(5000)])
        #dsmc = np.asarray([np.mean(np.random.choice(full_data['time'], size=30)) for x in range(5000)])
        #print(f'MEANS: {np.mean(dsma)}\n{np.mean(dsmb)}\n{np.mean(dsmc)}')
        #print(f'STDS: {np.std(dsma)}\n{np.std(dsmb)}\n{np.std(dsmc)}')
        
        conf_size = scipy.stats.norm.ppf(conf_interval_prop)
        conf_low = full_avg - conf_size * full_std
        conf_high = full_avg + conf_size * full_std
        stats_list = [full_avg, verts_avg, full_std, verts_std]
        #dist_name, _, _ = get_best_distribution(full_data['time'])
        #dists.append(dist_name)
        #plt.clf()
        #plt.hist(full_data['time'], bins=20)
        #plt.savefig(f"profiler/timing_dists/dist_{m}_{s}.png")
        timing_stats.loc[len(timing_stats)] = [s, m] + stats_list
    
    best_dists.append(dists)

print("collated statistics")
print(best_dists)
#https://www.marsja.se/python-manova-made-easy-using-statsmodels/

        
plt.title("Combined execution time by steps for runtime modes")
plt.ylabel("Execution Time (s)")
plt.xlabel("Steps")
mode_data = {}
for m in modes: 
    mode_data[m] = timing_stats.loc[timing_stats['mode'] == m]


for mode in modes:
    data = mode_data[mode]['full_avg']
    plt.plot(steps, data, marker='o', markersize='5', linestyle='solid', label=mode)
plt.axis([
    0, max(steps),
    0, timing_stats['full_avg'].max()
])



plt.legend(loc='best')
plt.savefig("profiler/avg_unified_singleaxis.png")

CI_pval = .95
CI_zscore = st.norm.pdf(CI_pval)
for mode in modes:
    plt.clf()
    plt.title(f"Average Runtime With 95% CI for {mode} mode vs #steps")
    plt.ylabel("Runtime (s)")
    plt.xlabel("Steps")
    data = mode_data[mode]['full_avg']
    std = mode_data[mode]['full_std']
    plt.plot(steps, data, marker= 'o', markersize='5', linestyle='solid', label=mode)
    plt.fill_between(steps, data-std*CI_zscore, data+std*CI_zscore, color=[(.6,1,.6,.5)])
    plt.axis([
        0, max(steps),
        0, data.max()
    ])
    plt.legend(loc='best')
    plt.savefig(f"profiler/avg_{mode}_CI.png")

plt.clf()
plt.title("Standard Deviation between trial execution time by steps for runtime modes")
plt.ylabel("Standard Deviation (s)")
plt.xlabel("Steps")
for mode in modes:
    data = mode_data[mode]["full_std"]
    plt.plot(steps, data, marker='o', markersize='5', linestyle='solid', label=mode)
plt.axis([
    0, max(steps),
    0, data.max()
])

plt.legend(loc='best')
plt.savefig("profiler/std_unified_singleaxis.png")


plt.clf()
plt.title("Fraction of execution time in verts function by steps")
plt.ylabel("Fraction spent in 'verts'")
plt.xlabel("Steps")
for mode in modes:
    verts_fractions = mode_data[mode]['verts_avg']/mode_data[mode]['full_avg']
    plt.plot(steps, verts_fractions, marker='o', markersize='5', linestyle='solid', label=mode)
plt.axis([
    0, max(steps),
    .6, 1,
])

plt.legend(loc='best')
plt.savefig("profiler/verts_unified_singleaxis.png")


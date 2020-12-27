This repository is a public version of the OSU F1/10 research made avaliable for review 

# NFM 2021 Artifacts #
Artifacts relevant to reachability profiling are listed below.


in `reachability/scripts/raw`
- *offline_FINAL.csv* contains the final offline profiling data
- *online_FINAL.csv* contains the final online profling data

in `reachability/scripts`
- *zono_node.py* defines the ros node for online reachability
- *F1QuickZono.py* defines the class used to initialize and call Quickzono Reachability
- *profiler_unified.py* performs offline profiling
- *graph_maker.py* generates graphs, including those in the paper
- *model_hardcode.py* computes linearized dynamics at a point based on identified kinematics

in `reachability/config`
- *quickzono.yaml* defines parameters for online reachability computation

Other files include utilities, testing output, additional graphical analysis, and deprecated/unused code.
# Overview #

This repository includes mapping, localization, planning, and control algorithms along with hardware level interface packages designed to work with the F1/10 platform.
Used together, these tools enable fully autonomous navigation and racing for the vehicle.

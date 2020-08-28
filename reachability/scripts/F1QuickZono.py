# library imports
from functools import partial
import importlib
#from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
#import dill
from multiprocessing import Pool

#local imports
from nl_dynamics import F1Dynamics

from quickzonoreach.zono import get_zonotope_reachset
from quickzonoreach.zono_projection import ZP_TYPE, get_ZP_instance

#wrapper to run quickzono repeatedly with different inputs
class F1QuickZono:
    def __init__(self):

        # ------------------ Semi-Static Configuration ----------------------
        self.dt = None          #dt from MPC metdata
        self.ttime = None
        self.num_steps = None

        self.input_uncertainty = []
        self.state_uncertainty = []

        self.model = None       #dynamics abstraction object

        #TODO automatic or otherwise validated variability
        # ------------------ Per-run Configuration ----------------------
        self.predictions = None #predictions from MPC 3d array : [ dt[ state[x,y,psi], inputs[d,v] ] ]

        self.init_box = []
        self.a_mat_list = []
        self.b_mat_list = []
        self.input_box_list = []
        self.dt_list = []
        self.save_list = []


        self.ZP = None 


    def set_model_params(self, state_uncertainty, input_uncertainty, dynamics_module="kinematics_model", runtime_mode=ZP_TYPE.CPU):
        model_gen = importlib.import_module(dynamics_module)
        self.modeList = []
        self.state_uncertainty = state_uncertainty
        self.input_uncertainty = input_uncertainty
        self.model = model_gen.getModel()
        self.model.setInputUncertainty(input_uncertainty)
        
        
        self.ZP = get_ZP_instance(runtime_mode)
        return 1
    
    def run(self, predictions, profile=False):
        self.predictions = predictions[:self.num_steps]

        self.quick = True
        self.a_mat_list = []
        self.b_mat_list = []
        self.input_box_list = []


        #profile.runctx('resultprof = self.run_zono(initialBox, core)', globals(), locals(), filename="profiler/prof/out_tmp.prof")
        #result = locals()['resultprof']
        self.make_dynamics()
        self.make_init(self.predictions[0][0])
        zonos = self.run_zono()


        #do projection for safety analysis in 2d
        if profile:
            return self.ZP.verts_timeit(zonos)
        else:
            return self.ZP.verts(zonos)

    def run_zono(self):
        zonotopes = get_zonotope_reachset(self.init_box, self.a_mat_list, self.b_mat_list, self.input_box_list, self.dt_list,
            save_list=self.save_list, quick=self.quick)
        return zonotopes

    def make_settings(self, dt, total):
        'make the reachability settings object'
        self.dt = dt
        self.ttime = total
        self.num_steps = int(self.ttime/self.dt)
        self.dt_list = [self.dt] * (self.num_steps)
        self.save_list = [True if x%1==0 else False for x in range(self.num_steps+1)]


    def make_init(self, initialState):
        'make the initial states'

        init_box = []
        lin_box = [[1, 1]] * 3
        time_extra = [[0, 0]] * 2

        #initial state = state uncertainty + linear var uncertainty + time uncertainty(0)
        #we assume the uncertainty is symettrical for x, y, yaw
        for i,s in zip(initialState, self.state_uncertainty):
            init_box.append([ 
                i - s,
                i + s
            ])


        init_box += lin_box
        self.init_box = init_box

        return init_box

    def make_input_box(self, inputs):
        input_box = [[0, 0]] * len(inputs)
        for s in range(len(inputs)):
            input_box[s] = (
                inputs[s] - self.input_uncertainty[s],
                inputs[s] + self.input_uncertainty[s]
            )
        
        return input_box 

    def make_dynamics(self):

        step = 0
        for state, inputs in self.predictions[:self.num_steps]:

            #get the dynamics linearized around this state/input set
            dynamics = self.model.linearized_dynamics(state[:3], inputs, self.dt)

            self.a_mat_list.append(dynamics['A'])
            self.b_mat_list.append(dynamics['B'])
            
            input_box = self.make_input_box(inputs)
            self.input_box_list.append(input_box)

            step += 1



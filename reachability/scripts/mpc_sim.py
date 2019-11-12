import rospy

class MPC_Sim:
    def __init__:
        self.nlDynamics = F1Dynamics()
        self.stepFunc = partial(nlDynamics.frontStep, nlDynamics)
        self.inputFunc = 

        self.prediction_pub_topic = rospy.get_param("mpc_topic")
        
        self.dt = rospy.get_param("dt", .1)
        self.totalTime rospy.get_param("total_time", 2)
        self.simulation_period = 100 #ms between publish events

    def 

    def simulate(self, state):
        sim = simulator.ModelSimulator(dt, total, initialState, self.stepFunc, self.inputFunc, headless)
        predictions = sim.simulate()
        return predictions
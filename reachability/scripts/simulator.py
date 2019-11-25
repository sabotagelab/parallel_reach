import math

#simulator for generating state approximations of nonlinear model
# --used to create approximation used to initialize HYLAA states
# stepFunc is as follows: (state can be any type, input must agree with inputfunc)

def exampleStepFunc(state, input):
    state[0] = state[0]

    return state

#inputfunc is as follows:

def exampleInputFunc(time):
    return math.cos(time) * 5 #constant modulation for this example

class ModelSimulator():

    def __init__(self, dt, totalTime, initialState, stepFunc=None, inputFunc=None, headless=False):
        self.dt = dt
        self.totalTime = totalTime
        assert dt < totalTime, "dt >= total time - undefined configuration"

        self.stepFunc = stepFunc
        self.inputFunc = inputFunc
        self.currentState = initialState

        self.verbose = not headless
    
    def simulate(self, stepFunc=None, inputFunc=None):
        if stepFunc:
            self.stepFunc = stepFunc
        assert self.stepFunc != None , "Step Function must be specified"

        if inputFunc:
            self.inputFunc = inputFunc 
        assert self.inputFunc != None , "Input Function must be specified"

        time = 0
        steps = int(self.totalTime / self.dt)

        if self.verbose:
            print("Simulating {} steps with dt={}".format(steps, self.dt))

        #this will store all the generated states
        self.stateList = [(self.currentState + [time], self.inputFunc(time))]

        if self.verbose:
            print("Initial state at time {}: \n\t{}".format(time, self.stateList[-1]))

        for step in range(steps):
            time += self.dt
            inputs = self.inputFunc(time)

            self.currentState = self.stepFunc(self.currentState, inputs, self.dt)
            self.stateList.append((
                self.currentState,
                inputs
            ))
            
            if self.verbose:
                print("Step {} at time {}: \n\t{}".format(step+1, time, self.stateList[-1]))
        
        return self.stateList



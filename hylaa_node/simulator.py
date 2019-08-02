import math

#simulator for generating state approximations of nonlinear model
# --used to create approximation used to initialize HYLAA states
# stepFunc is as follows: (state can be any type, input must agree with inputfunc)

def exampleStepFunc(state, input):
    state[0] = state[0]

    return state

#inputfunc is as follows:

def exampleInputFunc(time):
    return cos(time) * 5 #constant modulation for this example


#for this we will define state as:
#state is [x, y, e]
def frontDynamicsStepFunc(state, input, dt):


#input is [d, v]
def constantInputFunc(time):
    return [0, 1]


class ModelSimulator():

    def __init__(self, dt, totalTime, initialState, stepFunc=None, inputFunc=None):
        self.dt = dt
        self.totalTime = totalTime
        assert dt < totalTime, "dt >= total time - undefined configuration"

        self.stepFunc = stepFunc
        self.currentState = initialState
    
    def simulate(self, stepFunc=None, inputFunc=None):
        if stepFunc:
            self.stepFunc = stepFunc
        assert self.stepFunc not None , "Step Function must be specified"

        if inputFunc:
            self.inputFunc = inputFunc 
        assert self.inputFunc not None , "Input Function must be specified"

        time = 0
        self.currentState = None
        steps = self.totalTime / self.dt

        #this will store all the generated states
        self.stateList = [initialState]

        for step in range(steps):
            inputs = self.inputFunc(time)
            self.currentState = stepFunc(self.currentState, inputs, self.dt)
            self.stateList.append((
                self.currentState + [time],
                inputs
            ))
            time += dt
        
        return self.stateList



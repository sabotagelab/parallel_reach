import math

#model representing the linearized dynamics 
# and bounds of a system 
# for use with hylaa

# WHY
# ++
#abstracting this makes it easier to test different dynamics models 
# through rospy settings integration
# --
# makes it slightly more difficult to add additional dynamics fields or matrices
#  for use in reachability algorithm


class Model:
    def __init__(self):
        self.constants = [.32, 1]
        self.input_uncertainty = [0] * 2
    
    def setInputUncertainty(self, input_uncertainty):
        print(input_uncertainty)
        assert(len(input_uncertainty) == len(self.input_uncertainty))
        self.input_uncertainty = input_uncertainty

    def linearized_dynamics(self, state, inputs, dt):
        #current = state+inputs+self.constants+[dt]+self.input_uncertainty
        x, y, p = (val for val in state)
        v, d,  = (val for val in inputs)
        L, d0 = (val for val in self.constants)
        # i1, i2 = input_uncertainty[1](val for val in self.input_uncertainty)
        p_d = p + d*.5
        sin_p_d = math.sin(p_d)
        cos_p_d = math.cos(p_d)
        #sin_p_d = math.sin(p_d)
        #cos_p_d = math.cos(p_d)
        sin_d = math.sin(d)
        cos_d = math.cos(d)

        A = [
            [0, 0, -v*sin_p_d, d*v*sin_p_d + p*v*sin_p_d, 0, 0],
            [0, 0, v*cos_p_d, 0, -d*v*cos_p_d - p*v*cos_p_d, 0],
            [0, 0, 0, 0, 0, -d*v*cos_d/L],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]
        ]

        B = [
            [cos_p_d, -v*sin_p_d],
            [sin_p_d, v*cos_p_d],
            [sin_d/L, v*cos_d/L],
            [0, 0],
            [0, 0],
            [0, 0]
        ]
        return {
            "A" : A,
            "B" : B
        }


def getModel():
    return Model()

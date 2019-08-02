from sympy import sin, cos, symbols

#nonlinear dynamics represented with sympy
class F1Dynamics:
    def __init__(self):
        self.setupEquations()
    
    def setupEquations(self):
        x, y, e, d, v = symbols("x, y, e, d, v, dt")
        self.frontDynamics = [
            x + dt * v * cos(e + d),
            y + dt * v * sin(e + d),
            e + dt * (v/L) * sin(d)
        ]

        self.rearDynamics = [
            x + dt * v * cos(e + d),
            y + dt * v * sin(e + d),
            e + dt * (v / L) * tan(d)
        ]

    def frontStep(self, state, input, dt):
        return self.frontDynamics.subs(state + input + dt)
    
    def rearStep(self, state, input, dt):
        return self.rearDynamics.subs(state + input + dt)
    
    def getDims(self):
        return {
            "state" : [1, 3],
            "input" : [1, 2]
        }

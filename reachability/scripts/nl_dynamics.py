from sympy import sin, cos, tan, symbols, Matrix

#nonlinear dynamics represented with sympy
class F1Dynamics:
    def __init__(self):
        self.setupEquations()
    
    def setupEquations(self):
        L = .32
        x, y, e, v, d, dt = symbols("x y e v d dt")
        self.varSym = [x, y, e, v, d, dt]
        self.frontDynamics = Matrix([
            x + dt * v * cos(e + d),
            y + dt * v * sin(e + d),
            e + dt * (v/L) * sin(d)
        ])

        self.rearDynamics = Matrix([
            x + dt * v * cos(e + d),
            y + dt * v * sin(e + d),
            e + dt * (v / L) * tan(d)
        ])

    def frontStep(self, caller, state, input, dt):
        varSub = list(zip(self.varSym, state + input + [dt]))
        return list(self.frontDynamics.subs(varSub).evalf())
    
    def rearStep(self, caller, state, input, dt):
        varSub = list(zip(self.varSym, state + input + [dt]))
        return list(self.rearDynamics.subs(varSub).evalf())
    
    def getDims(self):
        return {
            "state" : [1, 3],
            "input" : [1, 2]
        }

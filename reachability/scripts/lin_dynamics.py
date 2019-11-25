from sympy import sin, cos, tan, sec, symbols, Matrix

#linear dynamics represented with sympy
class F1Dynamics_Linear:

    def __init__(self, initState):
        self.e0 = initState[2]
        self.d0 = 0
        self.setupEquations()
    
    def setupEquations(self):
        L = .32
        x, y, e, v, d, dt, e0, d0 = symbols("x y e v d dt e0 d0")
        self.varSym = [x, y, e, v, d, dt, e0, d0]
        self.frontDynamics = Matrix([
            x + dt * v * (cos(e0+d0) - (e-e0)*sin(e0+d0)-(d-d0)*sin(d0+e0)),
            y + dt * v * (sin(e0+d0) + (e-e0)*cos(e0+d0)+(d-d0)*cos(d0+e0)),
            e + dt * v * (sin(d0)+(d-d0)*cos(d0)) * (1/L)
        ])

        self.rearDynamics = Matrix([
            x + dt * v * (cos(self.e0)-(e-self.e0)*sin(self.e0)),
            y + dt * v * (sin(self.e0)+(e-self.e0)*cos(self.e0)),
            e + dt * (v / L) * (tan(0)+(d)*sec(0)*sec(0))
        ])

        self.hylaaDynamics = Matrix([
            x + dt * v * (cos(e0+d0) - e*sin(e0+d0) - d * sin(e0+d0) ),
            y + dt * v * (sin(e0 + d0) + e*cos(e0+d0) + d* cos(e0+d0)),
            e + dt * v * (1/L) * (sin(d0) + d*cos(d0))
        ])

    def frontStep(self, caller, state, input, dt):
        varSub = list(zip(self.varSym, state + input + [dt, self.e0, self.d0]))
        res = list(self.frontDynamics.subs(varSub).evalf())
        self.e0 = state[2]
        self.d0 = input[1]
        return res
    
    def rearStep(self, caller, state, input, dt):
        varSub = list(zip(self.varSym, state + input + [dt]))
        self.e0 = state[2]
        return list(self.rearDynamics.subs(varSub).evalf())
    
    #issue #1 is out of sync initial state (e0, d0)
    #issue #2 is flipped signs
    #issue #3 is 
    def hylaaStep(self, caller, state, input, dt):
        varSub = list(zip(self.varSym, state + input + [dt, self.e0, self.d0]))
        res = list(self.hylaaDynamics.subs(varSub).evalf())
        self.e0 = state[2]
        self.d0 = input[1]
        return res

    def getDims(self):
        return {
            "state" : [1, 3],
            "input" : [1, 2]
        }

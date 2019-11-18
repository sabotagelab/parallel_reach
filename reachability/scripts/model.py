from sympy import sin, cos, tan, Matrix, symbols, pprint

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
    def __init__(self, symbols, constants, A_jac, B_jac, bounds_rhs):
        self.symbols = symbols
        self.constants = constants
        self.A = A_jac
        self.B = B_jac

        self.bounds_rhs = bounds_rhs
        self.bounds_mat = self.standardBounds()
        self.input_uncertainty = [0] * self.B.shape[1]
        
        self.augmentDynamics()

    def standardBounds(self):
        return [
            [1, 0, 0, 0],[-1, 0, 0, 0],
            [0, 1, 0, 0],[0, -1, 0 , 0],
            [0, 0, 1, 0], [0, 0, -1, 0],
            [0, 0, 0, 1], [0, 0, 0, -1]
        ]

    def setInputUncertainty(self, input_uncertainty):
        assert(len(input_uncertainty) == len(self.input_uncertainty))
        self.input_uncertainty = input_uncertainty

    #augment dynamics with row/col for time + time accumulation
    def augmentDynamics(self):
        a = self.A
        arow = lambda : a.shape[0]
        acol = lambda : a.shape[1]
        b = self.B
        brow = lambda : b.shape[0]
        bcol = lambda : b.shape[1]

        #insert two columns of 0's
        a = a.col_insert(acol(), Matrix([0] * arow()))
        a = a.col_insert(acol(), Matrix([0] * arow()))
        #insert row with trailing one (time) and of 0's (time augment)
        a = a.row_insert(arow(), Matrix([[0] * (acol()-1) + [1]]))
        a = a.row_insert(arow(), Matrix([[0] * acol()]))
        self.A = a

        #augment B with rows of 0's
        b = b.row_insert(brow(), Matrix([[0] * bcol()])) #time
        b = b.row_insert(brow(), Matrix([[0] * bcol()])) #time accum
        self.B = b
    
    def linearized_dynamics(self, state, inputs):
        current = list(zip(self.symbols, state+inputs+self.constants+self.input_uncertainty))
        return {
            "A" : self.A.subs(current).evalf(),
            "B" : self.B.subs(current).evalf(),
            "bounds_mat" : self.bounds_mat,
            "bounds_rhs" : self.bounds_rhs.subs(current).evalf(),
            "current" : current
        }


import sympy
import rospy

#model representing the linearized dynamics 
# and bounds of a system 
# for use with hylaa
class Model:
    def __init__(self, symbols, constants A_jac, B_jac, input_bounds=None):
        self.symbols = symbols
        self.constants = constants
        self.A = A_jac
        self.B = B_jac

        self.input_bounds = input_bounds
        if not input_bounds:
            input_bounds = self.standardBounds


    #augment dynamics with row/col for time + time accumulation
    def augmentDynamics(self)
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
        current = list(zip(self.variables, state+inputs+self.constants))
        return {
            "A" : self.A.subs(current).evalf(),
            "B" : self.B.subs(current).evalf(),
            "bounds_mat" : self.bounds_mat.subs(current).evalf(),
            "bounds_rhs" : self.bounds_rhs.subs(current).evalf()
            "current" : current
        }


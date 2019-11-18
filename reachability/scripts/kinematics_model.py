from model import Model
from sympy import sin, cos, tan, Matrix, symbols

def getModel():
    # ---------- STATE (X) -----------
    #variables
    x, y, p = symbols('x y p')
    stateVars = [x, y, p]


    # ---------- INPUT (U) -----------
    #variables
    v, d, e0, d0 = symbols('v d e0 d0')
    inputVars = [v, d, e0, d0]

    #bounds
    vs, ds, e0s, d0s = symbols('vs ds e0s d0s')
    boundVars = [vs, ds, e0s, d0s]
    bounds_rhs = Matrix([
        v + vs, -(v - vs),
        d + ds, -(d - ds),
        e0, -e0,
        d0, -d0
    ])

    # ---------- CONSTANTS -----------
    #variables
    L = symbols('L')
    L0 = .32
    constantVals = [L0]
    constantSym = [L]

    nl_equations = Matrix([
        v * cos(p),
        v * sin(p),
        (v/L) * sin(d)
    ])



    #TODO cleaner manual jacobian changes using substitution
    #   half-working commented in model_OLD.py

    #substitutions for generating proper linearizations after evaluation
    A_jacobian = nl_equations.jacobian(stateVars)
    B_jacobian = nl_equations.jacobian(inputVars)

    matidx = lambda shape, r, c : shape[1] * r + c
    idx_A = lambda r, c : matidx(A_jacobian.shape, r, c)
    idx_B = lambda r, c : matidx(B_jacobian.shape, r, c)


    B_jacobian[idx_B(0,2)] = -1 * A_jacobian[idx_A(0,2)] #e0 for x
    B_jacobian[idx_B(1,2)] = -1 * A_jacobian[idx_A(1,2)] #e0 for y
    B_jacobian[idx_B(0,3)] = -1 * B_jacobian[idx_B(0,1)] #d0 for x
    B_jacobian[idx_B(1,3)] = -1 * B_jacobian[idx_B(1,1)] #d0 for y
    B_jacobian[idx_B(2,3)] = -1 * B_jacobian[idx_B(2,1)] #d0 for e

    all_vars = stateVars + inputVars + constantSym + boundVars
    return Model(all_vars, constantVals, A_jacobian, B_jacobian, bounds_rhs)
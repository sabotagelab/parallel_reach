from model import Model
from sympy import sin, cos, tan, Matrix, symbols, MatrixSymbol, BlockMatrix, eye, diag, zeros, pprint, init_printing

def getModel():
    # ---------- STATE (S) -----------
    #variables
    x, y, p = symbols('x y  p')
    S_k = [x, y, p]


    # ---------- INPUTS (U) -----------
    #variables
    v, d = symbols('v d')
    U_k = Matrix([v, d])

    # ---------- STATE LINEARIZATION (Sbar)  -----------
    xb, yb, pb = symbols('xb yb pb')
    Sbar = Matrix([xb, yb, pb])

    # ---------- INPUT LINEARIZATION (Ubar)  -----------
    vb, db = symbols('vb db')
    Ubar = Matrix([vb, db])

    # ---------- BOUNDS -----------
    vs, ds = symbols('vs ds')
    boundVars = Matrix([vs, ds])
    bounds_rhs = Matrix([
        vb + vs, -(vb - vs),
        db + ds, -(db - ds)
    ])

    # ---------- CONSTANTS -----------
    #variables
    L, dt = symbols('L dt')
    L0 = .32
    constantVals = [L0]
    constantSym = Matrix([L, dt])

    # F(Sbar, UBar)
    F = Matrix([
        vb * cos(pb + db),
        vb * sin(pb + db),
        (vb/L) * sin(db)
    ])

    #find partials
    dFdS = F.jacobian(Sbar)
    dFdU = F.jacobian(Ubar)

    #construct constant matrix
    C = F - (dFdS * Sbar) - (dFdU * Ubar)

    #define final state/inputs
    S = S_k + [1] * C.shape[0]
    U = U_k

    #make pieces of A-matrix and combine (state dynamics)
    M1 = dFdS  #+ eye(Sbar.shape[0])
    M2 = diag(*C)
    M3 = zeros(Sbar.shape[0])
    M4 = zeros(C.shape[0])
    
    A_mat = Matrix(BlockMatrix([
        [M1, M2],
        [M3, M4]
    ]))

    #Assemble B-matrix (input dynamics)
    B_mat =  Matrix( [
        dFdU,
        zeros(C.shape[0], U.shape[0])
    ])

    init_printing()
    #pprint(F)
    #pprint(dFdU * Ubar)
    #pprint(F - dFdU * Ubar)
    #pprint(dFdS * Sbar)
    #pprint(C)
    pprint(A_mat)
    pprint(B_mat)


    all_vars = list(Sbar) + list(Ubar) + list(constantSym) + list(boundVars)
    return Model(all_vars, constantVals, A_mat, B_mat, bounds_rhs)
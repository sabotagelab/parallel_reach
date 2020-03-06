from model import Model
import theano as th
import theano.Tensor as T
import numpy as np

def getModel():
    # ---------- STATE (S) -----------
    #variables
    x = T.dscalar('x')
    y = T.dscalar('y')
    p = T.dscalar('p')
    S_k = T.dvector([x, y, p])


    # ---------- INPUTS (U) -----------
    #variables
    v = T.dscalar('v')
    d = T.dscalar('d')
    U_k = T.dvector([v, d])

    # ---------- STATE LINEARIZATION (Sbar)  -----------
    xb = T.dscalar('xb')
    yb = T.dscalar('yb')
    pb = T.dscalar('pb')
    Sbar = T.dvector([xb, yb, pb])

    # ---------- INPUT LINEARIZATION (Ubar)  -----------
    vb = T.dscalar('vb')
    db = T.dscalar('db')
    Ubar = T.dvector([vb, db])

    # ---------- BOUNDS -----------
    vs = T.dscalar('vs')
    ds = T.dscalar('ds')
    boundVars = T.dvector([vs, ds])

    # ---------- CONSTANTS -----------
    #variables
    L = T.dscalar('L')
    dt = T.dscalar('dt')
    L0 = .32
    constantVals = [L0]
    constantSym = T.dvector([L, dt])


    # F(Sbar, UBar)
    F = T.dmatrix([
        vb * cos(pb + db),
        vb * sin(pb + db),
        (vb/L) * sin(db)
    ])

    #find partials
    dFdS = th.gradient.jacobian(F, Sbar)
    dFdU = th.gradient.jacobian(F, Ubar)

    #construct constant matrix
    C = F - (dFdS * Sbar) - (dFdU * Ubar)

    #define final state/inputs
    S = S_k + [1] * C.shape[0]
    U = U_k

    #make pieces of A-matrix and combine (state dynamics)
    M1 = dFdS  
    M2 = np.diag(C)
    M3 = np.zeros((Sbar.shape[0], Sbar.shape[0]))
    M4 = np.zeros((C.shape[0], C.shape[0]))
    
    A_mat_func = T.stack([
        T.stack([M1, M2], axis=2),
        T.stack([M3, M4], axis=2)
    ], axis=1)
    A_mat = th.function([
        [M1, M2],
        [M3, M4]
    ], A_mat_func )

    #Assemble B-matrix (input dynamics)
    B_mat_func = T.stack([p1, p2], axis=1)
    B_mat =  th.function([
        dFdU,
        np.zeros((C.shape[0], U.shape[0]))
    ], B_mat_func)


    all_vars = list(Sbar) + list(Ubar) + list(constantSym) + list(boundVars)
    return Model(all_vars, constantVals, A_mat, B_mat, bounds_rhs)

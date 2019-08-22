from sympy import sin, cos, Matrix, symbols, pprint

# ---------- STATE (X) -----------
#variables
x, y, p = symbols('x y p')
stateVars = [x, y, p]

#initial values 
x = 0   #x coord
y = 0   #y coord
p0 = 0  #yaw
# --------------------------------

# ---------- INPUT (U) -----------
#variables
v, d = symbols('v d')
inputVars = [v, d]

vMax = 1.2
vMin = .8
dMax = 3.14/4 + .2
dMin = 3.14/4 - .2
#bound
bounds_mat = [
    [1, 0],[-1, 0],
    [0, 1],[0, -1]
]

#bounds_rhs = [
    #dMax, -dMin,
    #vMax, -vMin
#]
bounds_rhs = [
    vMax, -vMin,
    dMax, -dMin
]

# for example, iff vMin <= v <= vMax

#initial values
d0 = 0  #steering angle
v0 = 0  #velocity

# --------------------------------


# ---------- CONSTANTS -----------
#variables
L = symbols('L')
constantSym = [L]

#values
L0 = .32 #wheelbase in m
constants = [L0]
# --------------------------------

#define matrices
rear_mat = Matrix([v*cos(p), v*sin(p), (v/L)*d])
front_mat = Matrix([v*cos(p + d), v*sin(p + d), (v/L)*sin(d)])

def showJacobianSet(js, lb):
    for j,l in zip(js, lb):
        print(l)
        pprint(j)
    print("\n")

labels = [
    "\n------X------",
    "\n------U------"]

#calculate jacobians
rearJacobian = (
    rear_mat.jacobian(stateVars),   #X
    rear_mat.jacobian(inputVars)    #U
)

frontJacobian = (
    front_mat.jacobian(stateVars),  #X
    front_mat.jacobian(inputVars)   #U
)

usingDynamics = frontJacobian

#3 X 2 matrix that does something
C = [
    [ 1, 0, 0, 0 ],
    [ 0, 1, 0, 0 ],
    [ 0, 0, 1, 0 ],
    [ 0, 0, 0, 1 ]
]
def getMatrices(state, inputs):
    current = list(zip(stateVars + inputVars + constantSym, state + inputs + constants))
    return {
        "A" : usingDynamics[0].subs(current).evalf(),
        "B" : usingDynamics[1].subs(current).evalf(),
        "C" : C,
        "bounds_mat" : bounds_mat,
        "bounds_rhs" : bounds_rhs,
        "current" : current
    }

def getTimeAugmentMatrices(state, inputs):
    state = state[:-1]
    mat = getMatrices(state, inputs)
    a = mat["A"]
    arow = lambda : a.shape[0]
    acol = lambda : a.shape[1]
    b = mat["B"]
    brow = lambda : b.shape[0]
    bcol = lambda : b.shape[1]

    #insert two columns of 0's
    a = a.col_insert(acol(), Matrix([0] * arow()))
    a = a.col_insert(acol(), Matrix([0] * arow()))
    #insert row with trailing one (time) and of 0's (time augment)
    a = a.row_insert(arow(), Matrix([[0] * (acol()-1) + [1]]))
    a = a.row_insert(arow(), Matrix([[0] * acol()]))
    mat["A"] = a

    #augment B with rows of 0's
    b = b.row_insert(brow(), Matrix([[0] * bcol()]))
    b = b.row_insert(brow(), Matrix([[0] * bcol()]))
    mat["B"] = b

    return mat

#pprint(getTimeAugmentMatrices([0, 0, 0], [0, 1]))

#pprint(getMatrices([0, 0, 0], [1, 3.14/8]))
#print("\nREAR JACOBIAN:")
#showJacobianSet(rearJacobian, labels)

#print("\nFRONT JACOBIAN:")
#showJacobianSet(frontJacobian, labels)





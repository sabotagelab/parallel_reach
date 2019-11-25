syms xp yp ep vp x y dt d0 e0 d e v L a

% d0 is initial steering angle
% e0 is initial yaw
% d is variable steering angle
% e is constant (previos) yaw
% v is constant (previous) velocity
% L is constant wheelbase length
% a is variable acceration

% only the steering and yaw will vary
d0 = 0
e0 = 0
L = .325

vars = [ d e ]

eq = [ v*(cos(e0+d0)-(e-e0)*sin(e0)-(d-d0)*sin(e0+d0)),
    v*(sin(e0+d0) - (e-e0)*cos(e0)+(d-d0)*cos(e0+d0)),
    (v/L)*sin(d0)+(d-d0)*cos(d0),
    1 ];

[A, b] = equationsToMatrix(eq, vars)
    
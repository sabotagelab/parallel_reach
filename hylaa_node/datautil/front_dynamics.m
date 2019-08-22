epsi0 = 0;
delta0= 0;
v0= 0.5;
steps = 20;  %steps for forward simulations
accel = 0.5;
dt = 0.1;
L=0.325; %wheelbase
delta = deg2rad(30);  %steering angle 30 degrees 
delta0 = deg2rad(30);
x= zeros(steps,1);
y= zeros(steps,1);
shi = zeros(steps,1);
v = zeros(steps,1);
horizon = 1:1:steps;
x(1)= 0;
y(1)=0;
shi(1) = epsi0;
v(1) = v0;
%Non linearized kinematics model
for k = 1:steps-1
    x(k+1) = x(k) + dt*v(k)*cos(shi(k)+delta);
    y(k+1) = y(k) + dt*v(k)*sin(shi(k)+delta);
    shi(k+1) = shi(k) + dt*(v(k)/L)*sin(delta);
    v(k+1) = v(k) + dt*accel;
end
plot(x,y)
accel = 0.5;
xl= zeros(steps,1);
yl= zeros(steps,1);
shil = zeros(steps,1);
vl = zeros(steps,1);
xl(1)= 0;
yl(1)=0;
shil(1) = epsi0;
vl(1) = v0;
%Linarized version  for kinematic model with front wheel reference 
for k = 1:steps-1
    xl(k+1) = xl(k) + dt*vl(k)*(cos(epsi0+delta0)-(shil(k)-epsi0)*sin(epsi0)-(delta-delta0)*sin(epsi0+delta0));
    yl(k+1) = yl(k) + dt*vl(k)*(sin(epsi0+delta0)+(shil(k)-epsi0)*cos(epsi0)+(delta-delta0)*cos(epsi0+delta0));
    shil(k+1) = shil(k) + dt*(vl(k)/L)*(sin(delta0)+(delta-delta0)*cos(delta0));
    vl(k+1) = vl(k) + dt*accel;
    
    epsi0=shil(k); %used for testing successive linearization
end
hold on
plot(xl,yl)
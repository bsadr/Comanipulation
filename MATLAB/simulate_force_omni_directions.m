function y = simulate_force_omni_directions(data, dt) 
%simulate_force Returns the simulated force for x
%   data: time, x, y, theta, vx, vy, vtheta
%   y: t; x; xdot f; xd'; xd_dot'


% X = data(:,2:end);
X = data(:,2:7);
T = data(:,1);

%%%%%%%%%%%%% Real Object Properties
m = 1; % mass, kg 
I = 1; % moment of inertia

t0 = T(1,1); 
tf   = T(end,1);  %final time in seconds. 
time_span = t0:dt:tf;
    
x0 = X(1,1:6)'; % initial state [x;y;theta;xdot;ydot;thetadot] 
xf = X(end,1:6)'; % final state 

%%%%%%%%%%%%% Real Object
m = 1; % mass, kg 
I = 1; % moment of inertia
C = [0.1; 0.1; 0.1];
A = [0         0         0         1         0         0;
    0          0         0         0         1         0;
    0          0         0         0         0         1;   
    0          0         0        -C(1)/m    0         0;
    0          0         0         0        -C(2)/m    0;  
    0          0         0         0         0        -C(1)/I];
B = [0   0    0;
    0    0    0
    0    0    0
    1/m  0    0;
    0    1/m  0;
    0    0    1/I];
%%%%%%%%%%%%%%%%%%%%%%%%%% Solve for x and h (y=[x(motion);h(generalized force)])
y = [];
[t,xn]=ode23(@rhs,time_span,x0); 
[~,idu] = unique(y(1,:));
unique_y = y(:,idu);
tmp = t';
for i = 2:size(y,1)
    yq = interp1(unique_y(1,:), unique_y(i,:),t');
    tmp =[tmp; yq];
end
y = tmp';
%clear tmp unique_y idu

%%%%%%%%%%%%%%%%%%%%%%%%%%% Aux. Functions
function xdot=rhs(t,x) 
    f = force(t, x);
    xd=interp1(T, X, t);
    y = [y [t; x; f; xd']];
    xdot=A*x+B*f;
end

function [f]=force(t, x) 
    xd=interp1(T, X, t);
    e = xd(1:6)' - x;
    w1=.5;
%     si1=.707;
%     si2=.707;
%     si3=.707;
    si1=.7;
    si2=.7;
    si3=.7;
    k1=[w1*w1 2*w1*si1];
    k2=[w1*w1 2*w1*si2];
    k3=[w1*w1 2*w1*si3];
    K = [k1(1) 0     0     k1(2) 0     0;
         0     k2(1) 0     0     k2(2) 0;
         0     0     k3(1) 0     0     k3(2)];
%     p=[-1 -2 -3 -4 -5 -6];
%   K=place(A,B,p); 
%     mye = eig(A-B*K)
    f=K*e;
end

end

function y = simulate_force(data, dt) 
%simulate_force Returns the simulated force for x
%   x: time, x, y, theta, vx, vy, vtheta

X = data(:,2:end);
T = data(:,1);

%%%%%%%%%%%%% Real Object Properties
m = 1; % mass, kg 
I = 1; % moment of inertia

t0 = T(1,1); 
tf   = T(end,1);  %final time in seconds. 
time_span = t0:dt:tf;
    
x0 = X(1,1:6)'; % initial state [x;y;theta;xdot;ydot;thetadot] 
xf = X(end,1:6)'; % final state 

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
    %xdot=A*x+B*f;
    v = sqrt(x(4)*x(4)+x(5)*x(5));
    c = cos(x(3));
    s = sin(x(3));
    xdot=[  v*c;
            v*s;
            x(6);
            f(1)/m*c;
            f(1)/m*s;
            f(2)/I];
        angle= (x(3));
end

function [f]=force(t, x) 
    xd=interp1(T, X, t);
    theta_des=atan2(xd(5),xd(4));
    v_des=sqrt(xd(4)*xd(4)+xd(5)*xd(5));
    xdd_d=sqrt(xd(7)*xd(7)+xd(8)*xd(8));
    omega = x(6);
    if v_des ~= 0
        omega_des = (xd(8)*xd(4)-xd(7)*xd(5))/(v_des*v_des);
    else
        omega_des = xd(6);
    end
    omegad_des=xd(9);
    v = sqrt(x(4)*x(4)+x(5)*x(5));
    c=cos(x(3));
    s=sin(x(3));
    R=[c s 0;
        -s c 0;
        0 0 1];
    E=xd(1:6)' - x;
    e = R*E(1:3);
    if e(3)>pi
        e(3) = e(3)-2*pi;
    elseif e(3)<-pi
        e(3) = e(3)+2*pi;
    end
%     k = [1 .1 .1];
%     u = [0; 0];
%     u(1)=-k(1)*e(1);
%     u(2)=-k(2)*e(2)-k(3)*e(3);   
%     f=[vd*cos(e(3))-u(1);
%         omegad-u(2)];
%     k = [.1 .1 .1];
%     f=[vd*cos(e(3))+k(1)*e(1)+k(2)*omegad*e(2);
%         omegad+k(3)*sin(e(3))];

    k = [0.1 0.1 .5];
    f=[.3*xdd_d*cos(e(3))+k(1)*e(1)+k(2)*omegad_des*e(2);
        omegad_des+k(3)*sin(e(3))];

end

function [f]=force_old(t, x) 
    xd=interp1(T, X, t);
    xdd_d=sqrt(xd(7)*xd(7)+xd(8)*xd(8));
    omega = x(6);
    omegad_des=xd(9);
    v = sqrt(x(4)*x(4)+x(5)*x(5));
    c=cos(x(3));
    s=sin(x(3));
    R=[c s 0;
        -s c 0;
        0 0 1];
    E=xd(1:6)' - x;
    e = R*E(1:3);
    if e(3)>pi
        e(3) = e(3)-2*pi;
    elseif e(3)<-pi
        e(3) = e(3)+2*pi;
    end
%     k = [1 .1 .1];
%     u = [0; 0];
%     u(1)=-k(1)*e(1);
%     u(2)=-k(2)*e(2)-k(3)*e(3);   
%     f=[vd*cos(e(3))-u(1);
%         omegad-u(2)];
%     k = [.1 .1 .1];
%     f=[vd*cos(e(3))+k(1)*e(1)+k(2)*omegad*e(2);
%         omegad+k(3)*sin(e(3))];

    k = [0.1 0.1 .5];
    f=[.3*xdd_d*cos(e(3))+k(1)*e(1)+k(2)*omegad_des*e(2);
        omegad_des+k(3)*sin(e(3))];

end

end

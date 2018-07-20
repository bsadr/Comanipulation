function y = planar_obj(mdlHuman, mdlRobot) 
% grasp points expressed in the object frame
% obj_len   = .50;    % length of the object
% obj_width = .20;    % width of the object
% obj_points = [[obj_len/2; 0] [-obj_len/2; 0]]; % human and robot grasp points
t0 = 0; 
tf   = 15;  %final time in seconds. 
dt = 0.005;
dt = 0.1;
time_lines=[t0,tf/2,tf];
time_span_sub = t0:dt:tf/2;
time_span = t0:dt:tf;

f_h_pre = [0;0;0];

switch_sequence=[];
% tmp=0;
% while tmp <2
%     tmp = tmp + rand(1)/2;
%     switch_sequence=[switch_sequence, tmp];
% end
switch_sequence = t0:dt:tf;
switch_mode = ones(size(switch_sequence));
switches = 1/dt*[2, 6];
for i = switches(1):switches(2)
    switch_mode(i)=0;
end
clear tmp
%%%%%%%%%%%%% Real Object
mo = 1; % mass, kg 
Co = [0; 0; 0];
%%%%%%%%%%%%% Virtual Object
kv = [.1; .1; .1]; % spring stiffness. N/m
Dv = [.7; .7; .7]; % damping coeff. N-s/m 
kv = [0; 0; 0]; % spring stiffness. N/m
Dv = [0; 0; 0]; % damping coeff. N-s/m 
mv = 1; % mass, kg 
Iv = 1; % moment of inertia
A = [0         0         0         1         0         0;
    0          0         0         0         1         0;
    0          0         0         0         0         1;   
    -kv(1)/mv  0         0         -Dv(1)/mv 0         0;
    0          -kv(2)/mv 0         0        -Dv(2)/mv  0;  
    0          0         -kv(3)/Iv 0         0        -Dv(3)/Iv];

B = [0   0     0;
    0    0     0
    0    0     0
    1/mv 0     0;
    0    1/mv  0;
    0    0     1/Iv];
% Mo.Mv^(-1)
MoMv1 = [mo/mv 0     0;
         0     mo/mv 0;
         0     0     mo/Iv];

x = dlmread('config.txt',',');
x0=x(1,1:6);
xf=x(2,1:6);

     
%%%%%%%%%%%%%%%%%%%%%%%%%% Trust parameters
% Trust time window
tau = 20;
prs = ones(1, tau);
%%%%%%%%%%%%%%%%%%%%%%%%%% Solve for x and h (y=[x(motion);h(generalized force)])
y = [];
% persistent variables used in rhs
t_pre = -1;
xhd=[];
fhd=[];
xrd=[];
frd=[];
[t,x]=ode45(@rhs,time_span,x0); 
[~,idu] = unique(y(1,:));
unique_y = y(:,idu);
tmp = t';
for i = 2:size(y,1)
    yq = interp1(unique_y(1,:), unique_y(i,:),t');
    tmp =[tmp; yq];
end
y = tmp';
clear tmp unique_y idu
%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Results
% figure
% plt_titles = {'x', '\dot{x}', 'y', '\dot{y}', '\theta', '\dot{\theta}',};
% for i = 2:7
%     subplot(3, 2, i-1)
%     plot(y(:,1), y(:,i))
%     title(plt_titles(i-1),  'Interpreter','latex')
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%% Aux. Functions
function xdot=rhs(t,x)
    if x(3)>pi
        x(3)=x(3)-pi;
    elseif x(3)<-pi
        x(3)=x(3)+pi;
    end    
    pattern = 1;
    dt_ode = t-t_pre;
    if dt_ode>(.5*dt)
        [xhd, fhd] = desired_human(x,f_h_pre, pattern);
        [xrd, frd] = desired_robot(x,f_h_pre, pattern);
    end
    t_pre = t;
    [f, f_imp, f_ext, f_e, f_r, f_h, f_int, mode] = force_imp(x, xhd, xrd, t);
    sig = .5;
    [pr, fi] = factors(f_e, f_int, f_h, f_r, xhd, x, sig);
%     if ~mode
%         fi(3) = 0;
%         fi(4) = fi(4)*.5;
%         pr = mean(fi);
%     end
    D = 1-fi(2);
    P_R = performance(pr);
%     y = [y, [t; x; f; f_e; f_r; f_h; f_int; P_R; D; mode; pr; fi; f_imp; f_ext; xhd]];
    y = [y, [t; x; f; f_e; f_r; f_h; f_int; P_R; D; mode; pr; fi; f_imp; f_ext; xhd; xrd]];
%     y = [y, [t; x; fhd; fhd; fhd; fhd; f_int; P_R; D; mode; pr; fi; f_imp; f_ext; xhd; xrd]];
%     disp([t, x' P_R f_h'])
    f_h_pre = f;
     xdot=A*x+B*f;
%    xdot=A*x+B*fhd;
end

function [f, f_imp, f_ext, f_e, f_r, fh, f_int, mode]=force_imp(x, xhd, xrd, t) 
    %%%%%%% estimate human trajectory and find robot force
    er = xrd - x;
    k1=[2  0  0 7];
    k2=[2  0  0 7];
    k3=[2 0  0  7];
    Kr=[k1 0  0;
        0  k2 0;
        0  0  k3];
    fr=.75*Kr*er;
    
    %%%%%%% simulate human desired trajectory and find human force
    eh=xhd-x;
    k4=[2 0  0  7];
    k5=[2 0  0  7];
    k6=[2 0  0  7];
    Kx=[k4 0  0;
        0  k5 0;
        0  0  k6];
    fh=.75*Kx*eh;
    
    seq=find(switch_sequence>=t);
    current_mode = switch_mode(seq(1));
    %persistent f_last;
    if current_mode
        f = fh+fr;
        f_imp = f;                      % impedance force h_{imp}
        f_ext = [0; 0; 0];              % external force h_{ext}
        f_e=force_obj(x, f_imp, f_ext); % desired effective force h_e^d
        f_r=force_robot(fh, f_e);       % robot force h_{S,r}
    else
        f = fh;
        f_imp = [0; 0; 0];
        f_ext = f;
        f_e=force_obj_reactive(x, f_imp, f_ext);
        f_r=f_e-fh;
    end
    f_int = f_r+fh-f_e;  % internal force
    mode = current_mode;
end

function f=force_obj(x, f_imp, f_ext)
    xdes = [x(4); x(5); x(6)];
    f = Co - f_ext + MoMv1 * (f_imp+f_ext-Dv.*xdes);
    if norm(f)>20
%         disp(f);
    end
end

function f=force_obj_reactive(x, f_imp, f_ext)
    xdes = [x(4); x(5); x(6)];
    f = Co - f_ext + MoMv1 * (f_imp+f_ext-Dv.*xdes);
    if norm(f)>20
%         disp(f);
    end
end


function f=force_robot(f_h, f_e)
    A_h = zeros(3,1);
    for k = 1:3
        if abs(f_e(k))>1e-3
            A_h(k) = f_h(k)/f_e(k);
        else
            A_h(k) = 1;
        end
    end
    %a_h = f_h./f_e;
    %z = f_e ~= 0;
    %A_h = isnan(a_h).*z;
    A_r = 1 - A_h;
    f = A_r.*f_e;
end

function [pr, fi] = factors(f_e, f_int, f_h, f_r, x_hat, x, sig)
    fe = norm(f_e);
    f1 = fe/(norm(f_int)+fe);
    if fe == 0 || isnan(f1)
        f1=1;
    end
    fhfr = f_h' * f_r;
    if fhfr > 0
        f2 = fhfr/(norm(f_h)*norm(f_r));
    elseif fhfr ==0
        f2 = 1;
    else
        f2 = 0;
    end
    xe = norm(x_hat-x);
    if xe<3*sig
        f3 = 1-xe/(3*sig);
    else
        f3 = 0;
    end
    f4 = min(norm(f_r)/fe, 1);
    fi = [f1; f2; f3; f4];
    pr = sum(fi)*.25;
end

function P_R = performance(pr)
    P = circshift(prs, 1);
    P(1) = pr;
    prs = P;
    P_R = mean(prs);
end


function [des_x, des_f] = desired_human(x,f, pattern)
    xh = [];
    for i=1:size(x,1)
%         xpred = predict(mdlHuman{i,1}, [x' f' pattern]);
        xpred = predict(mdlHuman{i,1}, [x' f']);
    	xh = [xh xpred];
    end
    des_x = xh';
    
    fh = [];
    for i=7:9
%         fpred = predict(mdlHuman{i,1}, [x' f' pattern]);
        fpred = predict(mdlHuman{i,1}, [x' f']);
    	fh = [fh fpred];
    end
    des_f = fh';
    
    
end

function [des_x, des_f] = desired_robot(x, f, pattern)
    xh = [];
    for i=1:size(x,1)
%         xpred = predict(mdlRobot{i,1}, [x' f' pattern]);
        xpred = predict(mdlRobot{i,1}, [x' f']);
    	xh = [xh xpred];
    end
    des_x = xh';

    fh = [];
    for i=7:9
%         fpred = predict(mdlHuman{i,1}, [x' f' pattern]);
        fpred = predict(mdlHuman{i,1}, [x' f']);
    	fh = [fh fpred];
    end
    des_f = fh';
end


end

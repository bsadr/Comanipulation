function [xn] = kalman3d(x)
%kalman3d Returns the filtered data
%   Detailed explanation goes here

A = x(:,2:end);
B = x(:,1);
dof = size(x,2)-1;
num_points = size(x,1);

%find delta time
a=1;
b=[1 -1];
T = filter(b,a,B); % sampling interval 

%system parameters
sigma_n=.1;  %measurement noise, variable
Q2R=50;    %dynamic noise to measurement noise ratio
sigma_d=sigma_n*Q2R;    %dynamic noise, variable
%matrices
Id = eye(dof);
I2d = eye(2*dof);
Zd = zeros(dof);
Phi=phi(T(1));  %transition matrix for predition of next state
Q=[Zd Zd;
   Zd sigma_d*Id]; %dynamic noise covariance
M=[Id Zd];    %observation matrix
R=sigma_n*Id;   %measurement noise covariance

%initial values
Xtm1tm1=[A(1,:)';zeros(dof,1)];  %X_{t-1,t-1}  
Stm1tm1=Q;%  %S_{t-1,t-1}
Xnew=zeros(num_points,dof*2);
%iteration
for i=1:num_points
Phi=phi(T(i));    
Xttm1=Phi*Xtm1tm1;  %I: predict next state
Sttm1=Phi*Stm1tm1*Phi'+Q;   %II: predict next state covariance
%if norm(A(i,:))>0   %check to see if the data is valid
Yt=A(i,:)';  %III: obtain measurement
%else
%    A(i,:)=A(i-1,:);
%end
Kt=Sttm1*M'*inv(M*Sttm1*M'+R);  %IV: calculate weight
Xtt=Xttm1+Kt*(Yt-M*Xttm1);  %V: update state
Stt=(I2d-Kt*M)*Sttm1;  %VI: update state covariance
Xnew(i,:)=Xtt';    %store the filtered data
Xtm1tm1=Xtt;  %VII: loop
Stm1tm1=Stt;  %VII: loop
end
xn = [B Xnew];

function Phi = phi(t)
    %transition matrix for predition of next state
    Phi=[Id t*Id;
        Zd Id]; 
end
end


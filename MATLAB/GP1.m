%%
% raw data
% x = dlmread('chest_striaght.txt',',',1, 0);
x = dlmread('chest_obstacle.txt',',',1, 0);
dt = 0.003;
% remove unnecesessary data data 
a=1;
b=[1 -1];
y = filter(b,a,x);
I = y(:,1)>=1e-3;
x = x(I,:);
% smoothed data
% a=1;
% b=[1/4 1/4 1/4 1/4];
% y = filter(b,a,x);

%%
% visualization
close all
% figure(1)
% for i=2:5
%     subplot(3,1,i-1)
%     plot(x(:,1), x(:,i),'r-')
%     hold on 
%     plot(x(:,1), x(:,i+3),'b--')
%     ylabel(num2str(i-1));
%     grid on
% end
% xlabel('X');
% ylabel('Y');
% box on
% grid on

figure(2)
subplot(2,1,1)
plot(x(4000:7000,2), x(4000:7000,3),'r-')
hold on
plot(x(4000:7000,5), x(4000:7000,6),'b--')
    xlim([-2500 2500]);
    xlabel('X');
    ylabel('Y');

theta = atan2d(x(:,3)-x(:,6), x(:,2)-x(:,5));
subplot(2,1,2)
plot(x(4000:7000,1), theta(4000:7000))
%%
% calculate object position and angle
theta = atan2(x(:,3)-x(:,6), x(:,2)-x(:,5));
x=[x(:,1) .5*(x(:,2)+x(:,5)) .5*(x(:,3)+x(:,6)) theta];


% object position and angle and their velocity from kalman
% x = kalman3d(x);
% v = kalman3d(v);
% x = [x(:,1:7) v(:,5:7)];
% plot(x(4000:7000,1), theta(4000:7000))
plot(x(:,1), theta)

%%
close all
figure(1)
for i=2:5
    subplot(3,1,i-1)
    plot(v(:,1), v(:,i), 'r-.')
    ylabel(num2str(i-1));
    grid on
    hold on
    plot(v(:,1), v(:,i+3), 'b--')
end

%%
% visualization
close all
figure(1)
for i=2:7
    subplot(2,3,i-1)
    plot(x(:,1), x(:,i))
    ylabel(num2str(i-1));
    grid on
end
xlabel('X');
ylabel('Y');
box on
grid on

figure(2)
subplot(2,1,1)
plot(x(:,1), x(1:end,3))
subplot(2,1,2)
plot(x(:,1), x(1:end,6))

%%
% patterns start and end timestamps, and pattern type number
tp = [
    [9 24 1]
    [25 32.6 1]
    [37 47 2]
    [52.5,59.5 1]
    [64.5 71.9 2]
    [75.3 83.2 1]
    [86.5 94 2]];
%  patterns data
num_patterns = size(tp,1);
patterns = cell(num_patterns, 2);
% for i = 1:num_patterns
for i = 1:1
    I = (x(:,1) > tp(i,1)) & (x(:,1) < tp(i,2));
    tmp = x(I,:);
    tmp(:,1) = tmp(:,1) - tmp(1,1); 
    
    tmp = kalman3d(tmp);
    v_tmp = [tmp(:,1) tmp(:,5:7)];
    v_tmp = kalman3d(v_tmp);
    tmp = [tmp(:,1:4) v_tmp(:,2:7)];
    
    xf = simulate_force(tmp, dt);
    patterns{i,1} = xf;
    patterns{i,2} = tp(i,3);    
end

%%
% visualization
figure(2)
%for i=1:num_patterns
for i=1:1
    y = patterns{i,1};
    for i=2:7
        subplot(3,3,i-1)
        plot(y(:,1), y(:,i))
        ylabel(num2str(i-1));
        grid on
        hold on
        plot(y(:,1), y(:,i+8), 'b--')
    end

    for i=8:10
        subplot(3,3,i-1)
        plot(tmp(:,1), tmp(:,i))
        ylabel(num2str(i-1));
        grid on
        hold on
    end
    
    figure(3)
    plot(y(:,2), y(:,3))
    grid on
    hold on
    plot(y(:,10), y(:,11), 'b--')
    xlim([-2500 2500]);
    xlabel('X');
    ylabel('Y');
end
%%
% visualization
figure(3)
% for i=1:num_patterns
for i=1:1
    y = patterns{i,1};
    subplot(1,2,1)
    plot(y(:,1), y(:,8))
    ylabel('Force');
    grid on
    subplot(1,2,2)
    plot(y(:,1), y(:,9))
    ylabel('Moment');
    grid on
end

%%
figure(4)
plot3(x(:,2), x(:,3), x(:,4))
xlabel('X');
ylabel('Y');
zlabel('Z');
box on
grid on

% load gpr models from file
load('gprMdls_obstacle.mat')
%load('gprMdls_straight.mat')
mdlHuman = gprMdls;
load('gprMdls_obstacle.mat')
mdlRobot = gprMdls;
%clear('gprMdls');

visualization = false;

y = planar_obj(mdlHuman, mdlRobot);
time_span = y(:,1);
P_R = y(:,23);
D = y(:,24);
% close all
% figure
% plt_titles = {'$x$', '$\dot{x}$', '$y$', '$\dot{y}$', '$\theta$', '$\dot{\theta}$'};
% for i = 2:7
%     subplot(3, 2, i-1)
%     plot(y(:,1), y(:,i))
%     title(plt_titles(i-1), 'Interpreter', 'latex')
% end
%%

if visualization
figure(1)
hold on
% x
plot(y(:,2), y(:,3))
%ylim([-.5 .5])
hold on
% xh_d
plot(y(:,37), y(:,38))
% xr_d
plot(y(:,43), y(:,44))
title('x-y ', 'Interpreter', 'latex')
%xlim([-1.500 2.500]);
xlabel('X');
ylabel('Y');
legend('x', 'xh_d', 'xr_d')
%% x and xd
figure(2)
plt_titles = {'${x}$', '${y}$', '${\theta}$'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+1))
    hold on
    %xh_d
    plot(y(:,1), y(:,i+36), 'b--')
    title(plt_titles(i), 'Interpreter', 'latex')
end
%% xdot and xdot_h
figure(3)
plt_titles = {'$\dot{x}$', '$\dot{y}$', '$\dot{\theta}$'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+4))
    hold on
    %xh_d
    plot(y(:,1), y(:,i+39), 'b--')
    title(plt_titles(i), 'Interpreter', 'latex')
end
%% f_e
figure(4)
plt_titles = {'$f_{e,x}$', '$f_{e,y}$', '$f_{e,\theta}$'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+10))
    title(plt_titles(i), 'Interpreter', 'latex')
end
%% f_r
figure(5)
plt_titles = {'$f_{r,x}$', '$f_{r,y}$', '$f_{r,\theta}$'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+13))
    title(plt_titles(i), 'Interpreter', 'latex')
end
%% f_h
figure(6)
plt_titles = {'$f_{h,x}$', '$f_{h,y}$', '$f_{h,\theta}$'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+16))
    title(plt_titles(i), 'Interpreter', 'latex')
end

%% f_int
figure(7)
plt_titles = {'$f_{int,x}$', '$f_{int,y}$', '$f_{int,\theta}$'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+19))
    title(plt_titles(i), 'Interpreter', 'latex')
 end
%% P_R, D, mode
figure(8)
plt_titles = {'$P_R$', '$D$', 'mode'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+22))
    title(plt_titles(i), 'Interpreter', 'latex')
end
%% p_r
figure(9)
plt_titles = {'$p_r$', '$p_1$', '$p_2$', '$p_3$', '$p_4$'};
for i = 1:5
    subplot(5, 1, i)
    plot(y(:,1), y(:,i+25))
    title(plt_titles(i), 'Interpreter', 'latex')
    ylim([-.09 1.09])
end
%% f_imp
figure(10)
plt_titles = {'$f_{imp,x}$', '$f_{imp,y}$', '$f_{imp,\theta}$'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+30))
    title(plt_titles(i), 'Interpreter', 'latex')
 end
%% f_ext (0,0,0) 
% figure(11)
% plt_titles = {'$f_{ext,x}$', '$f_{ext,y}$', '$f_{ext,\theta}$'};
% for i = 34:36
%     subplot(3, 1, i-33)
%     plot(y(:,1), y(:,i))
%     title(plt_titles(i-33), 'Interpreter', 'latex')
% end
%% xh_d
 figure(12) 
 plt_titles = {'$\hat{x}$','$\hat{y}$','$\hat{\theta}$'};
 for i = 1:3
     subplot(3, 1, i)
     plot(y(:,1), y(:,i+36))
     hold on
     %plot(y(:,1), y(:,i+1))
     title(plt_titles(i), 'Interpreter', 'latex')
 end
end
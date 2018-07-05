% load gpr models from file
% load('gprMdls_obstacle.mat')
% load('gprMdls_straight.mat')
mdlHuman = gprMdls;
% load('gprMdls_obstacle.mat')
mdlRobot = gprMdls;
%clear('gprMdls');

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

figure(1)
hold on
plot(y(:,2), y(:,3))
%ylim([-.5 .5])
hold on
plot(y(:,37), y(:,38))
plot(y(:,43), y(:,44))
title('x-y ', 'Interpreter', 'latex')
%xlim([-1.500 2.500]);
xlabel('X');
ylabel('Y');
pause(.5)
%% x and xd
figure(8)
plt_titles = {'${x}$', '${y}$', '${\theta}$'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+1))
    hold on
    plot(y(:,1), y(:,i+36), 'b--')
    title(plt_titles(i), 'Interpreter', 'latex')
end
%% xdot and xdot_h
figure(9)
plt_titles = {'$\dot{x}$', '$\dot{y}$', '$\dot{\theta}$'};
for i = 1:3
    subplot(3, 1, i)
    plot(y(:,1), y(:,i+4))
    hold on
    plot(y(:,1), y(:,i+39), 'b--')
    title(plt_titles(i), 'Interpreter', 'latex')
end
%%
figure(2)
plt_titles = {'$f_{e,x}$', '$f_{e,y}$', '$f_{e,\theta}$'};
for i = 11:13
    subplot(3, 1, i-10)
    plot(y(:,1), y(:,i))
    title(plt_titles(i-10), 'Interpreter', 'latex')
end

figure(3)
plt_titles = {'$f_{r,x}$', '$f_{r,y}$', '$f_{r,\theta}$'};
for i = 14:16
    subplot(3, 1, i-13)
    plot(y(:,1), y(:,i))
    title(plt_titles(i-13), 'Interpreter', 'latex')
end

figure(4)
plt_titles = {'$f_{h,x}$', '$f_{h,y}$', '$f_{h,\theta}$'};
for i = 17:19
    subplot(3, 1, i-16)
    plot(y(:,1), y(:,i))
    title(plt_titles(i-16), 'Interpreter', 'latex')
end

% figure
% plt_titles = {'$f_{int,x}$', '$f_{int,y}$', '$f_{int,\theta}$'};
% for i = 20:22
%     subplot(3, 1, i-19)
%     plot(y(:,1), y(:,i))
%     title(plt_titles(i-19), 'Interpreter', 'latex')
% end

figure(5)
plt_titles = {'$P_R$', '$D$', 'mode'};
for i = 23:25
    subplot(3, 1, i-22)
    plot(y(:,1), y(:,i))
    title(plt_titles(i-22), 'Interpreter', 'latex')
end

figure(6)
plt_titles = {'$p_r$', '$p_1$', '$p_2$', '$p_3$', '$p_4$'};
for i = 26:30
    subplot(5, 1, i-25)
    plot(y(:,1), y(:,i))
    title(plt_titles(i-25), 'Interpreter', 'latex')
end

% figure
% plt_titles = {'$f_{imp,x}$', '$f_{imp,y}$', '$f_{imp,\theta}$'};
% for i = 31:33
%     subplot(3, 1, i-30)
%     plot(y(:,1), y(:,i))
%     title(plt_titles(i-30), 'Interpreter', 'latex')
% end
% 
% figure
% plt_titles = {'$f_{ext,x}$', '$f_{ext,y}$', '$f_{ext,\theta}$'};
% for i = 34:36
%     subplot(3, 1, i-33)
%     plot(y(:,1), y(:,i))
%     title(plt_titles(i-33), 'Interpreter', 'latex')
% end

 figure(7)
 plt_titles = {'$\hat{x}$', '$\hat{\dot{x}}$', '$\hat{y}$',
     '$\hat{x}$', '$\hat{y}$', '$\hat{\theta}$'};
 for i = 37:39
     subplot(3, 1, i-36)
     plot(y(:,1), y(:,i))
     hold on
     plot(y(:,1), y(:,i-36))
     title(plt_titles(i-36), 'Interpreter', 'latex')
 end
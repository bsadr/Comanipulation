% load gpr models from file
%load('gprMdls_obstacle.mat')
load('gprMdls_straight.mat')
mdlHuman = gprMdls;
load('gprMdls_obstacle.mat')
mdlRobot = gprMdls;
%clear('gprMdls');

visualization = true;

y = planar_obj_trust(mdlHuman, mdlRobot);
time_span = y(:,1);
P_R = y(:,23);
D = y(:,24);

% save('result_thesis.mat', 'y', 'time_span', 'P_R', 'D');
% close all
% figure
% plt_titles = {'$x$', '$\dot{x}$', '$y$', '$\dot{y}$', '$\theta$', '$\dot{\theta}$'};
% for i = 2:7
%     subplot(3, 2, i-1)
%     plot(y(:,1), y(:,i))
%     title(plt_titles(i-1), 'Interpreter', 'latex')
% end
%%
mode_pro = y(:,25)>.5;
mode_re = y(:,25)<.5;
y(:,25)=mode_pro.*ones(size(mode_pro));
Mode = y(:,25);
y(:,43:45)=repmat(mode_re,1,3).*y(:,2:4)+repmat(mode_pro,1,3).*y(:,43:45);
if visualization
figure(1)
axis equal
% x
plot(y(:,2), y(:,3), 'r-')
%ylim([-.5 .5])
hold on
% xh_d
plot(y(:,37), y(:,38), 'b:')
% xr_d
plot(y(:,43), y(:,44), 'm--')
% title('x-y ', 'Interpreter', 'latex')
ylim([-1 1])
xlim([-1 5]);
xlabel('$x$', 'interpreter', 'latex');
ylabel('$y$', 'interpreter', 'latex');
legend({'xo', 'xh', 'xr'}, 'location', 'northwest')
 
%figure3.PaperUnits = 'inches';
% figure3.PaperSize = [6 2];
%figure3.PaperPosition = [0 0 6 2];
print('result_switching_motion','-depsc','-r0')
print('result_switching_motion','-dpdf','-r0')
%% x and xd
figure2=figure(2);
plt_titles = {'${x}$', '${y}$', '${\theta}$'};
s = ['x','y','t'];

for i = 1:3
    subplot(3, 3, i)
    plot(y(:,1), y(:,i+1), 'r-')
    hold on
    %xh_d
    plot(y(:,1), y(:,i+36), 'b:')
    plot(y(:,1), y(:,i+42), 'm--')
    ylabel(plt_titles(i), 'Interpreter', 'latex')
    xlim([0 8])
%     legend(legends)
%     legend({strcat(s(i),'o'), strcat(s(i),'h'), strcat(s(i),'h')},'location', 'northwest')
end
% xdot and xdot_h
% figure(3)
plt_titles = {'$\dot{x}$', '$\dot{y}$', '$\dot{\theta}$'};
for i = 1:3
    subplot(3, 3, 3+i)
    plot(y(:,1), y(:,i+4), 'r-')
    hold on
    %xh_d
    plot(y(:,1), y(:,i+39), 'b:')
    plot(y(:,1), y(:,i+45), 'm--')
    ylabel(plt_titles(i), 'Interpreter', 'latex')
    xlim([0 8])
%     legend('object', 'human', 'robot')

end
% f_e, f_h & f_r
% figure(5)
plt_titles = {'$f_{x}$', '$f_{y}$', '$f_{\theta}$'};
for i = 1:3
    subplot(3, 3, 6+i)
    plot(y(:,1), y(:,i+10), 'r-')
    hold on
    plot(y(:,1), y(:,i+13), 'b:')
    plot(y(:,1), y(:,i+16), 'm--')
    ylabel(plt_titles(i), 'Interpreter', 'latex')
    xlim([0 8])
%     legend('object', 'human', 'robot')
end
figure2.PaperUnits = 'inches';
figure2.PaperPosition = [0 0 9 6];
print('result_switching_trajectories','-depsc','-r0')
print('result_switching_trajectories','-dpdf','-r0')

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

 %% trust
  figure(13) 
 plt_titles = {'trust'};
 subplot(4, 1, 3)
 hold on
 plot(y(:,1), y(:,49),'b.')
 hold on
 %plot(y(:,1), y(:,i+1))
 title(plt_titles, 'Interpreter', 'latex')
 
end
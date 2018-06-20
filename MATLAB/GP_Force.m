%% Reading data
% x = dlmread('chest_striaght.txt',',',1, 0);
x = dlmread('chest_obstacle.txt',',',1, 0);
dt = 0.01;
% remove unnecesessary data data 
a=1;
b=[1 -1];
y = filter(b,a,x);
I = y(:,1)>=1e-3;
x = x(I,:);
%dlmwrite('chest_obstacle_filtered.txt',x);
% smooth data
% a=1;
% b=[1/4 1/4 1/4 1/4];
% y = filter(b,a,x);
visualization = true;
%% Visualising raw data
if visualization
    close all
    figure(1)
    theta = atan2d(x(:,3)-x(:,6), x(:,2)-x(:,5));
    ytitles = ["x", "y", "z", "\theta"];
    for i=2:4
        subplot(4,2,2*i-3)
        plot(x(:,1), x(:,i),'r-');      hold on 
        plot(x(:,1), x(:,i+3),'b--');   grid on
        ylabel(ytitles(i-1));
    end
    subplot(4,2,7)
    plot(x(:,1), theta,'r-');    grid on
    ylabel(ytitles(4));
    subplot(1,2,2)
    plot(x(:,2), x(:,3),'r-');   hold on
    plot(x(:,5), x(:,6),'b--')
    xlim([-1500 2500]);
    xlabel('X');
    ylabel('Y');
end
%% calculateing object position and angle 
theta = atan2(x(:,3)-x(:,6), x(:,2)-x(:,5));
x=[x(:,1) .5*(x(:,2)+x(:,5)) .5*(x(:,3)+x(:,6)) theta];
%% Splitting patterns
% patterns start and end timestamps, and pattern type number
% tp=dlmread('chest_straight_patterns.txt');
tp=dlmread('chest_obstacle_patterns.txt');
%  patterns data
num_patterns = size(tp,1);
patterns = cell(num_patterns, 3);
% for i = 1:num_patterns
for i = 1:num_patterns
    % split the data
    I = (x(:,1) > tp(i,1)) & (x(:,1) < tp(i,2));
    tmp = x(I,:);
    tmp(:,1) = tmp(:,1) - tmp(1,1); 
    
    % apply kalman filter
    tmp = kalman3d(tmp);
    v_tmp = [tmp(:,1) tmp(:,5:7)];
    v_tmp = kalman3d(v_tmp);
    tmp = [tmp(:,1:4) v_tmp(:,2:7)]; % tmp = [time, x, xdot, xddot]
    
    % reduce the data size
    timespan = linspace(0,tmp(end,1),100);
    xq = interp1(tmp(:,1), tmp(:,2:end), timespan);
    xq(end,4:end)=0.*xq(end,4:end);
    % simulate force    
    xf = simulate_force_omni_directions(tmp, dt);   
    % xf = t(1); x(2:4); dx(5:7); f(8:10); 
    %      xref(11:13); dxref(14:16); ddxref(17:19)
%     xf = tmp(:,2:7); % xf = [x and xdot]
%     xf = xq(:,1:6); % xf = [x and xdot]
    patterns{i,1} = xf(:,2:10);
    patterns{i,2} = tp(i,3); 
    patterns{i,3} = xf;     
end

%% Visualizating force simulation

if visualization
    % y = t(1); x(2:4); dx(5:7); f(8:10); 
    % xref(11:13); dxref(14:16); ddxref(17:19)
    ytitles = ["$x$", "$y$", "$\theta$"];
    ytitles = [ytitles, "$\dot{x}$", "$\dot{y}$", "$\dot{\theta}$"];
    ytitles = [ytitles, "$f_x$", "$f_y$", "$f_\theta$"];
%     for i=1:num_patterns
    for i=1:num_patterns
        % prompt before showing the plots
        uiwait(msgbox(num2str(i),'Plotting','modal'));
        y = patterns{i,3};
        close all
        figure(2)
        % plot x and dx
        for i=2:7
            % simulation
            subplot(3,3,i-1)
            plot(y(:,1), y(:,i))
            ylabel(ytitles(i-1), 'interpreter', 'latex');
            grid on
            hold on
            % desired
            plot(y(:,1), y(:,i+9), 'b--')
        end
        % plot f
        for i=8:10
            subplot(3,3,i-1)
            plot(y(:,1), y(:,i))
            ylabel(ytitles(i-1), 'interpreter', 'latex');
            grid on
            hold on
        end

        % plot x-y trajectories
        figure(3)
        plot(y(:,2), y(:,3))
        grid on
        hold on
        plot(y(:,11), y(:,12), 'b--')
        xlim([-2500 2500]);
        xlabel('X');
        ylabel('Y');
        legend({'simulation', 'reference'}, 'Location','southoutside')
    end
end

%% find x0 and xf
x0=[];
xf=[];
for i = 1:num_patterns
    if patterns{i,2} == 1
        x0 = [x0; patterns{i,1}(1,:)];
    else
        xf = [xf; patterns{i,1}(1,:)];
    end
end
x0 = mean(x0);
xf = mean(xf);
dlmwrite('config.txt',[x0; xf]);

msgbox('Training the GP models. It would take a while','Training','modal');
%% fit gaussian proccess model
predictor_data = [];
response_data = [];
for i = 1:num_patterns
% for i = 1:1
    % predictors, xi = [x y theta xdot ydot thetadot pattern_no]
    % predictors, xi = [x y theta xdot ydot thetadot f_x f_y f_z pattern_no]
    pattern_no = patterns{i,2};
    xi = patterns{i,1};
%     xi = [xi(:,1:3)  ones(size(xi,1),1)*pattern_no];
    xi = [xi  ones(size(xi,1),1)*pattern_no];
    predictor_data = [predictor_data; xi];
    
    % responses = [x y theta xdot ydot thetadot pattern_no] at the next time step
    % responses = [x y theta xdot ydot thetadot f_x f_y f_z pattern_no]
    % find next values
    tmp = [xi(2:end,:); xi(end,:)];
%     tmp = [xi(6:end,:); xi(end-4:end,:)];
    response_data = [response_data; tmp];
end
num_Mdls = size(response_data,2);
gprMdls = cell(num_Mdls,1);
for i=1:num_Mdls
    gprMdls{i,1} = fitrgp(predictor_data,response_data(:,i)); 
end
% save('gprMdls_obstacle', 'gprMdls');
save('gprMdls_straight', 'gprMdls');

%% 
% Predict the response corresponding to the rows of |xi| (resubstitution
% predictions) using the trained model. 
test_data = [];
for i=1:2
    tmp = resubPredict(gprMdls{i,1});
    test_data = [test_data tmp];
end
%%
n = 100;
new_s = linspace(-pi, pi, n)';
new_x = ones(n,1)*700.*(1+sin(new_s));
new_y = linspace(-2100,2000,n)';
new_theta = ones(n,1)*pi/2;
new_pattern = ones(n,1)*1;
new_data = [new_x new_y new_theta zeros(n,3) new_pattern];

new_response = [];
for i=1:2
    ypred = predict(gprMdls{i,1}, new_data);
    new_response = [new_response ypred];
end


%%
if visualization
    figure(2)
    plot(predictor_data(:,1), predictor_data(:,2), 'k--')
    hold on
    plot(test_data(:,1), test_data(:,2), 'r')
    plot(new_response(:,1), new_response(:,2), 'b');
    plot(new_data(:,1), new_data(:,2), 'r');
    xlim([-2500 2500])
    num_points = size(new_data,1);
    for i=1:num_points
        xx=[new_data(i,1) new_response(i,1)];
        yy=[new_data(i,2) new_response(i,2)];
        plot(xx, yy)
    end
end
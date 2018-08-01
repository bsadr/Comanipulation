%% Reading data
clear all
visualization = true;
visualization = false;
exp = ExpConditions.Straight;
% exp = ExpConditions.Obstacle;

if exp == ExpConditions.Straight
    % raw data from motion capture  
    file_src = 'chest_straight_filtered.txt';
    % start and end of each motion
    file_pattern = 'chest_straight_patterns.txt';
    % output data (GP model)
    file_gprMdls = 'gprMdls_straight';
end

if exp == ExpConditions.Obstacle
    % raw data from motion capture  
    file_src = 'chest_obstacle_filtered.txt';
    % start and end of each motion
    file_pattern = 'chest_obstacle_patterns.txt';
    % output data (GP model)
    file_gprMdls = 'gprMdls_obstacle';  
end
x = dlmread(file_src,',',1, 0);
dt = 0.2;
% remove unnecesessary data data 
a=1;
b=[1 -1];
y = filter(b,a,x);
I = y(:,1)>=1e-3;
x = x(I,:);
if exp == ExpConditions.Obstacle
    %dlmwrite('chest_obstacle_filtered.txt',x);
end
if exp == ExpConditions.Straight
    %dlmwrite('chest_straight_filtered.txt',x);
end
% smooth data
% a=1;
% b=[1/4 1/4 1/4 1/4];
% y = filter(b,a,x);
if exp == ExpConditions.Straight
    x(:,2) = mean(x(1500:24000,2));
    x(:,5) = mean(x(1500:24000,5));
    x(:,8) = mean(x(1500:24000,8));
    x(:,11) = mean(x(1500:24000,11));
end
x(:,2:end) = x(:,2:end)*0.001;
% calculateing object position and angle 
%theta = atan2(x(:,3)-x(:,6), x(:,2)-x(:,5));
%x=[x(:,1) .5*(x(:,2)+x(:,5)) .5*(x(:,3)+x(:,6)) theta];
theta = atan2(x(:,2)-x(:,5), x(:,3)-x(:,6));
x=[x(:,1) .5*(x(:,3)+x(:,6)) .5*(x(:,2)+x(:,5)) theta];

%% Visualising raw data
if visualization
    close all
    figure(1)
    ytitles = ['x', 'y', '\theta'];
    for i=2:4
        subplot(3,2,2*i-3)
        plot(x(:,1), x(:,i));      hold on 
        ylabel(ytitles(i-1));
    end
    subplot(1,2,2)
    plot(x(:,2), x(:,3),'r-');   hold on
%     xlim([-1.500 2.500]);
    ylim([-1.500 2.500]);
    xlabel('X');
    ylabel('Y');
end
%% Splitting patterns
% patterns start and end timestamps, and pattern type number
tp=dlmread(file_pattern);
%  patterns data
num_patterns = size(tp,1);
patterns = cell(num_patterns, 3);
for i = 1:num_patterns
% for i = 1:1
    % split the data
    I = (x(:,1) > tp(i,1)) & (x(:,1) < tp(i,2));
    tmp = x(I,:);
    % transfer data points to the origin
    tmp(:,1:3) = tmp(:,1:3) - tmp(1,1:3); 
    pattern_no = tp(i,3);    

    % transfer outbounds data points in x- directions to the origin
    if pattern_no == 1 || pattern_no == 3
%             J = (tmp(:,2) < tmp(1,2));
%             tmp(J,2) = tmp(2,2);
            J = (tmp(:,3) < tmp(1,3));
            tmp(J,3) = tmp(2,3);
    elseif pattern_no ==  2 || pattern_no == 4
%             J = (tmp(:,2) > tmp(1,2));
            J = (tmp(:,3) > tmp(1,3));
%            tmp(J,2) = tmp(2,2);
    end
    % add goal points to the end of data
    for j = 1:1
        tmp = [tmp; tmp(end,:)];

        tmp(end,1)=tmp(end,1)+dt;
        tmp(end,3)=tmp(1,3);
%         tmp(end,1:2) = [tmp(end,1)+dt tmp(1,2)];
    end
            
    % apply kalman filte
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
    xf(:,7:9)=xf(:,7:9)*.5;
    % xf = t(1); x(2:4); dx(5:7); f(8:10); 
    %      xref(11:13); dxref(14:16); ddxref(17:19)
%     xf = tmp(:,2:7); % xf = [x and xdot]
%     xf = xq(:,1:6); % xf = [x and xdot]
    patterns{i,1} = xf(:,2:10);
    patterns{i,2} = pattern_no;
    patterns{i,3} = xf;     
end

%% Visualizating force simulation
if visualization
    % add obstacle and human-human to the figure
    figure(3)
    ha =gca;
    haPos = get(ha,'position');
    [img, map, alphachannel] = imread('coffee table.png');
    s =size(img)*.001*.2;
    img_size = [s(2) s(1)];
    img_corner = [.65,.57].*[haPos(3)-haPos(1), haPos(4)-haPos(2)];
    img_pos = [haPos(1:2)+img_corner-img_size*.5, img_size];
    ha2=axes('position',img_pos);
    image(img)
%     image(img, 'AlphaData', alphachannel)
    colormap (map)
    set(ha2,'handlevisibility','off','visible','off')  

    % y = t(1); x(2:4); dx(5:7); f(8:10); 
    % xref(11:13); dxref(14:16); ddxref(17:19)
    ytitles = ["$x$", "$y$", "$\theta$"];
    ytitles = [ytitles, "$\dot{x}$", "$\dot{y}$", "$\dot{\theta}$"];
    ytitles = [ytitles, "$f_x$", "$f_y$", "$f_\theta$"];
    %for i=3:3
    %%
    idx=[1,3,5,7];
    for j=1:1%num_patterns
        i=idx(j);
%     for i=1:num_patterns      
        % prompt before showing the plots
%         uiwait(msgbox(num2str(i),'Plotting','modal'));
        y = patterns{i,3};
%         close all
        figure2=figure(2);
        % plot x and dx
        for i=2:7
            % simulation
            subplot(3,3,i-1)
            plot(y(:,1), y(:,i), 'b-')
            ylabel(ytitles(i-1), 'interpreter', 'latex');
            grid on
            hold on
            % desired
            plot(y(:,1), y(:,i+9), 'r-.')
            xlim([0 15])
        end
        % plot f
        for i=8:10
            subplot(3,3,i-1)
            plot(y(:,1), y(:,i), 'b-')
            ylabel(ytitles(i-1), 'interpreter', 'latex');
            grid on
            hold on
            xlim([0 15])
        end
        figure2.PaperUnits = 'inches';
        figure2.PaperPosition = [0 0 9 6];
%         print('training_controller','-depsc','-r0')
%         print('training_controller','-dpdf','-r0')
      
        %%
        % plot x-y trajectories
        figure3=figure(3);
    % display \theta on the figure
    [meshX,meshY] = meshgrid(y(:,2),y(:,3));
    [~,mesht1] = meshgrid(y(:,2),y(:,4));
    [~,mesht2] = meshgrid(y(:,3),y(:,4));
    u = cos(mesht1);
    v = sin(mesht1);
    mask = zeros(size(meshX))==ones(size(meshX));
    l=[1,25:5:(size(mask,2)-5)];
    for k=1:size(l,2)%num_patterns
        mask(l(k),l(k))=true;
    end
    quiver(meshX(mask),meshY(mask),u(mask),v(mask), 'AutoScaleFactor',.12, ...
        'Marker', '.', 'MarkerSize', 6)
    hold on
    
 %%%theta original
     % display \theta on the figure
    [meshX,meshY] = meshgrid(y(:,11),y(:,12));
    [~,mesht1] = meshgrid(y(:,11),y(:,13));
    [~,mesht2] = meshgrid(y(:,12),y(:,13));
    u = cos(mesht1);
    v = sin(mesht1);
    mask = zeros(size(meshX))==ones(size(meshX));
    l=[1,25:5:(size(mask,2)-5)];
    for k=1:size(l,2)%num_patterns
        mask(l(k),l(k))=true;
    end
    quiver(meshX(mask),meshY(mask),u(mask),v(mask), 'AutoScaleFactor',.12, ...
        'Marker', '.', 'MarkerSize', 6)
    hold on
%%%%    
    
        ax = gca;
        ax.ColorOrderIndex = j;
        axis equal
%         axis tight
        plot(y(:,2), y(:,3), 'b-')                        
        grid off
        box on
        hold on
         plot(y(:,11), y(:,12), 'r-.')
%         xlim([-2.500 2.500]);
        ylim([-1 1]);
        xlim([-1 5]);
%         xlim([-3 3]);

        xlabel('$x$', 'interpreter', 'latex');
        ylabel('$y$', 'interpreter', 'latex');
        %legend({'simulation', 'reference'}, 'Location','southeast')
    end
end
%figure3.PaperUnits = 'inches';
% figure3.PaperSize = [6 2];
%figure3.PaperPosition = [0 0 6 2];
% print('training_motion','-depsc','-r0')
% print('training_motion','-dpdf','-r0')

% print('sample_motion','-depsc','-r0')
% print('sample_motion','-dpdf','-r0')

%% find x0 and xf
x0=[];
xf=[];
for i = 1:num_patterns
    if patterns{i,2} == 1
        x0 = [x0; patterns{i,1}(1,:)];
        xf = [xf; patterns{i,1}(end,:)];
    end
end
if size(x0,1) > 1
    x0 = mean(x0);
end
if size(xf,1) > 1
    xf = mean(xf);
end
    xf(4:6)=zeros(1,3);
if exp == ExpConditions.Obstacle
    dlmwrite('config.txt',[x0(1:6); xf(1:6)]);
end       
%% fit gaussian proccess model
f = waitbar(0,'Training the GP models. It would take a while','Name','Training GP Models',...
    'CreateCancelBtn','setappdata(gcbf,''canceling'',1)');
% setappdata(f,'canceling',0);

predictor_data = [];
response_data = [];
for i = 1:num_patterns
% for i = 1:1
    % predictors, xi = [x y theta xdot ydot thetadot f_x f_y f_z pattern_no]
    pattern_no = patterns{i,2};
    if pattern_no == 1
        xi = patterns{i,1};
        % xi = [xi(:,1:3)  ones(size(xi,1),1)*pattern_no];
        % xi = [xi  ones(size(xi,1),1)*pattern_no];
        predictor_data = [predictor_data; xi];

        % responses = [x y theta xdot ydot thetadot pattern_no] at the next time step
        % responses = [x y theta xdot ydot thetadot f_x f_y f_z pattern_no]
        % force is not a output
        % find next values
%       tmp = [xi(2:end,:); xi(end,:)];
%       tmp = [xi(3:end,:); xi(end-1:end,:)];
        tmp = [xi(4:end,:); xi(end-2:end,:)];
%       tmp = [xi(5:end,:); xi(end-3:end,:)];
        response_data = [response_data; tmp];

        % generate points near goal
        goal = predictor_data(end,:);
        goal(4:9)=zeros(1,6);
        r = -.05:.10:.05;
        [Rx, Ry, Rt] = meshgrid(r,r,r);
        for l = 0:0
            for m = 1:length(r)
                for n = 1:length(r)
                    for o = 1:length(r)
                        neighbour = goal;
                        neighbour(1+l*3) = Rx(m,n,o) + goal(1+l*3);
                        neighbour(2+l*3) = Ry(m,n,o) + goal(2+l*3);
                        neighbour(3+l*3) = Rt(m,n,o) + goal(3+l*3);
                        predictor_data = [predictor_data; neighbour];
                        response_data = [response_data; goal];
                    end
                end
            end
        end
    end
end
num_Mdls = size(response_data,2);
gprMdls = cell(num_Mdls,1);
for i=1:num_Mdls
    gprMdls{i,1} = fitrgp(predictor_data,response_data(:,i)); 
    if getappdata(f,'canceling')
        break
    end
    waitbar(i/num_Mdls,f,sprintf('Training GP Model%2d of%3d',i,num_Mdls))
end
save(file_gprMdls, 'gprMdls');
delete(f)
%% Visualising training predictor data
if visualization
    close all
    figure(4)
    ytitles = ["x", "y", "\theta"];
    for i=1:3
        subplot(3,2,2*i-1)
        plot(predictor_data(:,i));      hold on 
        plot(response_data(:,i));   
        ylabel(ytitles(i));
    end
    subplot(1,2,2)
    plot(predictor_data(:,1), predictor_data(:,2),'r-');   hold on
    plot(response_data(:,1), response_data(:,2),'b.');
    %xlim([-1.500 2.500]);
    xlabel('$x$', 'interpreter', 'latex');
    ylabel('$y$', 'interpreter', 'latex');
end
%% 
% Predict the response corresponding to the rows of |xi| (resubstitution
% predictions) using the trained model. 
test_data = [];
for i=1:2
    tmp = resubPredict(gprMdls{i,1});
    test_data = [test_data tmp];
end
%%
%test_pattern = 1;
%zk=[x0 test_pattern];
zk=x0;
z = [zk];
zk1 = zk;
for j = 1:300
    for i=1:num_Mdls
        zk1(i) = predict(gprMdls{i,1}, zk);
    end
    z = [z; zk1];
    zk = zk1;
end

if visualization
    figure(1)
    hold on
    plot(z(:,1), z(:,2)); hold on
    plot(response_data(:,1), response_data(:,2),'b.');
    xlim([-2.500 2.500])
end

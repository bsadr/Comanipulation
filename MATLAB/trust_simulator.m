% Input data
PR = P_R';
%D = D';
size_time = size(time_span,1);

%%%%%%%%%%%%% Trust Parameters
% Trust model coefficients
AT1 = 1;
BT1 = 0.01;
CT1 = 0.01;
%CT1 = 0.5;
% Recursive Bielef variables
st = 400; % number of samples in belief distribution
bel_step = 1/st; % belief distribution step
t_v = 0:bel_step:1; % trust t(k) vector
t1_v = (0:bel_step:1)'; % trust t(k-1) vector
bel_size = size(t_v,2); % sizr_trust is the size of t
t = 1 - (0:bel_step:1);% trust t(k) matrix
t1 = 1- (0:bel_step:1)';% trust t(k) matrix
tt = 1- ones(bel_size,1)*(0:bel_step:1);
bel_f = ones(1,bel_size);
% P: p(T(k), T(k − 1), PR(k), PR(k − 1))
P= zeros(bel_size,bel_size,size_time);
% O_f: p(T(k), D(k))
O_f= zeros(bel_size,bel_size,size_time);
bel_bar= zeros(bel_size,bel_size,size_time);
sigma_t = 0.005;    % standard deviation for trust model
sigma_f = 0.1;      % standard deviation for trust observation
integ = 1:1:10000;

bel_f_max = zeros(1,size_time);
bel_f_max(1)=1;

% Offline Trust simulator
for k = 2:1:size_time
    p_k1(k) = BT1*PR(1,k)+CT1*PR(1,k-1);    
    if k>1
        % P: p(T(k), T(k − 1), PR(k), PR(k − 1)) 
        P(:,:,k) = ((1./(sqrt(2.*pi).*sigma_t)).*exp((-(t-(AT1*t1+p_k1(k))).^2)./(2.*sigma_t.^2)));
        P(:,:,k) = bel_step*P(:,:,k);            % Truncated normal distribution part starts from here.
        % O_f: p(T(k), D(k))
        O_f(:,:,k) = normpdf(D(k), tt, sigma_f);
        O_f(:,:,k)= bel_step*O_f(:,:,k);
        % bel_bar
        bel_bar(:,:,k) = O_f(:,:,k).* P(:,:,k) .* (ones(bel_size,1)*bel_f(k-1,:));
        % bel_f
        logprobs_mat_prior = repmat(log(bel_f(k-1,:)), bel_size, 1);
        logprobs_mat_propagate = log(P(:,:,k));
        logprobs_mat_observe = log(O_f(:,:,k));
        logprobs_mat_local_factors = logprobs_mat_propagate + logprobs_mat_observe;
        prob_mat_posterior = exp(logprobs_mat_prior + logprobs_mat_local_factors);
        probs_posterior = trapz(prob_mat_posterior, 2)';
        norm_posterior = trapz(probs_posterior);
        norm_posterior = norm_posterior / bel_size;
        x_latest_pmf = probs_posterior ./ norm_posterior;
        bel_f(k,:) = x_latest_pmf;

        [~,Im] = max(x_latest_pmf);
        bel_f_max(k) = (Im-1)*bel_step;
    end
end

bel = bel_f';
[trust_most,trust_most1] = max(bel);
trust_most1 = trust_most1(1,1:size_time)/(st+1);
%% Plot Trust
a = k;

az = 0;
el = 90;

figure1 = figure;
subplot(4,1,1)
plot(time_span,PR(1,1:size_time),'k','LineWidth',1)
axis([0 10 0 inf])
grid on
ylabel('P_R','FontSize',18)
set(gca,'FontSize',18)

subplot(4,1,2)
tout_M1 = time_span.*ones(size_time,bel_size);
t_v_M1 = t_v.*ones(size_time,bel_size);
mesh(tout_M1,t_v_M1,bel_step*bel_f(1:size_time(),:))
view([az,el])
caxis([0 0.04])
cb = colorbar('west','Color','w','location','west');
ylabel('bel_f','FontSize',18)
 axis([0 10 0 inf])
set(gca,'FontSize',18)

subplot(4,1,3)
plot(time_span,bel_f_max(1,1:size_time),'k','LineWidth',1) 
hold on 
%plot(time_span,trust_most1(1,1:size_time),'b--','LineWidth',.5) 
axis([0 10 0 inf])
grid on
ylabel('\mu_{bel_f}','FontSize',18)
set(gca,'FontSize',18)



subplot(4,1,4)
plot(time_span,D(1:size_time));
xlabel('t','FontSize',18)
ylabel('D','FontSize',18)
grid on
axis([0 10 0 1])
set(gca,'FontSize',18)


figure1.PaperUnits = 'inches';
figure1.PaperPosition = [0 0 8 6];
print('result_trust','-depsc','-r0')
print('result_trust','-dpdf','-r0')
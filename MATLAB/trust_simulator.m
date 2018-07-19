
PR = P_R';
%D = D';
size_time1 = size(time_span,1);


st = 400;
step = 1/st;
t_v = 0:step:1; % trust t(k) vector
t1_v = (0:step:1)'; % trust t(k-1) vector
[~,size_trust] = size(t_v); % sizr_trust is the size of t
t = 1 - (0:step:1);% trust t(k) matrix
t1 = 1- (0:step:1)';% trust t(k) matrix
tt = 1- ones(size_trust,1)*(0:step:1);

bel_f = ones(1,size_trust);
% P: p(T(k), T(k − 1), PR(k), PR(k − 1))
P= zeros(size_trust,size_trust,1000);
% O_f: p(T(k), D(k))
O_f= zeros(size_trust,size_trust,1000);
bel_bar= zeros(size_trust,size_trust,1000);

AT1 = 1;
BT1 = 0.01;
CT1 = 0.01;
%CT1 = 0.5;
sigma_t = 0.005;
sigma_f = 0.1;

integ = 1:1:10000;

c_k1(1,:) = 2;

for k = 2:1:size_time1
    p_k1(k) = BT1*PR(1,k)+CT1*PR(1,k-1);    
    if k>1
        % P: p(T(k), T(k − 1), PR(k), PR(k − 1)) 
        P(:,:,k) = ((1./(sqrt(2.*pi).*sigma_t)).*exp((-(t-(AT1*t1+p_k1(k))).^2)./(2.*sigma_t.^2)));
        P(:,:,k) = step*P(:,:,k);            % Truncated normal distribution part starts from here.
        % O_f: p(T(k), D(k))
        O_f(:,:,k) = normpdf(D(k), tt, sigma_f);
        O_f(:,:,k)= step*O_f(:,:,k);
        % bel_bar
        if k == 2 % initial point
            bel_bar(:,:,k) = O_f(:,:,k).* P(:,:,k) .* bel_f(k-1); % when k = 2, bel_f(1) = 1
        else
            bel_bar(:,:,k) = O_f(:,:,k).* P(:,:,k) .* (ones(size_trust,1)*bel_f(k-1,:));
        end       
        % bel_f
        logprobs_mat_prior = repmat(log(bel_f(k-1,:)), size_trust, 1);
        logprobs_mat_propagate = log(P(:,:,k));
        logprobs_mat_observe = log(O_f(:,:,k));
        logprobs_mat_local_factors = logprobs_mat_propagate + logprobs_mat_observe;
        prob_mat_posterior = exp(logprobs_mat_prior + logprobs_mat_local_factors);
        probs_posterior = trapz(prob_mat_posterior, 2)';
        norm_posterior = trapz(probs_posterior);
        norm_posterior = norm_posterior / size_trust;
        x_latest_pmf = probs_posterior ./ norm_posterior;
        bel_f(k,:) = x_latest_pmf;
    end
end

bel = bel_f';
[trust_most,trust_most1] = max(bel);
trust_most1 = trust_most1(1,1:size_time1)/401;
%% Plot Trust
a = k;

az = 0;
el = 90;

figure1 = figure;
subplot(3,1,1)

plot(time_span,PR(1,1:size_time1),'k','LineWidth',2)
axis([0 10 0 inf])
grid on
ylabel('P_R','FontSize',18)
set(gca,'FontSize',18)
subplot(3,1,2)
tout_M1 = time_span.*ones(size_time1,size_trust);
t_v_M1 = t_v.*ones(size_time1,size_trust);
mesh(tout_M1,t_v_M1,step*bel_f(1:size_time1(),:))
view([az,el])
caxis([0 0.04])
cb = colorbar('west','Color','w','location','west');
ylabel('bel_f','FontSize',18)
 axis([0 10 0 inf])
 set(gca,'FontSize',18)
 
 
% subplot(7,1,3)
% plot(integ(1,1:size_td1),tb_trust1,'b')
% hold on
% grid on
% ylabel('Maximum Likelihood Trust','FontSize',18)
% set(gca,'FontSize',16)
% axis([0 size_time1 0 1])

% subplot(7,1,4)
% plot(integ(1,1:size_td1),t_d1,'b')
% hold on
% plot(thres_x1,thres_y1,'k--')
% grid on
% ylabel('Trust Change','FontSize',18)
% set(gca,'FontSize',16)
% axis([0 size_time1 -inf inf]) 

% subplot(4,1,3)
% plot(tout(2:size_time1,1),m_k1(1,2:size_time1),'*');
% % xlabel('t','FontSize',18)
% ylabel('m_1','FontSize',18)
% grid on
% axis([0 size_time1 0 1])
% set(gca,'FontSize',18)
% subplot(7,1,6)
% plot(tout(2:size_time1,1),c_k1(1,2:size_time1),'*');
% % xlabel('t','FontSize',18)
% ylabel('c_1','FontSize',18)
% grid on
% axis([0 size_time1 -1 1])
%  set(gca,'FontSize',16)
subplot(3,1,3)
plot(time_span,D(1:size_time1));
xlabel('t','FontSize',18)
ylabel('D','FontSize',18)
grid on
axis([0 10 0 1])
set(gca,'FontSize',18)


figure1.PaperUnits = 'inches';
figure1.PaperPosition = [0 0 8 6];
print('result_trust','-depsc','-r0')
print('result_trust','-dpdf','-r0')
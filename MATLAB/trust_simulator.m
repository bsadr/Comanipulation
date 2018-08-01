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
bel_size = size(t_v,2); % sizr_trust is the size of t
t = 1 - (0:bel_step:1);% trust t(k) matrix
t1 = 1- (0:bel_step:1)';% trust t(k) matrix
tt = 1- ones(bel_size,1)*(0:bel_step:1);
bel_f = ones(1,bel_size);
% P: p(T(k), T(k − 1), PR(k), PR(k − 1))
P= zeros(bel_size,bel_size,size_time);
% O_f: p(T(k), D(k))
O_f= zeros(bel_size,bel_size,size_time);
sigma_t = 0.01;    % standard deviation for trust model
sigma_f = 0.2;      % standard deviation for trust observation

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
        % mu value of bel_f
        [~,Im] = max(x_latest_pmf);
        bel_f_max(k) = (Im-1)*bel_step;
    end
end
%% Plot Trust
figure1 = figure(1);
subplot(4,1,1)
plot(time_span,PR(1,1:size_time),'b')
axis([0 8 0 1])
grid off
box off
ylabel('$P_R$', 'Interpreter', 'latex')

% reduced_horizon=1:size(bel_f,1);
% mask1 = repmat(reduced_horizon', 1,size(bel_f,2));
% mask = mod(mask1,3)==1;
% tmp=bel_f(mask);
% belf_f=reshape(tmp, size(tmp,1)/size(mask,2), size(mask,2));
% tmp=tout_M1(mask);
% tout_M1=reshape(tmp, size(tmp,1)/size(mask,2), size(mask,2));
% tmp=t_v_M1(mask);
% t_v_M1=reshape(tmp, size(tmp,1)/size(mask,2), size(mask,2));


az = 0;
el = 90;
subplot(4,1,2)
tout_M1 = time_span.*ones(size_time,bel_size);
t_v_M1 = t_v.*ones(size_time,bel_size);
% surf(tout_M1(2:end-1, 2:end-1),t_v_M1(2:end-1, 2:end-1),bel_step*bel_f(1:size_time(),:))
mesh(tout_M1(2:end-1, 10:end),t_v_M1(2:end-1, 10:end),bel_step*bel_f(2:end-1, 10:end))
% colorMap = [ones(256,1), linspace(1,0,256)', ones(256,1)];
colorMap = [linspace(1,0,20)', linspace(1,0,20)', ones(20,1)];
colormap(colorMap)
% colormap summer
shading interp
%   set(h,'linestyle','none','facecolor',[0 .7 0]);
view([az,el])
caxis([0 0.04])
cb = colorbar('Color','k','location','west', ...
    'Position', [0.925 0.55 0.02 0.15]);

ylabel('$bel_f$', 'Interpreter', 'latex')
axis([0 8 0 inf])
box off
grid off


subplot(4,1,3)
plot(time_span,D(1:size_time),'b');
grid on
axis([0 8 0 1])
grid off
box off
ylabel('$D$', 'Interpreter', 'latex')


subplot(4,1,4)
plot(time_span,Mode(1:size_time),'b');
grid on
axis([0 8 0 1])
grid off
box off
ylabel('mode', 'Interpreter', 'latex')


%%
% figure1.PaperUnits = 'inches';
% figure1.PaperSize = [6 5];
%  figure1.PaperPosition = [-0.1 0 6 5];
print('result_trust','-depsc','-r0')
print('result_trust','-dpdf','-r0')
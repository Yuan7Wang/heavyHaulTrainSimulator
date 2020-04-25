% test_train__offline.m
%
% clear;clc 
% % load TrainDataSet0813.mat QLOC W_obj
% load alldata_20180813.mat QLOC W_obj
% load alldata_20180813b.mat QLOC % W_obj 

% QT = QLOC;

%% 

% size_ = size(QLOC);
% 
% ind_ = 1:31:size_(1);
% 
% cmat = (QLOC(ind_, 6)-min(QLOC(ind_, 6))).^10;
% cmat = cmat-min(cmat);
% cmat = cmat/max(cmat);
% 
% figure(1);clf
% scatter(QLOC(ind_, 1), QLOC(ind_, 2),10, cmat, '.')
% colorbar
% 
% figure(2);clf
% scatter(QLOC(ind_, 4), QLOC(ind_, 5),10, cmat, '.')
% colorbar
% 
% ind_ = 30:50:size_(1);
% figure(1);clf
% hold on;
% scatter(QLOC(ind_, 2), QLOC(ind_, 6), 'b.')
% scatter(QLOC(ind_, 5), QLOC(ind_, 6), 'r.') 
% 
% figure(3);clf
% hold on;
% plot(QLOC(ind_, 6), 'c.')
% plot(get_mean(QLOC(ind_, 6), 500, 10), 'r-')

n_singlE = 50;
% QT(n_singlE:n_singlE, 6) = -QT(n_singlE:n_singlE, 6);

Ne = 50;
QLOC = permute(reshape(QT, n_singlE*Ne, [], 6), [1 3 2]);
sizeQ = size(QLOC);

episode_loops = sizeQ(3);

% QLOC__ = QLOC_(:, :, 1);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%

%% 

% nl = 2; % 两个机车
% nNegNotch = 0;
% nPosNotch = 12; % Notch 个数
% nNotch = nNegNotch + nPosNotch + 1;


%%
%% initialization


%% 

detaS = 400; % unit m
detaV = 40;  % unit km/h
maxV = 100;  % unit km/h
maxS = ceil(max(QLOC(:, 4))/detaS+1)*detaS; % unit m
nTiling = 50;  % number of tiling 
% nAction = (nPosNotch + nNegNotch + 1)^nl;
nl = 2;
nAction = max(QT(:, 3));
nNodes = 100;

nNotch = round(sqrt(nAction));

% W_obj = get_Wobj_linear_Nonlinear_Offline(nNodes, maxS, maxV, detaS, detaV, nTiling, nAction);
W_obj = get_Wobj_linear_Offline(maxS, maxV, detaS, detaV, nTiling, nl, nNotch);

%% Reinforcement Learning paramerters

gamma = 0.98;
alpha = 0.001;

W_obj_copy = W_obj;
wobj_copy_num = 1;
wobj_copy_count = 0;



%% experience replay 

TLOOP = 1;

replay_buffer_size = 40;
% replay_buffer_size = episode_loops;
replay_point = 0;    % 数据记录指针
replay_flag = 0;     % 标识记录总数
replay_batch = sizeQ(1);
% Rs_record = zeros(in_epi_number, Ne);    % 每一步的回报
% Qloc = zeros(in_epi_number, 6, Ne);    % 每一步的 state-action
% QLOC = zeros(replay_batch * replay_buffer_size, 6)*NaN;    % 每一步的 state-action

nQloc = sizeQ(2);
Qloc = zeros(replay_batch, nQloc);    % 每一步的 state-action
local_point = 0;
Qtmp = zeros(replay_batch * replay_buffer_size, nQloc)*NaN;    % 每一步的 state-action

%% Recorder

err_record = zeros(episode_loops * TLOOP, 1);
err_count = 0;

%% 低速、超速保护机制

%% weight

%% initialize figure

state_ = [1000 1000; 60 70];
current_ind_sv = coding_index_transform(W_obj, state_);
vals = sum(W_obj.W(:, current_ind_sv(:, 1)),2);
vtmp = reshape(vals, nNotch, nNotch);

figure(2);clf
subplot(121);
hmesh1 = mesh(vtmp);
set(gca, 'zdir', 'reverse');
% set(gca, 'zlim', [-120 10]);
title(state_(:, 1));
view(-159, 58)
subplot(122);
hmesh2 = mesh(vtmp);
set(gca, 'zdir', 'reverse');
% set(gca, 'zlim', [-120 10]);
title(state_(:, 2));
view(-159, 58)

figure(3);clf
h_err = semilogy(err_record);

%% 主循环

tic
hwait = waitbar(0, 'Processing ...');
for ktr = 1:TLOOP
    for etr = 1 : episode_loops
        
        if rem(etr, 10) == 0
            waitbar(etr/episode_loops, hwait, [sprintf('Processing ... %2.1f',etr/episode_loops*100) '%']);
        end
        
        wobj_copy_count = wobj_copy_count + 1;
        if wobj_copy_count == wobj_copy_num
            W_obj_copy = W_obj;      % 克隆一个当前值，并用于选择 aciton
            wobj_copy_count = 0;
        end
        
        %% initialization
        
        %%
        
        Qloc = QLOC(:, :, etr);
%         Qloc = QLOC(:, :, ceil(rand*episode_loops));
        
        %% 当一次探索完成后，更新Q矩阵
        
        %     W_obj = update_ActionFunction_linear_Offline(W_obj, W_obj_copy, Qloc(end:-1:1, :), alpha);
        
        %% 滚动追加最近的 replay_buffer_size 个数据组
        
        replay_point = replay_point + 1;
        if replay_point > replay_buffer_size
            replay_point = 1;
        end
        
        Qtmp((replay_point-1)*replay_batch+1:replay_point*replay_batch, :) = Qloc;
        
        if replay_flag < replay_point*replay_batch
            replay_flag = replay_point*replay_batch;
        end
        
        % experience replay
        uniform_ind = ceil(rand(replay_batch*10, 1)*replay_flag);
        [W_obj, err_norm] = update_ActionFunction_linear_Offline(W_obj, Qtmp(uniform_ind, :), alpha, gamma);
        
        err_count = err_count+1;
        err_record(err_count) = err_norm;
        
        if rem(etr, 5) == 0
            vtmp = reshape(sum(W_obj.W(:, current_ind_sv(:, 1)),2), nNotch, nNotch);
            set(hmesh1, 'zdata', vtmp);
            
            vtmp = reshape(sum(W_obj.W(:, current_ind_sv(:, 2)),2), nNotch, nNotch);
            set(hmesh2, 'zdata', vtmp);
            
            set(h_err, 'ydata', err_record);
            drawnow;
            pause(0.0000001);
        end
    end
%     replay_flag = 0;     % 标识记录总数
%     replay_point = 0;
end

close(hwait)
tooc = toc;
toc

%% initialize figure

figure(102);clf 
hmesh1 = mesh(vtmp);
set(gca, 'zdir', 'reverse');
% set(gca, 'zlim', [-120 10]);
view(-159, 58)
% set(gca, 'zlim', [-50 20]);

s_ = 100:10:3000;
state_ = [s_' zeros(length(s_), 1)+60]';
current_ind_sv = coding_index_transform(W_obj, state_);

Tmp = zeros(nNotch*nNotch, length(s_));
Ind = zeros( length(s_), 2);

index_ = reshape(1:nNotch*nNotch, nNotch, nNotch);

for itr = 1:length(s_)
    tmp = sum(W_obj.W(:, current_ind_sv(:, itr)),2);
    Tmp(:, itr) = tmp;
    
    [max_, ind_] = max(tmp);
    Ind(itr, :) = [max_, ind_];
    
%     vtmp = reshape(sum(W_obj.W(:, current_ind_sv(:, itr)),2), nNotch, nNotch);
%     set(hmesh1, 'zdata', vtmp);
%     title(s_(itr));
%     drawnow;
%     pause(0.1);
end


% nei = 1;
% figure(101);clf
% hold on;
% hnei(1) = plot(initial_notch(1, :), initial_notch(2, :), 'ks', 'linewidth', 2);
% hnei(2) = plot(loc_Notchs1(:), loc_Notchs2(:), 'co');
% hnei(3) = plot(loc_Notchs1(:, nei), loc_Notchs2(:, nei), 'r.-');
% set(gca, 'xlim', [-1 1]*12, 'ylim', [-1 1]*12);
% box on;
% grid on;




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

Ne = 20;
QLOC = permute(reshape(QT, 50*Ne, [], 6), [1 3 2]);
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
nTiling = 40;  % number of tiling 
% nAction = (nPosNotch + nNegNotch + 1)^nl;
nl = 2;
nAction = max(QT(:, 3));
nNodes = 100;

nNotch = round(sqrt(nAction));

W_obj = get_Wobj_linear_Nonlinear_Offline(nNodes, maxS, maxV, detaS, detaV, nTiling, nAction);

%% Reinforcement Learning paramerters

gamma = 1;
alpha = 0.001;

W_obj_copy = W_obj;
wobj_copy_num = 3;
wobj_copy_count = 0;


%% Recorder

%% experience replay 

replay_buffer_size = 20;
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

%% 低速、超速保护机制

%% weight

%% initialize figure

%% 主循环

tic
hwait = waitbar(0, 'Processing ...');
for etr = 1 : episode_loops
    
    if rem(etr, 10) == 0
        waitbar(etr/episode_loops, hwait, [sprintf('Processing ... %2.1f',etr/episode_loops*100) '%']);
    end
    
    wobj_copy_count = wobj_copy_count + 1;
    if wobj_copy_count == wobj_copy_num  
        W_obj_copy = W_obj;      % 克隆一个当前值，并用于选择 aciton
        wobj_copy_num = 0;
    end
    
    %% initialization
    
    %% 
    
    Qloc = QLOC(:, :, etr);
    
    %% 当一次探索完成后，更新Q矩阵
    
%     W_obj = update_ActionFunction_Nonlinear_Offline(W_obj, W_obj_copy, Qloc(end:-1:1, :), alpha);
    
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
    uniform_ind = ceil(rand(replay_batch*min([replay_point 3]), 1)*replay_flag);
    W_obj = update_ActionFunction_Nonlinear_Offline(W_obj, W_obj_copy, Qtmp(uniform_ind, :), alpha);
    

end

close(hwait)
tooc = toc;
toc

%% initialize figure

current_ind_sv = coding_index_transform(W_obj, [350; 60]);

vals = W_obj.cal_final_output(W_obj, W_obj.cal_hidden_output(W_obj, current_ind_sv), 1:nNotch^2);
vtmp = reshape(vals, nNotch, nNotch);

figure(2);clf
hmesh = mesh(vtmp);
set(gca, 'zdir', 'reverse');
% set(gca, 'zlim', [-120 10]);
title(1);
view(-159, 58)

% nei = 1;
% figure(101);clf
% hold on;
% hnei(1) = plot(initial_notch(1, :), initial_notch(2, :), 'ks', 'linewidth', 2);
% hnei(2) = plot(loc_Notchs1(:), loc_Notchs2(:), 'co');
% hnei(3) = plot(loc_Notchs1(:, nei), loc_Notchs2(:, nei), 'r.-');
% set(gca, 'xlim', [-1 1]*12, 'ylim', [-1 1]*12);
% box on;
% grid on;




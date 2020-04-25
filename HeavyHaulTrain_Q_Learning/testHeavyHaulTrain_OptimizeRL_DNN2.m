% testHeavyHaulTrain_OptimizeRL_DNN.m

% clear; clc

%% 

total_step_max = 1e4;
maxStep = 5e2; 


%%

dt = 0.1;
% T = 100;
detaT = 10;
% tvec = dt:dt:T;
nT = round(detaT/dt);

TotalLength = 15e3;
% initial_posRange = TotalLength*0.1; 
% initial_position = rand*initial_posRange;
% initial_speed = 60/3.6;
% initial_notch = 7;

% hHpeed_Limits = [95 65 65]/3.6;
hHpeed_Limits = [80   65 85]/3.6;
lowSpeedLim = 15/3.6;
speed_ChangePoint = TotalLength*[2/10  6/10];

nSlim = length(speed_ChangePoint);


% MotorNotch = zeros(Nl, Ne) + round(rand(Nl, Ne)*24-12);     % 每个机车有独立的级位 

% TBCL_force

%% initialization of Vehicles 

Nt = 200+2;
Ne = 64;

X = zeros(Nt*2, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
U = zeros(Nt, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
countStepNe = zeros(1, Ne); % 记录当前 episode 执行了多少步了
bool_terminate = zeros(1, Ne); 
bool_restart = zeros(1, Ne);

% maxStep = 2e2; % 最大步长，超过该步长程序终止
phase_buffer = zeros(maxStep, 2, Ne)*NaN; % 记录轨迹

% X(1:Nt, :) = initial_speed;     % 初速度

carType = zeros(Nt, 1) == 1;  % 车辆类型，机车还是拖车
carType([1 end]) = true;        % 一头一尾两个机车

nl = sum(carType);
Nw = sum(~carType);

[Ad, Bd, TBcl, airTimeDelay, Abr, LenTrain, mTrainGroup, C0, Ca, KK, DD] = ...
    initialize_locomotive_char_V2(dt, carType, Ne);

% Bd = -Bd * 1e3;

LenTrains = LenTrain'*ones(1, Ne);

% nl = 2; % 两个机车
nNegNotch = 12;
nPosNotch = 12; % Notch 个数
% nNotch = nNegNotch + nPosNotch + 1;


%% 核心学习参数与函数

detaS = 400; % unit m
detaV = 40;  % unit km/h
maxV = [0 120];  % unit km/h
maxS = [0 TotalLength]; % unit m
nTiling = 30;  % number of tiling 
% nAction = (nPosNotch + nNegNotch + 1)^nl;
nHiddens = [256  ]; %#ok<NBRAK>
% nHiddens = [256 ];  
W_obj = get_Wobj_4HeavyHaul_DNN2(nHiddens, maxS, maxV, detaS, detaV, nTiling, nl, nPosNotch, nNegNotch);

% W__tmp = W_obj.W;
% save W_obj_W.mat W__tmp;

% load W_obj_W.mat W__tmp;
% W_obj.W = W__tmp;

%% Load the infrastructure

% rampList = [-10000 0; 0 0.01; 1500 -0.01; 3000 0; 10000 0; 20000 0; 50000 0];
% rampList = [-10000 0; 0 0.01; 1500 -0.01; 3000 0; 10000 0; 20000 0; 50000 0];

rampList = [-10000 0; 
    0 0.005; 
    5000 -0.005;
    10000 0.005; 
    15000 0; 
    30000 0; 
    50000 0];

rampList(:, 2) = - rampList(:, 2);

% rampList = [-10000 0; 0 0.01; 1500 -0.01; 3000 0; 4500 0.01; 6000 0;  20000 0; 50000 0; 80000 0; 120000 0];

%% Air break

% air_timer = zeros(Nw, Ne);        % 空气制动计时器, 当前的
% air_timer_ = zeros(Nw, Ne);        % 空气制动计时器， 上一步的
% % AirBrkNotch = round(-rand(1, Ne)*2);       % 空气制动，-2, -1, 0;
% AirBrkNotch = round(-rand(1, Ne)*0);       % 空气制动，-2, -1, 0;
% AirBrkNotch_ = zeros(1, Ne);      % 空气制动之前一次控制量，1表示制动，0表示缓解
% 
% F_target = zeros(Nw, Ne);
% currentF = zeros(Nw, Ne);

%% Reinforcement Learning paramerters

gamma = 0.95;
alpha = 5e-3/nTiling/5;

W_obj.gamma = gamma;
W_obj.alpha = alpha;

epsilons = [linspace(1e-1, 5e-2, total_step_max/2) linspace(1e-2, 1e-2, total_step_max/2) 0];
% epsilons = [linspace(1e-3, 1e-3, total_step_max/2) linspace(1e-2, 1e-2, total_step_max/2) 0];
epsilon = epsilons(1);

epsilons2 = [linspace(10e-1, 1e-1, total_step_max) 0];
% epsilons3 = [linspace(10e-1, 5e-1, total_step_max/2) linspace(4e-1, 4e-1, total_step_max/2) 0];
epsilons3 = [linspace(10e-1, 10e-1, total_step_max) 0];
% epsilons3 = [linspace(10e-1, 10e-1, total_step_max/2) linspace(2e-2, 2e-2, total_step_max/2) 0];
% epsilons2 = [linspace(10e-1, 5e-1, total_step_max/2) linspace(1e-1, 1e-1, total_step_max/2) 0];
epsilon2 = epsilons2(1);

initial_posRange = TotalLength*epsilons3(1);


%% 环境误差与测量误差

noise_force = randn(total_step_max+1, Ne);
for itr = 1:Ne
    noise_force(:, itr) = get_mean(noise_force(:, itr), 100, 1);
    noise_force(:, itr) = noise_force(:, itr)/std(noise_force(:, itr))*0.05;
end

noise_sensor = randn(total_step_max+1, Ne);
for itr = 1:Ne
    noise_sensor(:, itr) = get_mean(noise_sensor(:, itr), 100, 1);
    noise_sensor(:, itr) = noise_sensor(:, itr)/std(noise_sensor(:, itr))*0.05;
end



%% Recorder

% X_Recorder = zeros(length(tvec), Ne);

F_l =  zeros(round(detaT/dt), Ne);
F_w =  zeros(round(detaT/dt), Ne);
F_c =  zeros(round(detaT/dt), Ne);
errV =  zeros(round(detaT/dt), Ne);

R_Recorder = zeros(maxStep, Ne);

%% experience replay 

replay_buffer_size = 500;
% replay_buffer_size = episode_loops;
replay_point = 0;    % 数据记录指针
replay_flag = 0;     % 标识记录总数

% in_epi_number = round(T/detaT);
in_epi_number = 1;
replay_batch = in_epi_number * Ne;
Rs_record = zeros(in_epi_number, Ne);    % 每一步的回报 

nQloc = 2 + nl + 2 + 1;
Qloc = zeros(replay_batch, nQloc);    % 每一步的 state-action
local_point = 0;
QLOC = zeros(replay_batch * replay_buffer_size, nQloc)*NaN;    % 每一步的 state-action

%% Target Network Setup

W_obj_copy = W_obj;
wobj_copy_num = 80;
wobj_copy_count = 0;

%% 低速、超速惩罚

lowSpeedFlag = zeros(1, Ne) == 1;
highSpeedFlag = zeros(1, Ne) == 1;

a = -0.1;
b = -1;
large_neg_reward = -1e1;    % 一个很大的负reward
higSpeed_r = @(dv) (a*(abs(dv)+2).^3 + b).*(dv > 0);
lowSpeed_r = @(dv) (a*(abs(dv)+2).^3 + b).*(dv < 0);
% higSpeed_r = @(dv) large_neg_reward.*(dv > 0);
% lowSpeed_r = @(dv) large_neg_reward.*(dv < 0);

% figure; xx = linspace(-10,10, 100)'/3.6; plot(xx*3.6, higSpeed_r(xx))
% figure; xx = linspace(-10,10, 100)'/3.6; plot(xx*3.6, lowSpeed_r(xx))

% --------- weight

% reward_weigh = [1/1e7 1/1e7]*5e-1;   % 车钩力， 总力的积分
reward_weigh = [1/1e8 1/1e7]*1e0;   % 车钩力， 总力的积分

v_speeds = (30:1:85)/3.6;

%% initialization

% -------------------- %
% air_Notchs =  zeros(length(tvec), Ne)*NaN;
% loc_Notchs1 =  zeros(in_epi_number, Ne)*NaN;
% loc_Notchs2 =  zeros(in_epi_number, Ne)*NaN;
% %
% U_recorder = zeros(Nt, length(tvec))*NaN;
% L_recorder = zeros(2, length(tvec))*NaN;
% V_recorder = zeros(Nt, length(tvec))*NaN;
% RP_recorder = zeros(Nt, length(tvec))*NaN;

% -------------------- % 

Rspeed = ones(2, Ne)*hHpeed_Limits(1);    % 记录车头车尾的速度
% Lspeed = ones(2, Ne)*hHpeed_Limits(1);    % 记录车头车尾的限值速度

% X = zeros(Nt*2, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置 

initial_position = rand(1, Ne)*initial_posRange;
if rand < 0.01  % 有 1% 概率从0开始
    [~, min_] = min(initial_position);
    initial_position(min_) = 0;
end
X((1:Nt)+Nt, :) = ones(Nt, 1)*initial_position;     % 初位置

% hHpeed_Limits = [70/3.6 60/3.6 ];
% lowSpeedLim = 20/3.6;

Lspeed = W_obj.obtain_speed_limit(X([1+Nt Nt*2], :), speed_ChangePoint, hHpeed_Limits);

% bo_tmp = X([1+Nt Nt*2], :) < speed_ChangePoint(1);
% Lspeed(bo_tmp) = hHpeed_Limits(1);
% % Lspeed(~bo_tmp) = hHpeed_Limits(2);
% for itr = 2:nSlim
%     bo_tmp = ~bo_tmp & X([1+Nt Nt*2], :) < speed_ChangePoint(itr); 
%     Lspeed(bo_tmp) = hHpeed_Limits(itr);
% end
% bo_tmp = X([1+Nt Nt*2], :) >= speed_ChangePoint(itr);
% Lspeed(bo_tmp) = hHpeed_Limits(nSlim+1);
% 
% Lspeed(bo_tmp) = hHpeed_Limits(1);
% Lspeed(~bo_tmp) = hHpeed_Limits(2);

initial_speed = (rand(1, Ne).*(max(Lspeed) - lowSpeedLim)+lowSpeedLim);%*3.6;
X(1:Nt, :) = ones(Nt, 1)*initial_speed;     % 初速度

% initial_notch = ceil(rand(1, Ne)*10)-1;
initial_notch = round((initial_speed*3.6+randn*5)/10);
initial_notch(initial_notch > 12) = 12;
initial_notch(initial_notch < -12) = -12;
MotorNotch = zeros(nl, Ne) + initial_notch;     % 每个机车有独立的级位

v_tmp = X(1, :)*3.6;
s_tmp = X(Nt+1, :);
actions = W_obj.choose_next_action(W_obj, [s_tmp; v_tmp], MotorNotch, epsilon);

MotorNotch = actions;

countStepNe = countStepNe+1;
for i = 1:Ne
    phase_buffer(countStepNe(i), 1:2, i) = X([Nt+1 1], i);   % 起始状态
end

%% initialize the figure

figure(10000);clf
hold on; box on;

h_traj = zeros(2, Ne);
for i = 1:Ne
    h_traj(2, i) = plot(phase_buffer(:, 1, i), phase_buffer(:, 2, i)*3.6, '.-');
end;

ind1122 = [1; 1]*(1:nSlim);
ind112233 = [1; 1]*(1:nSlim+1);
tmp = speed_ChangePoint(ind1122);
tmp(end+1, :) = NaN;
tmp = tmp([1 3 2], :);
x_ = [0; tmp(:); TotalLength];
tmp = hHpeed_Limits(ind112233)*3.6;
tmp(end+1, :) = NaN;
tmp(end) = [];
y_ = tmp(:);
plot(x_, y_, 'k', 'linewidth', 3);
plot([0  TotalLength], lowSpeedLim([1 1])*3.6, 'k', 'linewidth', 3);
set(gca, 'xlim', [0 TotalLength], 'ylim', maxV)

%% 

s = linspace(W_obj.rangS(1), W_obj.rangS(2), 400);
v = linspace(W_obj.rangV(1), W_obj.rangV(2), 50);
[Ms, Mv] = meshgrid(s, v);
% stateSet = [Ms(:), Mv(:)]';

current_ind_sv = W_obj.coding_index_transform(W_obj, [Ms(:), Mv(:)]');

Val = zeros(size(Ms));
% Val = W_obj.max_Q_value_of_State(W_obj, [Ms(:), Mv(:)]');
Np = [6; 6];
N_ind = W_obj.coding_action_transform(W_obj, Np);
for itr = 1:numel(Ms)
    Val(itr) = W_obj.action_Q_value_of_State(W_obj, current_ind_sv(:, itr), N_ind); 
end

figure(10001);clf
hmesh = mesh(Ms, Mv, -log10(abs(Val)));
view(-23, 52)
set(gca, 'zdir', 'reverse')

figure(10002);clf
contour(Ms, Mv, -log10(abs(Val)), 20)
%% 

% s = linspace(W_obj.rangS(1), W_obj.rangS(2), 100);
% v = linspace(W_obj.rangV(1), W_obj.rangV(2), 30);
% [Ms, Mv] = meshgrid(s, v);
% % stateSet = [Ms(:), Mv(:)]';
% current_ind_sv = W_obj.coding_index_transform(W_obj, [Ms(:), Mv(:)]');
% 
% Ainds = find(W_obj.ActionSet(1, :) > 4 & W_obj.ActionSet(2, :) > 4);
% 
% nMs = numel(Ms);
% Val = zeros(size(Ms));
% Action = zeros(size(Ms));
% for itr = 1:nMs
%     if rem(itr, 100) == 0
%         disp(itr/nMs);
%     end
%     [val, action] = W_obj.max_Q_value_of_State(W_obj, current_ind_sv(:, itr), W_obj.AllActionCode(:, Ainds));
%     Val(itr) = val;
%     Action(itr) = action;
% end
%     
% figure(10002);clf
% hmeshMax = mesh(Ms, Mv, Val);
% view(-23, 52)
% set(gca, 'zdir', 'reverse')

%% 
current_ind_SomeState1 = W_obj.coding_index_transform(W_obj, [2000 40/3.6]');
current_ind_SomeState2 = W_obj.coding_index_transform(W_obj, [6000 40/3.6]');  
figure(10003);clf
subplot(121);
hidden_out_ = W_obj.cal_hidden_output(W_obj, current_ind_SomeState1);
val = reshape(W_obj.cal_final_output(W_obj, hidden_out_{end}), 25, []);
hActionVal1 = mesh(W_obj.Ax, W_obj.Ay, val);
% view(-2, 41)
view(-48, 67)
% set(gca, 'zdir', 'reverse')
subplot(122);
hidden_out_ = W_obj.cal_hidden_output(W_obj, current_ind_SomeState2);
val = reshape(W_obj.cal_final_output(W_obj, hidden_out_{end}), 25, []);
hActionVal2 = mesh(W_obj.Ax, W_obj.Ay, val);
view(-48, 67)
% set(gca, 'zdir', 'reverse')0


%% 主循环

total_step_count = 0;

tic
e_count = 1;
while total_step_count <= total_step_max 
    %%
    
    total_step_count = total_step_count+1;
    epsilon  = epsilons(total_step_count);
    epsilon2  = epsilons2(total_step_count);
    initial_posRange = TotalLength*epsilons3(total_step_count); 
    
    
    if rem(total_step_count, 20) == 0  % update_plot 
        disp(total_step_count/total_step_max*100);
        for i = 1:Ne 
            set(h_traj(2, i), 'xdata', phase_buffer(:, 1, i), 'ydata', phase_buffer(:, 2, i)*3.6);
        end
        %
%         hidden_out_ = W_obj.cal_hidden_output(W_obj, current_ind_SomeState1);
%         val = reshape(W_obj.cal_final_output(W_obj, hidden_out_{end}), 25, []);
%         set(hActionVal1, 'zdata', val);
%         hidden_out_ = W_obj.cal_hidden_output(W_obj, current_ind_SomeState2);
%         val = reshape(W_obj.cal_final_output(W_obj, hidden_out_{end}), 25, []);
%         set(hActionVal2, 'zdata', val);
        drawnow;
        pause(0.00001);
    end
    
    wobj_copy_count = wobj_copy_count + 1;
    if wobj_copy_count == wobj_copy_num  
        W_obj_copy = W_obj;      % 克隆一个当前值，并用于选择 aciton
        wobj_copy_count = 0;
    end
    
%     epsilon  = 0.1;
    
    R_tmp = zeros(2, Ne); 
    for itr = 1:nT        
        %% 计算四个力
        
        U = zeros(Nt, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
        % ---------------------------------------------- 机车牵引力
        tmp_Fl = GetLocomotiveF_multi(X(carType, :)*3.6, MotorNotch, TBcl);
%         tmp_Fl = tmp_Fl.*(1+rand(size(tmp_Fl))*0.02-0.01);
        U(carType, :) = tmp_Fl * 1e3;
        
        % ---------------------------------------------- 空气制动力
        
        % ---------------------------------------------- 附加阻力阻力
        pTrains = LenTrains + X(Nt+1:end, :);         % -------- 计算每个车子的位置
        [addForce, rempTrains] = GetAdditionalF(mTrainGroup, pTrains, rampList);
        U = U + addForce ;                           %
        
        % ---------------------------------------------- 基本阻力
        basicForce = GetBasicF(mTrainGroup, X(1:Nt, :), C0, Ca);
        basicForce(X(1:Nt, :) <= 0) = 0;
        U = U + basicForce;
        
        U = U.*(1 + noise_force(total_step_count, :));
        
        %% 更新状态方程
        
        X = Ad*X + Bd*U;
        
        %% 记录状态变量
        
        F_c_ = KK.*diff(X(Nt+1:Nt*2, :)) + DD.*diff(X(1:Nt, :)); 
        F_c(itr, :) = max(abs(F_c_)); 
        F_l(itr, :) = sum(tmp_Fl.^2); 
        
    end
    %%   处理所有回报，并判断是否有终止发生
    
    countStepNe = countStepNe + 1; 
    for i = 1:Ne
        phase_buffer(countStepNe(i), 1:2, i) = X([Nt+1 1], i);
    end
    
    R_tmp(1, :) = sum(F_c);
    R_tmp(2, :) = sum(F_l);   % Rsum = Rsum + mean(R_tmp, 2);
    
%     R = -reward_weigh*R_tmp - detaT*2 ;
    R = - detaT/8 ;
%     R = -reward_weigh*R_tmp - 1 ;
    
    Lspeed = W_obj.obtain_speed_limit(X([1+Nt Nt*2], :), speed_ChangePoint, hHpeed_Limits);
%     bo_tmp = X([1+Nt Nt*2], :) < speed_ChangePoint;
%     Lspeed(bo_tmp) = hHpeed_Limits(1);
%     Lspeed(~bo_tmp) = hHpeed_Limits(2);
    
    speed_r = higSpeed_r(X(1, :) - max(Lspeed)) + lowSpeed_r(X(1, :) - lowSpeedLim);
    R = R + speed_r;
    
    R = R.*(1 + noise_sensor(total_step_count, :));
    
    bool_terminate = s_tmp >= TotalLength;
    R(bool_terminate) = -(R(bool_terminate));  % 转化为正数回报以标识终点回报
    
%     if any(R < -5)
%         pause(0.01);
%     end
    
    v_tmp_ = v_tmp;
    s_tmp_ = s_tmp;
    
    v_tmp = X(1, :)*3.6;
    s_tmp = X(Nt+1, :);
    
    for i = 1:Ne   % 记录，并更新Q值  
        Qloc(i, :) = [s_tmp_(i), v_tmp_(i), MotorNotch(:, i)', s_tmp(i), v_tmp(i), R(i)]; 
    end
    % 先临时更新一次，数据至少使用一次
%     W_obj = W_obj.update_ActionFunction(W_obj, W_obj_copy, Qloc);
%     W_obj = W_obj.update_ActionFunction(W_obj, W_obj_copy, Qloc);
    
    %% 如果有终止发生，生成新的随机状态，开始探索
    
    bool_restart = bool_terminate | countStepNe >= maxStep  | X(1, :) - lowSpeedLim < -2  | X(1, :) - max(Lspeed) > 5;
    if any(bool_restart)   % 如果有终止发生, 生成随机种子，重新开始
        inds = find(bool_restart);
        tmp_ind = length(inds); 
        initial_position = rand(1, tmp_ind)*initial_posRange;
        bo = rand(size(initial_position)) > epsilon2;
        initial_position(bo) = initial_position(bo)/10;
%         if rand < 0.01  % 有 1% 概率从0开始
%             [~, min_] = min(initial_position);
%             initial_position(min_) = 0;
%         end
        X((1:Nt)+Nt, inds) = ones(Nt, 1)*initial_position;     % 初位置
        
        % Lspeed_ = zeros(2, tmp_ind);
        Lspeed_ = W_obj.obtain_speed_limit(X([1+Nt Nt*2], inds), speed_ChangePoint, hHpeed_Limits);
        
        %         bo_tmp = X([1+Nt Nt*2], inds) < speed_ChangePoint;
        %         Lspeed_(bo_tmp) = hHpeed_Limits(1);
        %         Lspeed_(~bo_tmp) = hHpeed_Limits(2);
        
        initial_speed = (rand(1, tmp_ind).*(max(Lspeed_) - lowSpeedLim)+lowSpeedLim);%*3.6;
        
%         if rand < epsilon2
%             initial_speed = (rand(1, tmp_ind).*(max(Lspeed_) - lowSpeedLim)+lowSpeedLim);%*3.6;
%         else
%             initial_speed = zeros(1, tmp_ind);%*3.6;
%             for i = 1:length(inds)
%                 s_position = initial_position(i);
%                 Val = W_obj.get_state_value_by_speed(W_obj, s_position, v_speeds);
%                 [~, ind_] = max(Val);
%                 v_ = v_speeds(ind_);
%                 v_(v_ > max(Lspeed_(:, i))) = max(Lspeed_(:, i));
%                 initial_speed(i) = v_;
%             end
%         end
        
        X(1:Nt, inds) = ones(Nt, 1)*initial_speed;     % 初速度
        
        initial_notch = round((initial_speed*3.6+randn*5)/10);
        initial_notch(initial_notch > 12) = 12;
        initial_notch(initial_notch < -12) = -12;
        MotorNotch(:, inds) = zeros(nl, tmp_ind) + initial_notch;     % 每个机车有独立的级位
        
        countStepNe(inds) = 1;   % 重置为1 
        phase_buffer(:, :, inds) = NaN;
        for i = 1:length(inds)
            phase_buffer(countStepNe(inds(i)), 1:2, inds(i)) = X([Nt+1 1], inds(i));   % 起始状态
        end
        
        e_count = e_count + tmp_ind;
    end
    
    %% 选择下一步的操作 % epsilon-greedy
    
    v_tmp = X(1, :)*3.6;
    s_tmp = X(Nt+1, :);
    actions = W_obj.choose_next_action(W_obj, [s_tmp; v_tmp], MotorNotch, epsilon);
    MotorNotch = actions;
    
    %% 滚动追加最近的 replay_buffer_size 个数据组
    
    replay_point = replay_point + 1;
    if replay_point > replay_buffer_size
        replay_point = 1;
    end
    
    QLOC((replay_point-1)*replay_batch+1:replay_point*replay_batch, :) = Qloc;
    
    if replay_flag < replay_point*replay_batch
        replay_flag = replay_point*replay_batch;
    end
    
    % experience replay, 每一次做决定都 replay 一次
    uniform_ind = ceil(rand(replay_batch*30, 1)*replay_flag);
    W_obj = W_obj.update_ActionFunction(W_obj, W_obj_copy, QLOC(uniform_ind, :));
    
end

% close(hwait)
tooc = toc;
toc

% save AllData_20180911.mat
%% 
% % figure(1);clf
% % mesh(U_recorder')
% % 
% % figure(2);clf
% % plot(V_recorder')
% 
% figure(601);clf
% % semilogy(Rs_record4.*(ones(size(Rs_record4,1), 1)*reward_weigh))
% plot(Rs_record4.*(ones(size(Rs_record4,1), 1)*reward_weigh))
% legend('车钩力','机车牵引力积分', '拖车力积分' ,'速度偏差');
% 
% col = [1 202];
% figure(501);clf
% axes(1) = subplot(411);
% hold on;
% plot(U_recorder(col(1), :)/1e3, 'linewidth', 2);
% plot(U_recorder(col(2), :)/1e3, 'linewidth', 2);
% ylabel('牵引力');
% axes(2) = subplot(412);
% hold on;
% plot(V_recorder(col(1), :)*3.6, 'linewidth', 2);
% plot(V_recorder(col(2), :)*3.6, 'linewidth', 2);
% ylabel('速度 km/h');
% set(gca,'ylim', [20 90]);
% axes(3) = subplot(413);
% hold on;
% plot(loc_Notchs1, 'linewidth', 2);
% plot(loc_Notchs2, 'linewidth', 2);
% ylabel('级位 km/h');
% set(gca,'ylim', [-12 12]);
% axes(4) = subplot(414);
% hold on;
% plot(RP_recorder(col(1), :), 'linewidth', 2);
% plot(RP_recorder(col(2), :), 'linewidth', 2);
% ylabel('坡度');
% % linkaxes(axes, 'x');
% 
% set(gca,'xlim', [0 length(tvec)]);
% 
% % % col = 2;
% % figure(102);clf
% % axes(1) = subplot(411);
% % plot(U_recorder(2, :)/1e3, 'linewidth', 2);
% % ylabel('牵引力');
% % axes(2) = subplot(412);
% % plot(V_recorder(1, :)*3.6, 'linewidth', 2);
% % ylabel('速度 km/h');
% % axes(3) = subplot(413);
% % plot(loc_Notchs1, 'linewidth', 2);
% % ylabel('级位 km/h');
% % axes(4) = subplot(414);
% % plot(RP_recorder(1, :), 'linewidth', 2);
% % ylabel('坡度');
% % linkaxes(axes, 'x');
% 
% 
% figure(302);clf
% hold on;
% plot(R_Recorder, 'c' ,'DisplayName','R_Recorder')
% plot(mean(R_Recorder, 2), 'k' ,'linewidth', 2)
% 
% 
% % figure(303);clf
% % plot(X(1:Nt, :)-Rspeed)
% 
% % figure(303);clf
% % plot(L_recorder')
% 
% % save AllResults20180713.mat
% % save alldata_20180813.mat
% 
% % save alldata_20180813b.mat
% 
% 
% %% initialize figure
% 
% current_ind_sv = coding_index_transform(W_obj, [1700; 70]);
% vtmp = reshape(sum(W_obj.W(:, current_ind_sv(:, 1)), 2), nNotch, nNotch);
% 
% figure(2);clf
% hmesh = mesh(vtmp);
% set(gca, 'zdir', 'reverse');
% % set(gca, 'zlim', [-120 10]);
% title(1);
% view(-159, 58)
% 
% figure(601);clf
% % semilogy(Rs_record4.*(ones(size(Rs_record4,1), 1)*reward_weigh))
% hRs = plot(Rs_record4.*(ones(size(Rs_record4,1), 1)*reward_weigh));
% legend('车钩力','机车牵引力积分', '拖车力积分' ,'速度偏差');
% set(gca, 'xlim', [0 episode_loops]);
% 
% nei = 1;
% figure(101);clf
% hold on;
% hnei(1) = plot(initial_notch(1, :), initial_notch(2, :), 'ks', 'linewidth', 2);
% hnei(2) = plot(loc_Notchs1(:), loc_Notchs2(:), 'co');
% hnei(3) = plot(loc_Notchs1(:, nei), loc_Notchs2(:, nei), 'r.-');
% set(gca, 'xlim', [-1 1]*12, 'ylim', [-1 1]*12);
% box on;
% grid on;
% 
% 





















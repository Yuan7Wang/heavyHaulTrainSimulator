% testHeavyHaulTrain_OptimizeRL_DNN.m

% clear; clc

flag_up_or_down_ramp = 1;    % 0 表示上山，1 表示下山
flag_speed_limit = 1;        % 0 表示 limit 1，1 表示 limit 2
% function testHeavyHaulTrain_OptimizeRL_DNN3(flag_up_or_down_ramp, flag_speed_limit)
%% 

total_step_max = 1e4;
maxStep = 5e2;
wobj_copy_num = 20;

% % ------------- weight 多个目标之间的相互权重大小
reward_weigh = [2/1e6 5/1e6*1.5]*1e0;   % 车钩力， 总能耗

%%

dt = 0.1;
detaT = 10;
nT = round(detaT/dt);

TotalLength = 40e3;
initial_posRange = TotalLength*0.8;

if flag_speed_limit
%     hHpeed_Limits = [70 85 75 95]/3.6;
%     speed_ChangePoint = TotalLength*[2/10 5/10 7/10];
    hHpeed_Limits = [80, 70, 90, 65]/3.6;
    speed_ChangePoint = TotalLength*[0.3, 0.4, 0.8];
else
    hHpeed_Limits = [85 65 90 70]/3.6;
    speed_ChangePoint = TotalLength*[2/10 4/10 8/10];
end

lowSpeedLim = 10/3.6;
nSlim = length(speed_ChangePoint);

%% initialization of Vehicles 

Nt = 200+2;
Ne = 32;

X = zeros(Nt*2, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
U = zeros(Nt, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
countStepNe = zeros(1, Ne); % 记录当前 episode 执行了多少步了
bool_terminate = zeros(1, Ne); 
bool_restart = zeros(1, Ne);

% maxStep = 2e2; % 最大步长，超过该步长程序终止
phase_buffer = zeros(maxStep, 9, Ne)*NaN; % 记录轨迹
hm_counter = 0;
episode_history = zeros(maxStep, 9, round(total_step_max/10))*NaN; % 记录历史轨迹

% X(1:Nt, :) = initial_speed;     % 初速度

carType = zeros(Nt, 1) == 1;  % 车辆类型，机车还是拖车
carType([1 end]) = true;        % 一头一尾两个机车

nl = sum(carType);
Nw = sum(~carType);

power_recycle_discout = 0.8;
power_consumption_discout = 0.9;

[Ad, Bd, TBcl, airTimeDelay, Abr, LenTrain, mTrainGroup, C0, Ca, KK, DD] = ...
    initialize_locomotive_char_V2(dt, carType, Ne);

% Bd = -Bd * 1e3;

LenTrains = LenTrain'*ones(1, Ne);
pos_train = LenTrains([1 end])';

% nl = 2; % 两个机车
nNegNotch = 12;
nPosNotch = 12; % Notch 个数
% nNotch = nNegNotch + nPosNotch + 1;

%% 核心学习参数与函数

detaS = 4000; % unit m
detaV = 40;  % unit km/h
maxV = [0 120];  % unit km/h
maxS = [0 TotalLength]; % unit m
nTiling = 30;  % number of tiling 
% nAction = (nPosNotch + nNegNotch + 1)^nl;
nHiddens = [256  ]; %#ok<NBRAK>
% nHiddens = [256 300 200];  
W_obj = get_Wobj_4HeavyHaul_DNN2(nHiddens, maxS, maxV, detaS, detaV, nTiling, nl, nPosNotch, nNegNotch);


%% recorder for training performance display

s = W_obj.rangS(1): 2000: W_obj.rangS(2);
v = [40  60];
[Ms, Mv] = meshgrid(s, v);
recorder_sv_coding = W_obj.coding_index_transform(W_obj, [Ms(:), Mv(:)]');
recorder_Np = [4 6;4 6];
recorder_N_ind = W_obj.coding_action_transform(W_obj, recorder_Np);
N_len_recorder = numel(Ms);

value_recerder = zeros(N_len_recorder, 1);
Value_history = zeros(N_len_recorder, total_step_max)*NaN;

%% Load the infrastructure

% rampList = [-10000 0; 0 0.01; 1500 -0.01; 3000 0; 10000 0; 20000 0; 50000 0];
% rampList = [-10000 0; 0 0.01; 1500 -0.01; 3000 0; 10000 0; 20000 0; 50000 0];

% rampList_up = [-10000 0;  
%     0000 0.001
%     5000 0.002; 
%     10000 -0.001; 
%     15000 0.001;
%     21000 -0.002;
%     24000 0.002;
%     32000 0.001; 
%     38000 0; 
%     40000 0; 
%     50000 0; 
%     100000 0];
rampList_up = [-10000 0;  
    0000 0.000
    2000 0.001; 
    10000 -0.001; 
    15000 0.001;
    23000 -0.0005;
    32000 0.0005;
    38000 -0.001; 
    38000 0; 
    40000 0; 
    50000 0; 
    100000 0];
rampList_up(:, 2) = rampList_up(:, 2) * 4;

rampList_down = [-10000 0; 
    2000 0;
    5000 -0.001; 
    12000 0.001; 
    15000 -0.001;
    25000 0.001; 
    28000 -0.001; 
    37000 0; 
    40000 0; 
    50000 0; 
    50000 0; 
    100000 0];
rampList_down(:, 2) = rampList_down(:, 2) * 4;
% 
if flag_up_or_down_ramp
    rampList = rampList_down;
else
    rampList = rampList_up;
end
% rampList = rampList_down;

plot_rampList = zeros(size(rampList, 1), 2);
dxr = diff(rampList(:, 1));
plot_rampList(2:end, 1) = rampList(2:end, 1);
plot_rampList(2:end, 2) = cumsum(dxr.*rampList(1:end-1, 2));
min_ramp = min(plot_rampList(:, 2));
max_ramp = max(plot_rampList(:, 2));
plot_rampList = [plot_rampList(1, 1) min_ramp; plot_rampList];
plot_rampList(end, :) = [plot_rampList(end, 1) min_ramp];

figure(6);
hfill = fill(plot_rampList(:, 1)/1e3, plot_rampList(:, 2), 'g');
set(hfill,'edgealpha', 0.1,'facealpha',0.3) 
set(gca, 'ylim', [min_ramp max_ramp+(max_ramp - min_ramp)*1], 'xlim', [0 TotalLength]/1e3);

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
% alpha = 5e-3/nTiling/5;
alpha = 2e-5;

W_obj.gamma = gamma;
W_obj.alpha = alpha;

epsilon = 0.2;
epsilon_min = 0.05;
epsilon_deta = 4e-4;

epsilon2 = 0.8;
epsilon2_min = 0.2;
epsilon2_deta = 2e-4;

% epsilons3 = [linspace(10e-1, 10e-1, total_step_max) 0];


%% Target Network Setup

W_obj_copy = W_obj;
wobj_copy_count = 0;

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

Energy_consumption =  zeros(round(detaT/dt), Ne);
F_w =  zeros(round(detaT/dt), Ne);
F_c =  zeros(round(detaT/dt), Ne);
errV =  zeros(round(detaT/dt), Ne);

F_c_max_ = zeros(size(F_c));
F_c_min_ = zeros(size(F_c));

R_Recorder = zeros(maxStep, Ne);

%% experience replay 

replay_buffer_size = 500;
% replay_buffer_size = episode_loops;
replay_point = 0;    % 数据记录指针
replay_flag = 0;     % 标识记录总数

% in_epi_number = round(T/detaT);
in_epi_number = 1;
replay_batch = 2^11; 
Rs_record = zeros(in_epi_number, Ne);    % 每一步的回报 

nQloc = 2 + nl + 2 + 1;
Qloc = zeros(Ne, nQloc);    % 每一步的 state-action
local_point = 0;
QLOC = zeros(Ne * replay_buffer_size, nQloc)*NaN;    % 每一步的 state-action


%% 低速、超速惩罚

lowSpeedFlag = zeros(1, Ne) == 1;
highSpeedFlag = zeros(1, Ne) == 1;

a = -0.1;
b = -100;
large_neg_reward = -1e1;    % 一个很大的负reward
higSpeed_r = @(dv) (a*(abs(dv)+5).^3 + b).*(dv > 0);
lowSpeed_r = @(dv) (a*(abs(dv)+5).^3 + b).*(dv < 0);
% higSpeed_r = @(dv) large_neg_reward.*(dv > 0);
% lowSpeed_r = @(dv) large_neg_reward.*(dv < 0);

% figure; xx = linspace(-10,10, 100)'/3.6; plot(xx*3.6, higSpeed_r(xx))
% figure; xx = linspace(-10,10, 100)'/3.6; plot(xx*3.6, lowSpeed_r(xx))


%% initialization

Rspeed = ones(2, Ne)*hHpeed_Limits(1);    % 记录车头车尾的速度

initial_position = rand(1, Ne)*initial_posRange;
if rand < 0.01  % 有 1% 概率从0开始
    [~, min_] = min(initial_position);
    initial_position(min_) = 0;
end
X((1:Nt)+Nt, :) = ones(Nt, 1)*initial_position;     % 初位置

% hHpeed_Limits = [70/3.6 60/3.6 ];
% lowSpeedLim = 20/3.6;

% Lspeed = W_obj.obtain_speed_limit(X([1+Nt Nt*2], :), speed_ChangePoint, hHpeed_Limits);
Lspeed = W_obj.obtain_speed_limit(X([1+Nt Nt*2], :) + pos_train, speed_ChangePoint, hHpeed_Limits);

initial_speed = (rand(1, Ne).*(min(Lspeed) - lowSpeedLim)+lowSpeedLim);%*3.6;
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
    phase_buffer(countStepNe(i), 1:2, i) = X([Nt+1 1], i);   % 起始状态 ---- 第一个机车
    phase_buffer(countStepNe(i), 3:4, i) = X([Nt*2 Nt], i);   % 起始状态  ---- 第二个机车
    phase_buffer(countStepNe(i), 5:6, i) = MotorNotch(:, i);   % 起始状态
    phase_buffer(countStepNe(i), 7:9, i) = 0;   % 起始状态
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
% plot([0  TotalLength], lowSpeedLim([1 1])*3.6, 'k', 'linewidth', 3);
set(gca, 'xlim', [0 TotalLength], 'ylim', maxV)


xlabel('Milage (m)');
ylabel('Speed (km/h)');
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
yyaxis right
hfill = fill(plot_rampList(:, 1), plot_rampList(:, 2), 'g');
set(hfill,'edgealpha', 0.1,'facealpha',0.3) 
set(gca, 'ylim', [min_ramp max_ramp+(max_ramp - min_ramp)*1]);
ylabel('Altitude (m)')
% set(gca, 'ylim', [min(rampList(:, 3)) max(rampList(:, 3))]*3);
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
box on;

%% 
% temp_contour_drawing_DNN3()

%% 

Recorder_r = zeros(total_step_max, 3) * NaN;


%% 主循环

total_step_count = 0;

disp_cycle = [50 400 5000];

tic
e_count = 1;
for total_step_count = 1:total_step_max 
    %% 更新绘图显示
    if rem(total_step_count, disp_cycle(1)) == 0  % update_plot
        time_consumed = toc;
        disp(['exploring step: ' num2str(total_step_count) ', time comsumed: ' num2str(time_consumed)]);
        for i = 1:Ne
            set(h_traj(2, i), 'xdata', phase_buffer(:, 1, i), 'ydata', phase_buffer(:, 2, i)*3.6);
        end
        drawnow;
        pause(1e-10);
    end
    
    if rem(total_step_count, disp_cycle(2)) == 0
        temp_sample_drawing_DNN3()
    end
    
    if rem(total_step_count, disp_cycle(3)) == 0
        temp_contour_drawing_DNN3()
    end
    
    %% 增量更新
    epsilon  = epsilon - epsilon_deta;
    epsilon(epsilon < epsilon_min) = epsilon_min;
    epsilon2  = epsilon2 - epsilon2_deta;
    epsilon2(epsilon2 < epsilon2_min) = epsilon2_min;
%     epsilon2  = epsilons2(total_step_count);
%     initial_posRange = TotalLength*epsilons3(total_step_count); 
    
    %% 
    for itr = 1:N_len_recorder
        value_recerder(itr) = W_obj.max_Q_value_of_State(W_obj, recorder_sv_coding(:, itr), recorder_N_ind);
    end
    Value_history(:, total_step_count) = value_recerder;
    

    %%
    wobj_copy_count = wobj_copy_count + 1;
    if wobj_copy_count == wobj_copy_num  
        W_obj_copy = W_obj;      % 克隆一个当前值，并用于选择 aciton
        wobj_copy_count = 0;
    end
    
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
        [addForce, ~] = GetAdditionalF(mTrainGroup, pTrains, rampList);
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
        F_c_max_(itr, :) = max(F_c_);
        F_c_min_(itr, :) = min(F_c_);
        F_c(itr, :) = max(abs(F_c_)); 
        
        tmp_F2 = tmp_Fl;
        tmp_Fl(tmp_Fl < 0) = 0;      % 牵引力做功消耗能量
        tmp_F2(tmp_F2 > 0) = 0;      % 制动力回馈能量
        Energy_consumption(itr, :) = (sum(tmp_Fl.*X([1 Nt], :))/power_consumption_discout ...
            + power_recycle_discout*sum(tmp_F2.*X([1 Nt], :)))*dt; 
    end
    %%   处理所有回报，并判断是否有终止发生
    
    F_c_max = max(F_c_max_);
    F_c_min = min(F_c_min_);
    R_tmp(1, :) = max(F_c);
    R_tmp(2, :) = sum(Energy_consumption);   % Rsum = Rsum + mean(R_tmp, 2);
    
    countStepNe = countStepNe + 1; 
    for i = 1:Ne
        phase_buffer(countStepNe(i), 1:2, i) = X([Nt+1 1], i);   % 起始状态 ---- 第一个机车
        phase_buffer(countStepNe(i), 3:4, i) = X([Nt*2 Nt], i);   % 起始状态  ---- 第二个机车 
        phase_buffer(countStepNe(i), 5:6, i) = MotorNotch(:, i);  % 级位
        phase_buffer(countStepNe(i), 7:9, i) = [F_c_max(i) F_c_min(i) R_tmp(2, i)];   % 起始状态
    end
    
    V_observation = X(1, :).*(1 + noise_sensor(total_step_count, :));
    R = (V_observation-33)*0.3;
    R(R > 0) = 0;
    
    % ------------- 分别记录各项 reword 的值
    Recorder_r(total_step_count, :) = [mean(R), ...
        reward_weigh(1)*mean(R_tmp(1, :)), reward_weigh(2)*mean(R_tmp(2, :))];
    
    R = R - reward_weigh*R_tmp;
    
    
    Lspeed = W_obj.obtain_speed_limit(X([1+Nt Nt*2], :) + pos_train, speed_ChangePoint, hHpeed_Limits);
    
    speed_r = higSpeed_r(V_observation - min(Lspeed));% + lowSpeed_r(V_observation - lowSpeedLim);
    R = R + speed_r;
    
    R = R.*(1 + noise_sensor(total_step_count, :));
    
    bool_terminate = s_tmp >= TotalLength;
    R(bool_terminate) = -R(bool_terminate);  % 转化为正数回报以标识终点回报
    
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
    W_obj = W_obj.update_ActionFunction(W_obj, W_obj_copy, Qloc);
    
    %% 如果有终止发生，生成新的随机状态，开始探索
    
    bool_restart = bool_terminate | countStepNe >= maxStep  ...
        | V_observation - max(Lspeed) > 5 | V_observation - lowSpeedLim < -1  ;
    if any(bool_restart)   % 如果有终止发生, 生成随机种子，重新开始
        inds = find(bool_restart);
        tmp_ind = length(inds); 
        initial_position = rand(1, tmp_ind)*initial_posRange;
        bo = rand(size(initial_position)) > epsilon2;
        initial_position(bo) = 0;
%         if rand < 0.01  % 有 1% 概率从0开始
%             [~, min_] = min(initial_position);
%             initial_position(min_) = 0;
%         end
        X((1:Nt)+Nt, inds) = ones(Nt, 1)*initial_position;     % 初位置
        
        % Lspeed_ = zeros(2, tmp_ind);
        Lspeed_ = W_obj.obtain_speed_limit(X([1+Nt Nt*2], inds) + pos_train, speed_ChangePoint, hHpeed_Limits);
        initial_speed = (rand(1, tmp_ind).*(min(Lspeed_) - lowSpeedLim)+lowSpeedLim);%*3.6;
        X(1:Nt, inds) = ones(Nt, 1)*initial_speed;     % 初速度
        
        initial_notch = round((initial_speed*3.6+randn*5)/10);
        initial_notch(initial_notch > 12) = 12;
        initial_notch(initial_notch < -12) = -12;
        MotorNotch(:, inds) = zeros(nl, tmp_ind) + initial_notch;     % 每个机车有独立的级位
        
        % 保存结束的 episode
        episode_history(:, :, hm_counter+1:hm_counter+tmp_ind) = phase_buffer(:, :, inds);
        hm_counter = hm_counter + tmp_ind;
        
        countStepNe(inds) = 1;   % 重置为1 
        phase_buffer(:, :, inds) = NaN;
        for i = inds
            phase_buffer(countStepNe(i), 1:2, i) = X([Nt+1 1], i);   % 起始状态 ---- 第一个机车
            phase_buffer(countStepNe(i), 3:4, i) = X([Nt*2 Nt], i);   % 起始状态  ---- 第二个机车 
            phase_buffer(countStepNe(i), 5:6, i) = MotorNotch(:, i);   % 级位
            phase_buffer(countStepNe(i), 7:9, i) = 0;   % 起始状态
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
    
    QLOC((replay_point-1)*Ne+1:replay_point*Ne, :) = Qloc;
    
    if replay_flag < replay_point*Ne
        replay_flag = replay_point*Ne;
    end
    
    % experience replay, 每一次做决定都 replay 一次
    uniform_ind = ceil(rand(Ne*30, 1)*replay_flag);
    W_obj = W_obj.update_ActionFunction(W_obj, W_obj_copy, QLOC(uniform_ind, :));
    
end

% close(hwait)
tooc = toc;
toc

%% 

A_epis = episode_history(:, :, 1:hm_counter);
len_epi = reshape((max(A_epis(:, 1, :)))-(min(A_epis(:, 1, :))), [hm_counter, 1]);
[~, ind_] = max(len_epi);

Atmp = episode_history(:, :, ind_); 

phase_ = Atmp(:, 1:4);
notch_ = Atmp(:, 5:6);
force_ = Atmp(:, 7:8);
energy_ = Atmp(:, 9);

phase_(:, [1 3]) = (phase_(:, [1 3]) + pos_train')/1e3;
force_ = -force_/1e3;
energy_ = energy_/1e3;
xx_ = mean(phase_(:, [1 3]), 2);

figure(501);clf
axes(1) = subplot(411);
hold on;
plot(x_/1e3, y_, 'k', 'linewidth', 3);
% plot([0  TotalLength], lowSpeedLim([1 1])*3.6, 'k', 'linewidth', 3);
set(gca, 'xlim', [0 TotalLength], 'ylim', maxV)
plot(phase_(:, 1), phase_(:, 2)*3.6, 'b-', 'linewidth', 2)
plot(phase_(:, 3), phase_(:, 4)*3.6, 'r-', 'linewidth', 2)
ylabel('Speed (km/h)');
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
yyaxis right
hfill = fill(plot_rampList(:, 1)/1e3, plot_rampList(:, 2), 'g');
set(hfill,'edgealpha', 0.1,'facealpha',0.3) 
set(gca, 'ylim', [min_ramp max_ramp+(max_ramp - min_ramp)*1]);
ylabel('Altitude (m)')
% set(gca, 'ylim', [min(rampList(:, 3)) max(rampList(:, 3))]*3);
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
box on;

axes(2) = subplot(412);
hold on;
stairs(xx_, notch_(:, 1), 'b-', 'linewidth', 2)
stairs(xx_, notch_(:, 2), 'r-', 'linewidth', 2)
set(gca, 'ylim', [-15 15])
hleg = legend('Locmotive #1','Locmotive #2');
set(hleg, 'Location', 'Southeast')
box on;
ylabel('Notch');
set(gca, 'fontsize', 14, 'fontname', 'Cambria')

axes(3) = subplot(413);
hold on;
stairs(xx_, force_(:, 1), 'b-', 'linewidth', 2)
stairs(xx_, force_(:, 2), 'r-', 'linewidth', 2)
hleg = legend('Maxmum','Minimum');
set(hleg, 'Location', 'Southeast')
box on;
ylabel('Coupler Force (KN)');
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
axes(4) = subplot(414); 
hold on;
plot(x_([1 end])/1e3, [0 0], 'k', 'linewidth', 2);
plot(xx_, energy_, '-', 'linewidth', 2) 
box on;
xlabel('Milage (km)');
ylabel('Energy consumption (KJ)');
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
linkaxes(axes, 'x');
set(gca, 'xlim', [1 40])


%% 





















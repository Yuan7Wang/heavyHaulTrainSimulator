
clear; clc

%% 
episode_loops = 5e2; 

nl = 2; % 两个机车
nNegNotch = 0;
nPosNotch = 12; % Notch 个数
nNotch = nNegNotch + nPosNotch + 1;

large_neg_reward = -1e10;    % 一个很大的负reward

%%
dt = 0.1;
T = 30;
detaT = 2;
tvec = dt:dt:T;
nT = length(tvec);

reference_speeds = [70/3.6 70/3.6];

initial_speed = 60/3.6;
initial_notch = 7;

Nt = 200+2;
Ne = 10;

X = zeros(Nt*2, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
U = zeros(Nt, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置


X(1:Nt, :) = initial_speed;     % 初速度

carType = zeros(Nt, 1) == 1;  % 车辆类型，机车还是拖车
carType([1 end]) = true;        % 一头一尾两个机车

Nl = sum(carType);
Nw = sum(~carType);

% MotorNotch = zeros(Nl, Ne) + round(rand(Nl, Ne)*24-12);     % 每个机车有独立的级位 

% TBCL_force

%% 

detaS = 400; % unit m
detaV = 40;  % unit km/h
maxS = ceil(100/3.6*T/200)*200; % unit m
maxV = 100;  % unit km/h
nTiling = 50;  % number of tiling 
W_obj = get_Wobj_linear_Fit(maxS, maxV, detaS, detaV, nTiling, nl, nPosNotch, nNegNotch);
% nAction = (nPosNotch + nNegNotch + 1)^nl;


% W_obj = update_ActionFunction_linear(W_obj, Qloc, current_r, alpha);

%% initialization
% 
[Ad, Bd, TBcl, airTimeDelay, Abr, LenTrain, mTrainGroup, C0, Ca, KK, DD] = ...
    initialize_locomotive_char_V2(dt, carType, Ne);

% Bd = -Bd * 1e3;

LenTrains = LenTrain'*ones(1, Ne);


%% Load the infrastructure
% load('RampPositionList');  % 1.Format:start(m) end(m) gradient, 2.Range: [-10000m,100000m]
% % load('RampPoint'); % 1.Format:position(m) Height(m),  2.Range:[9000m,37972m]
% 
% rampList = RampPositionList(:, [1 3]);
% 
% rampList(:, 2) = -rampList(:, 2);

rampList = [-10000 0; 0 0.01; 1500 -0.01; 3000 0; 10000 0; 20000 0; 30000 0];


%% Air break

air_timer = zeros(Nw, Ne);        % 空气制动计时器, 当前的
air_timer_ = zeros(Nw, Ne);        % 空气制动计时器， 上一步的
% AirBrkNotch = round(-rand(1, Ne)*2);       % 空气制动，-2, -1, 0;
AirBrkNotch = round(-rand(1, Ne)*0);       % 空气制动，-2, -1, 0;
AirBrkNotch_ = zeros(1, Ne);      % 空气制动之前一次控制量，1表示制动，0表示缓解

F_target = zeros(Nw, Ne);
currentF = zeros(Nw, Ne);

%% Q matrix

% nnum = nNotch;

Speed_Change_Point = 1500;

ActionSet = get_ActionSet2();
n_act = size(ActionSet, 1);
% ActionSet = zeros(27,3);
% NumOfActions = size(ActionSet,1);

% action = ones(1, Ne);
actions = zeros(2, Ne);
% action_ = action;

MotorNotch = actions;

% v_tmp = ceil(X(1, :)*3.6)+1;
% s_tmp = ceil(X(Nt+1, :)/dl)+1;

% figure; mesh(Q(:, : , 5))

%% Reinforcement Learning paramerters

gamma = 1;
alpha = 1e-2;

W_obj_copy = W_obj;
wobj_copy_num = 1;
wobj_copy_count = 0;

epsilons = [linspace(4e-1, 1e-2, episode_loops-1) 0];

%% Recorder

X_Recorder = zeros(length(tvec), Ne);

F_l =  zeros(round(detaT/dt), Ne);
F_w =  zeros(round(detaT/dt), Ne);
F_c =  zeros(round(detaT/dt), Ne);
errV =  zeros(round(detaT/dt), Ne);

locF = zeros(length(tvec), Ne);
airF = zeros(length(tvec), Ne);
addF = zeros(length(tvec), Ne);
basF = zeros(length(tvec), Ne);

    
in_epi_number = round(T/detaT);                         %

loc_Notchs1 =  zeros(in_epi_number, Ne)*NaN;
loc_Notchs2 =  zeros(in_epi_number, Ne)*NaN;
reward_Record = zeros(in_epi_number, episode_loops, 4);

R_Recorder = zeros(episode_loops, Ne);

Repisode = zeros(episode_loops, 4);    % 每一步 episode 的回报

Rs_record4 = zeros(episode_loops, 4)*NaN;    % 每一步的回报

%% experience replay 

replay_buffer_size = 80;
replay_point = 0;    % 数据记录指针
replay_flag = 0;     % 标识记录总数
replay_batch = in_epi_number * Ne;
Rs_record = zeros(in_epi_number, Ne);    % 每一步的回报

nQloc = 2 + nl + 2 + 1;
Qloc = zeros(replay_batch, nQloc);    % 每一步的 state-action
local_point = 0;
QLOC = zeros(replay_batch * replay_buffer_size, nQloc)*NaN;    % 每一步的 state-action

%% 低速、超速保护机制

lowSpeedFlag = zeros(1, Ne) == 1;
highSpeedFlag = zeros(1, Ne) == 1;

%% weight

% reward_weigh = [1/4e7 1/1e7 0 1/1e4*4];
reward_weigh = [1/1e7 1/1e7 0 1/1e4*1]*1e-1;

%% initialize figure

% current_ind_sv = coding_index_transform(W_obj, [320; 0]);
% vtmp = reshape(sum(W_obj.W(:, current_ind_sv(:, 1)), 2), nNotch, nNotch);
% 
% figure(2);clf
% hmesh = mesh(vtmp);
% set(gca, 'zdir', 'reverse');
% set(gca, 'zlim', [-120 10]);
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
% hnei(1) = plot(initial_notch, initial_notch, 'ks', 'linewidth', 2);
% hnei(2) = plot(loc_Notchs1(:), loc_Notchs2(:), 'co');
% hnei(3) = plot(loc_Notchs1(:, nei), loc_Notchs2(:, nei), 'r.-');
% set(gca, 'xlim', [-1 1]*12, 'ylim', [-1 1]*12);
% box on;
% grid on;

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
    
    air_Notchs =  zeros(length(tvec), Ne)*NaN;
    loc_Notchs1 =  zeros(in_epi_number, Ne)*NaN;
    loc_Notchs2 =  zeros(in_epi_number, Ne)*NaN;
%     
    U_recorder = zeros(Nt, length(tvec))*NaN;
    L_recorder = zeros(2, length(tvec))*NaN;
    V_recorder = zeros(Nt, length(tvec))*NaN;
    RP_recorder = zeros(Nt, length(tvec))*NaN;
    
    %% initialization
    
    update_flag = true;
    
%     initial_notch = ones(2, 1)*ceil(rand(1, Ne)*12);
    initial_notch = ceil(rand(Nl, Ne)*10)-1;
    MotorNotch = zeros(Nl, Ne) + initial_notch;     % 每个机车有独立的级位
    
    epsilon  = epsilons(etr);
%     epsilon  = 0;
    
    X = zeros(Nt*2, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
%     X(1:Nt, :) = ones(Nt, 1)*((rand(1, Ne)*80+10)/3.6);     % 初速度
    initial_speed = (rand(1, Ne)*50+40)/3.6;
    X(1:Nt, :) = ones(Nt, 1)*initial_speed;     % 初速度
%     X(1:Nt, :) = ones(Nt, Ne)*initial_speed;     % 初速度
    
    Rspeed = ones(Nt, Ne)*reference_speeds(1);
    
    v_tmp = X(1, :)*3.6;
    s_tmp = X(Nt+1, :);
    
    actions = get_action_by_Wobj_linear_Fit(W_obj_copy, [s_tmp; v_tmp], MotorNotch, epsilon);
    
    MotorNotch = actions;
    
    count = 0;
    count_detaT = 0;
    R_tmp = zeros(4, Ne);
    Rsum = zeros(4, 1);
    % Qloc = Qloc*0;
    local_point = 0;
    for itr = 1:nT
        
        tmpBo = X(Nt+1, :) < Speed_Change_Point;
%         Rspeed(:, tmpBo) = reference_speeds(1);
        Rspeed(:, ~tmpBo) = reference_speeds(2);
        
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
        U = U + basicForce ;
        
        %% 更新状态方程
        
        X = Ad*X + Bd*U;
        
        %% 记录状态变量
        
        count_detaT = count_detaT+1;
        
        X_Recorder(itr, :) = X(1, :);
        
        U_recorder(:, itr) = U(:, 1);
        L_recorder(:, itr) = tmp_Fl(:, 1);
        V_recorder(:, itr) = X(1:Nt, 1);
        RP_recorder(:, itr) = rempTrains(:, 1);
        
        F_c_ = KK.*diff(X(Nt+1:Nt*2, :)) + DD.*diff(X(1:Nt, :));
%         F_c(itr, :) = sum(F_c_.^2);
        F_c(count_detaT, :) = max(abs(F_c_));
        
        F_l(count_detaT, :) = sum(tmp_Fl.^2); 
        
        errV(count_detaT, :) = sum((X(1:Nt, :) - Rspeed).^2);
        
        
        %% -------- 低速保护  % -------- 超速保护
        
        boSpeedFlag = X(1,:)*3.6 <= 2 | X(1,:)*3.6 >= maxV; 
        
        if any(boSpeedFlag)     % 任何一个超速了，中断所有
            update_flag = false;
            disp('Speed limit, break the loop!');
            break;
        end
        
        %% 更新知识矩
        if rem(itr, floor(detaT/dt)) == 0   % 每 10 次 迭代做一次操作
            
            count_detaT = 0;
            
            count = count+1;
            R_tmp(1, :) = sum(F_c);
            R_tmp(2, :) = sum(F_l);
            R_tmp(3, :) = sum(F_w);
            R_tmp(4, :) = sum(errV);
            
            Rsum = Rsum + mean(R_tmp, 2);
            
            R = zeros(1, Ne);
            for atr = 1:4 
                R = R - reward_weigh(atr)*R_tmp(atr, :);
            end
            
            v_tmp_ = v_tmp;
            s_tmp_ = s_tmp;
            
            v_tmp = X(1, :)*3.6;
            s_tmp = X(Nt+1, :);
            
            for i = 1:Ne   % 只记录，不更新Q值，等完成整个 episode 后更新Q矩阵
                local_point = local_point + 1;
                Qloc(local_point, :) = [s_tmp_(i), v_tmp_(i), MotorNotch(:, i)', s_tmp(i), v_tmp(i), R(i)];
            end
            
            Rs_record(count, :) = R;
            
            %% 选择下一步的操作 % epsilon - greedy
            
            actions = get_action_by_Wobj_linear_Fit(W_obj_copy, [s_tmp; v_tmp], MotorNotch, epsilon);
            
            %%
            loc_Notchs1(count, :) = MotorNotch(1, :);
            loc_Notchs2(count, :) = MotorNotch(2, :);
            
            MotorNotch = actions;
            
        end   
    end
    %% 
    if update_flag
        %% 当一次探索完成后，更新Q矩阵
        
        %Qloc_ = reshape(permute(Qloc, [1 3 2]), [], 6);
        W_obj = update_ActionFunction_linear_Fit(W_obj, W_obj_copy, Qloc(end:-1:1, :), alpha);
        
        %% 滚动追加最近的 replay_buffer_size 个数据组
        
        replay_point = replay_point + 1;
        if replay_point > replay_buffer_size
            replay_point = 1;
        end
        
        QLOC((replay_point-1)*replay_batch+1:replay_point*replay_batch, :) = Qloc;
        
        if replay_flag < replay_point*replay_batch
            replay_flag = replay_point*replay_batch;
        end
        
        % experience replay
        uniform_ind = ceil(rand(replay_batch*min([replay_point 30]), 1)*replay_flag);
        W_obj = update_ActionFunction_linear_Fit(W_obj, W_obj_copy, QLOC(uniform_ind, :), alpha); 
        
        %%
        Rs_record4(etr, :) = Rsum';
        R_Recorder(etr, :) = sum(Rs_record);
        
        %     if rem(etr, 100) == 0
        %         %%
        %         set(hnei(1), 'xdata', initial_notch, 'ydata', initial_notch);
        %         set(hnei(2), 'xdata', loc_Notchs1(:), 'ydata', loc_Notchs2(:));
        %         set(hnei(3), 'xdata', loc_Notchs1(:, nei), 'ydata', loc_Notchs2(:, nei));
        %         drawnow;
        %
        %         %%
        %         %         for itr = 40:90
        %         for itr = 70
        %             current_ind_sv = coding_index_transform(W_obj, [320; itr]);
        %             vtmp = reshape(sum(W_obj.W(:, current_ind_sv(:, 1)), 2), nNotch, nNotch);
        %
        %             set(hmesh, 'zdata', vtmp);
        %             title(itr);
        %             drawnow;
        %         end
        %         %%
        %         tmp  = Rs_record4.*(ones(size(Rs_record4,1), 1)*reward_weigh);
        %         for ttr = 1:4
        %             set(hRs(ttr), 'ydata', tmp(:, ttr));
        %         end
        %
        %         drawnow;
        %         pause(0.0001);
        %
        %     end
    end

end

close(hwait)
tooc = toc;
toc
%% 
% figure(1);clf
% mesh(U_recorder')
% 
% figure(2);clf
% plot(V_recorder')

figure(601);clf
% semilogy(Rs_record4.*(ones(size(Rs_record4,1), 1)*reward_weigh))
plot(Rs_record4.*(ones(size(Rs_record4,1), 1)*reward_weigh))
legend('车钩力','机车牵引力积分', '拖车力积分' ,'速度偏差');

col = [1 202];
figure(501);clf
axes(1) = subplot(411);
hold on;
plot(U_recorder(col(1), :)/1e3, 'linewidth', 2);
plot(U_recorder(col(2), :)/1e3, 'linewidth', 2);
ylabel('牵引力');
axes(2) = subplot(412);
hold on;
plot(V_recorder(col(1), :)*3.6, 'linewidth', 2);
plot(V_recorder(col(2), :)*3.6, 'linewidth', 2);
ylabel('速度 km/h');
set(gca,'ylim', [20 90]);
axes(3) = subplot(413);
hold on;
plot(loc_Notchs1, 'linewidth', 2);
plot(loc_Notchs2, 'linewidth', 2);
ylabel('级位 km/h');
set(gca,'ylim', [-12 12]);
axes(4) = subplot(414);
hold on;
plot(RP_recorder(col(1), :), 'linewidth', 2);
plot(RP_recorder(col(2), :), 'linewidth', 2);
ylabel('坡度');
% linkaxes(axes, 'x');

set(gca,'xlim', [0 length(tvec)]);

% % col = 2;
% figure(102);clf
% axes(1) = subplot(411);
% plot(U_recorder(2, :)/1e3, 'linewidth', 2);
% ylabel('牵引力');
% axes(2) = subplot(412);
% plot(V_recorder(1, :)*3.6, 'linewidth', 2);
% ylabel('速度 km/h');
% axes(3) = subplot(413);
% plot(loc_Notchs1, 'linewidth', 2);
% ylabel('级位 km/h');
% axes(4) = subplot(414);
% plot(RP_recorder(1, :), 'linewidth', 2);
% ylabel('坡度');
% linkaxes(axes, 'x');


figure(302);clf
hold on;
plot(R_Recorder, 'c' ,'DisplayName','R_Recorder')
plot(mean(R_Recorder, 2), 'k' ,'linewidth', 2)


% figure(303);clf
% plot(X(1:Nt, :)-Rspeed)

% figure(303);clf
% plot(L_recorder')

% save AllResults20180713.mat

%%
% 
notchs1 = ones(25, 1)*(-12:12);
notchs2 = (-12:12)'*ones(1, 25);
notchs = [notchs1(:) notchs2(:)]';

[MX, MY] = meshgrid(-12:12);

figure(1);clf
hold on;
hmesh = mesh(MX, MY, tmp);
hplot = plot(current_theta(1), current_theta(2), 'ko', 'linewidth', 2);
set(gca, 'zlim', [-1 1]*80, 'xlim', [-12 12], 'ylim', [-12 12]);

for itr = 1:1:200
    
%     itr = 20;
    current_theta = W_obj.getTheta(W_obj, [itr; 4]);
    current_action_value = W_obj.funTheta(W_obj, current_theta, notchs);
    
    tmp = reshape(current_action_value, 25, []);
    
    set(hmesh, 'zdata', tmp);
    set(hplot, 'xdata', current_theta(1), 'ydata', current_theta(2));
    title(itr);
    drawnow;
    pause(0.1);
end

%% initialize figure

% current_ind_sv = coding_index_transform(W_obj, [320; 72]);
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




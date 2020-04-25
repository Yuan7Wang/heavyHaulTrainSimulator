% test_single_Episode6s.m
% Sarsa ---- -- 同步操控

% clear;clc

tic
episode_loops = 1e4; 

large_neg_reward = -1e10;    % 一个很大的负reward

%%
dt = 0.1;
T = 60;
detaT = 3;
tvec = dt:dt:T;
nT = length(tvec);

reference_speeds = [60/3.6 60/3.6];
initial_speed = 50/3.6;

Nt = 200+2;
Ne = 1;

X = zeros(Nt*2, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
U = zeros(Nt, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置


X(1:Nt, :) = initial_speed;     % 初速度

carType = zeros(Nt, 1) == 1;  % 车辆类型，机车还是拖车
carType([1 end]) = true;        % 一头一尾两个机车

Nl = sum(carType);
Nw = sum(~carType);

MotorNotch = zeros(Nl, Ne) + round(rand(Nl, Ne)*20-10);     % 每个机车有独立的级位 

% TBCL_force

%% initialization

[Ad, Bd, TBcl, airTimeDelay, Abr, LenTrain, mTrainGroup, C0, Ca, KK, DD] = ...
    initialize_locomotive_char(dt, carType, Ne);

% Bd = -Bd * 1e3;

LenTrains = LenTrain'*ones(1, Ne);


%% Load the infrastructure
load('RampPositionList');  % 1.Format:start(m) end(m) gradient, 2.Range: [-10000m,100000m]
% load('RampPoint'); % 1.Format:position(m) Height(m),  2.Range:[9000m,37972m]

rampList = RampPositionList(:, [1 3]);

rampList(:, 2) = -rampList(:, 2);

%% Air break

air_timer = zeros(Nw, Ne);        % 空气制动计时器, 当前的
air_timer_ = zeros(Nw, Ne);        % 空气制动计时器， 上一步的
% AirBrkNotch = round(-rand(1, Ne)*2);       % 空气制动，-2, -1, 0;
AirBrkNotch = round(-rand(1, Ne)*0);       % 空气制动，-2, -1, 0;
AirBrkNotch_ = zeros(1, Ne);      % 空气制动之前一次控制量，1表示制动，0表示缓解

F_target = zeros(Nw, Ne);
currentF = zeros(Nw, Ne);

%% Q matrix

nnum = 25;

S = 100/3.6*T;

Speed_Change_Point = 60/3.6*T/2;

dl = 300;   % meter
dv = 1;     % km/h

s_vec = 0:dl:S+dl*1;
v_vec = 0:dv:100+dv*1;

ActionSet = get_ActionSet3();
n_act = size(ActionSet, 1); 

Q = zeros(length(s_vec), length(v_vec), nnum, n_act);
Q(:, :, 1, 2) = NaN;
Q(:, :, end, 3) = NaN;

action = ones(1, Ne);
% action_ = action;

MotorNotch = MotorNotch + ones(2, 1)*ActionSet(action)';

v_tmp = ceil(X(1, :)*3.6)+1;
s_tmp = ceil(X(Nt+1, :)/dl)+1;

% figure; mesh(Q(:, : , 5))

%% Reinforcement Learning paramerters

gamma = 1;
alpha = 0.8;

% epsilons = [linspace(0.5, 1e-2, episode_loops*8/10) zeros(1, episode_loops*2/10)];
% epsilons = [linspace(5e-1, 1e-5, episode_loops*9/10) zeros(1, episode_loops*1/10)];
epsilons = linspace(5e-1, 0, episode_loops*1);

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

reward_Record = zeros(in_epi_number, episode_loops, 4);

R_Recorder = zeros(episode_loops, Ne);

%% 低速、超速保护机制

lowSpeedFlag = zeros(1, Ne) == 1;
highSpeedFlag = zeros(1, Ne) == 1;

%% weight

% reward_weigh = [1/2.4e+14 1/6.64e+5 2e-6 1/1.63e3];
% reward_weigh = [1/4.5e3 0 0 1/1.63e3*1];
% reward_weigh = [1/2e7 1/16e6 0 1/1e4*2];
reward_weigh = [1/2e17 0 0 1/1e4*2];

%% 主循环


hwait = waitbar(0, 'Processing ...');
for etr = 1 : episode_loops
    
    if rem(etr, 10) == 0
        waitbar(etr/episode_loops, hwait, [sprintf('Processing ... %2.1f',etr/episode_loops*100) '%']);
    end
    
    air_Notchs =  zeros(length(tvec), Ne)*NaN;
    loc_Notchs1 =  zeros(length(tvec), Ne)*NaN;
    loc_Notchs2 =  zeros(length(tvec), Ne)*NaN;
%     
    U_recorder = zeros(Nt, length(tvec))*NaN;
    L_recorder = zeros(2, length(tvec))*NaN;
    V_recorder = zeros(Nt, length(tvec))*NaN;
    RP_recorder = zeros(Nt, length(tvec))*NaN;
    
    %% initialization
    
    MotorNotch = zeros(Nl, Ne) + 6;     % 每个机车有独立的级位
    
    epsilon  = epsilons(etr);
%     epsilon  = 0;
    
    X = zeros(Nt*2, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
%     X(1:Nt, :) = ones(Nt, 1)*((rand(1, Ne)*80+10)/3.6);     % 初速度
    X(1:Nt, :) = ones(Nt, Ne)*initial_speed;     % 初速度
    
    Rspeed = ones(Nt, Ne)*reference_speeds(1);
    
    v_tmp = ceil(X(1, :)*3.6)+1;
    s_tmp = ceil(X(Nt+1, :)/dl)+1;
    
    bo_tmp = rand(1, Ne) > epsilon;
    for i = 1:Ne 
        if bo_tmp(i)      % epsilon - greedy
            [~, action(i)] = max(Q(s_tmp(i), v_tmp(i), MotorNotch(1, i)+13, :));
        else
            action_tmp = find(~isnan(Q(s_tmp(i), v_tmp(i), MotorNotch(1, i)+13, :)));
            action(i) = action_tmp(ceil(rand*length(action_tmp)));
        end
    end
    
    if any(MotorNotch + ones(2, 1)*ActionSet(action)' > 12)
        pause(0.001);
    end
    MotorNotch_ = MotorNotch;   % 保存上一次
    MotorNotch = MotorNotch + ones(2, 1)*ActionSet(action)';
    
    count = 0;
    count_detaT = 0;
    Rsum = 0;
    for itr = 1:nT
        
        tmpBo = X(Nt+1, :) < Speed_Change_Point;
%         Rspeed(:, tmpBo) = reference_speeds(1);
        Rspeed(:, ~tmpBo) = reference_speeds(2);
        
        %% 计算四个力
        
        U = zeros(Nt, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
        % ---------------------------------------------- 机车牵引力
        tmp_Fl = GetLocomotiveF(X(carType, :)*3.6, MotorNotch, TBcl);
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
        
        % 
        %% 更新状态方程
        
        X = Ad*X + Bd*U;
        
%         X = X_;
        
        %% 记录状态变量
        
        %     locF(itr, :) = tmp_Fl(1, :);
        %     airF(itr, :) = mean(currentF);
        %     addF(itr, :) = mean(addForce);
        %     basF(itr, :) = mean(basicForce);
        
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
        
        boSpeedFlag = X(1,:)*3.6 <= 10 | X(1,:)*3.6 >= 100; 
        
        if any(boSpeedFlag)     % 任何一个超速了，中断所有
            
            disp('Speed limit, break the loop!');
            
            v_tmp_ = v_tmp;
            s_tmp_ = s_tmp;
            
            v_tmp = ceil(X(1, :)*3.6) + 1;
            s_tmp = ceil(X(Nt+1, :)/dl) + 1;
            
            for i = 1:Ne
                if boSpeedFlag(i)
                    Q(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13,  action(i)) = ...
                        Q(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13,  action(i))...
                        + alpha*(large_neg_reward + gamma*max(Q(s_tmp(i), v_tmp(i), MotorNotch(1, i)+13, :)) ...
                        - Q(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13, action(i)));
                end
            end
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
            
            R = zeros(1, Ne);
            for atr = 1:4 
                R = R - reward_weigh(atr)*R_tmp(atr, :);
            end
            
            Rsum = Rsum + R;
            
            v_tmp_ = v_tmp;
            s_tmp_ = s_tmp;
            
            v_tmp = ceil(X(1, :)*3.6) + 1;
            s_tmp = ceil(X(Nt+1, :)/dl) + 1;
            
            
            %% 选择下一步的操作 % epsilon - greedy
            
            action_ = action;
            bo_tmp = rand(1, Ne) > epsilon;
            for i = 1:Ne
                if bo_tmp(i)      % epsilon - greedy
                    [~, action_(i)] = max(Q(s_tmp(i), v_tmp(i), MotorNotch(1, i)+13, :));
                else
                    action_tmp = find(~isnan(Q(s_tmp(i), v_tmp(i), MotorNotch(1, i)+13, :)));
                    action_(i) = action_tmp(ceil(rand*length(action_tmp)));
                end
            end
            
            %%
            for i = 1:Ne
                    Q(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13, action(i)) = ...
                        Q(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13, action(i))...
                        + alpha*(R(i) + gamma*(Q(s_tmp(i), v_tmp(i), MotorNotch(1, i)+13, action(i))) ...
                        - Q(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13, action(i))); 
            end
            
            action = action_;
            
            %% 
            
            if any(MotorNotch + ones(2, 1)*ActionSet(action)' > 12)
                pause(0.001);
            end
            
            MotorNotch_ = MotorNotch;   % 保存上一次
            MotorNotch = MotorNotch + ones(2, 1)*ActionSet(action)';   % 每次变化 -1, 0 or 1.  
            
        end
        
        loc_Notchs1(itr, :) = MotorNotch(1, :);
        loc_Notchs2(itr, :) = MotorNotch(2, :); 
        
    end
        R_Recorder(etr, :) =  Rsum;
end

close(hwait)
toc

%% 

% figure(1);clf
% mesh(U_recorder')
% 
% figure(2);clf
% plot(V_recorder')

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
linkaxes(axes, 'x');

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




% figure; 
% hold on;
% for itr = 1:4
%     plot(reward_Record(:, 1, itr)*reward_weigh(itr))
% end

% for itr = 1:4
%     wei(itr) = mean(max(reward_Record(:, :, itr)));
% end

figure(302);clf
plot(R_Recorder,'DisplayName','R_Recorder')















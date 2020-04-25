% test_single_Episode_RealNameAction_05.m
% Q-learning with replay

% clear;clc

episode_loops = 2e3; 

large_neg_reward = -1e10;    % 一个很大的负reward

%%
dt = 0.1;
T = 90;
detaT = 3;
tvec = dt:dt:T;
nT = length(tvec);

reference_speeds = [80/3.6 80/3.6];
initial_speed = 75/3.6;
initial_notch = 8;

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

nnum = 25;

S = 90/3.6*T;

Speed_Change_Point = 1500;
replay_buffer_size = Ne*50;

dl = round(reference_speeds(1)*detaT*2);   % meter
dv = 1;     % km/h

s_vec = 0:dl:S+dl*1;
v_vec = 0:dv:100+dv*1;

ActionSet = get_ActionSet2();
n_act = size(ActionSet, 1);
% ActionSet = zeros(27,3);
% NumOfActions = size(ActionSet,1);

Q = zeros(nnum, nnum, length(s_vec), length(v_vec));
Qc = zeros(size(Q));

% a_101 = [-1 0 1]';
% % Q_ = Q(:, :, 1, 1);
% Q_ = reshape(1:25*25, 25, 25);
% 
% MotorNotch_ = MotorNotch;

% MotorNotch = [-12; -12];

% a = a_101 + MotorNotch(1, :) + 13;
% b = a_101 + MotorNotch(2, :) + 13;
% 
% a(a > 25) = 25;
% a(a < 1) = 1;
% b(b > 25) = 25;
% b(b < 1) = 1;
% 
% ntr = 1;
% q_ = Q_(a(:, ntr), b(:, ntr));


% action = ones(1, Ne);
actions = zeros(2, Ne);
% action_ = action;

MotorNotch = actions;

v_tmp = ceil(X(1, :)*3.6)+1;
s_tmp = ceil(X(Nt+1, :)/dl)+1;

% figure; mesh(Q(:, : , 5))

%% Reinforcement Learning paramerters

gamma = 1;
alpha = 0.8;

% epsilons = [linspace(0.5, 1e-2, episode_loops*8/10) zeros(1, episode_loops*2/10)];
% epsilons = [linspace(1e-5, 1e-5, episode_loops*8/10) zeros(1, episode_loops*2/10)];
epsilons = [linspace(4e-1, 5e-2, episode_loops-1) 0];


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

Repisode = zeros(episode_loops, 4);    % 每一步 episode 的回报

Rs_record4 = zeros(episode_loops, 4);    % 每一步的回报

Rs_record = zeros(in_epi_number, Ne);    % 每一步的回报
Qloc = zeros(in_epi_number, 6, Ne);    % 每一步的 state-action
QLOC = zeros(in_epi_number, 6, replay_buffer_size)*NaN;    % 每一步的 state-action
QLOC_score = ones(in_epi_number+1, replay_buffer_size)*large_neg_reward;

%% 低速、超速保护机制

lowSpeedFlag = zeros(1, Ne) == 1;
highSpeedFlag = zeros(1, Ne) == 1;

%% weight

% reward_weigh = [1/2.4e+14 1/6.64e+5 2e-6 1/1.63e3];
% reward_weigh = [1/4.5e3 0 0 1/1.63e3*1];
reward_weigh = [1/4e7 1/1e7 0 1/1e4*4];
% reward_weigh = [1/4e7 1/1e7 0 1/1e4*4];
% reward_weigh = [1/2e7 0 0 1/1e4*2];

%% 主循环


tic
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
    
    MotorNotch = zeros(Nl, Ne) + initial_notch;     % 每个机车有独立的级位
    
    epsilon  = epsilons(etr);
%     epsilon  = 0;
    
    X = zeros(Nt*2, Ne);     % 前面 Nt 个是速度，后面 Nt 个是位置
%     X(1:Nt, :) = ones(Nt, 1)*((rand(1, Ne)*80+10)/3.6);     % 初速度
    X(1:Nt, :) = ones(Nt, Ne)*initial_speed;     % 初速度
    
    Rspeed = ones(Nt, Ne)*reference_speeds(1);
    
    v_tmp = ceil(X(1, :)*3.6)+1;
    s_tmp = ceil(X(Nt+1, :)/dl)+1;
    
    for i = 1:Ne
        actions(:, i) = get_Action(Q(:, :, s_tmp(i), v_tmp(i)), epsilon, MotorNotch(:, i));
    end
    
%     MotorNotch_ = MotorNotch;   % 保存上一次
    MotorNotch = actions;
    
    count = 0;
    count_detaT = 0;
    R_tmp = zeros(4, Ne);
    Rsum = zeros(4, 1);
    Qloc = Qloc*0;
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
            
%             v_tmp_ = v_tmp;
%             s_tmp_ = s_tmp;
%             
%             v_tmp = ceil(X(1, :)*3.6) + 1;
%             s_tmp = ceil(X(Nt+1, :)/dl) + 1; 
%             
%             for i = 1:Ne
%                 if boSpeedFlag(i)
%                     Q(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13, MotorNotch_(2, i)+13) = ...
%                         Q(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13, MotorNotch_(2, i)+13) ...
%                         + alpha*(large_neg_reward+ gamma*get_MaxQvalue( ...
%                         Q(:, : ,MotorNotch_(1, i)+13, MotorNotch_(2, i)+13), [s_tmp_(i) v_tmp_(i)]) ...
%                         - Q(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13, MotorNotch_(2, i)+13));
%                     
%                     Qc(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13, MotorNotch_(2, i)+13) = ...
%                         Qc(s_tmp_(i), v_tmp_(i), MotorNotch_(1, i)+13, MotorNotch_(2, i)+13)+1;
%                 end
%             end
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
            
            v_tmp = ceil(X(1, :)*3.6) + 1;
            s_tmp = ceil(X(Nt+1, :)/dl) + 1;
            
            for i = 1:Ne   % 只记录，不更新Q值，等完成整个 episode 后更新Q矩阵
                Qloc(count, :, i) = [MotorNotch(1, i), MotorNotch(2, i),  s_tmp_(i), v_tmp_(i), ...
                    s_tmp(i), v_tmp(i)];
            end
            Rs_record(count, :) = R;
            
            %% 选择下一步的操作 % epsilon - greedy
                        
            for i = 1:Ne
                actions(:, i) = get_Action(Q(:, :, s_tmp(i), v_tmp(i)), epsilon, MotorNotch(:, i));
            end
            
            %%
%             MotorNotch_ = MotorNotch;   % 保存上一次
            MotorNotch = actions;
        end
        
        loc_Notchs1(itr, :) = MotorNotch(1, :);
        loc_Notchs2(itr, :) = MotorNotch(2, :); 
        
    end
    %% 当一次探索完成后，更新Q矩阵
    
    Qloc(:, [1 2],:) = Qloc(:, [1 2],:)+13;
    
    for i = 1:Ne
        R_tmp_  = Rs_record(:, i);   
        for jtr = in_epi_number:-1:1       % 从后向前更新
            l = Qloc(jtr, :, i);
            Q(l(1), l(2), l(3), l(4)) = Q(l(1), l(2), l(3), l(4))...
                + alpha*(R_tmp_(jtr)+ gamma*get_MaxQvalue(Q(:, : ,l(5), l(6)), l(1:2)) ...
                - Q(l(1), l(2), l(3), l(4)));
            Qc(l(1), l(2), l(3), l(4)) = Qc(l(1), l(2), l(3), l(4)) + 1;
        end
    end
    
    %% 仅保存最优的那部分进行 replay
    tmp_score = sum(Rs_record);
    [current_min, ind_min] = min(QLOC_score(end, :));
    flag = false;
    for i = 1:Ne
        if flag
            [current_min, ind_min] = min(QLOC_score(end, :));
            flag = false;
        end
        if tmp_score(i) > current_min
            QLOC(:, :, ind_min) = Qloc(:, :, i);
            QLOC_score(:, ind_min) = [Rs_record(:, i); tmp_score(i)];
            flag = true;
        end
    end
    
    % experience replay
%     [~, sort_ind] = sort(QLOC_score(end, :), 'descend');    % 按照最优回报从高到底replay
    [~, sort_ind] = sort(rand(replay_buffer_size, 1));    % 随机 replay
    for i = 1:replay_buffer_size*0.2
        if QLOC_score(1, sort_ind(i)) > large_neg_reward
            R_tmp_  = QLOC_score(1:in_epi_number, sort_ind(i));
            for jtr = in_epi_number:-1:1       % 从后向前更新
                l = QLOC(jtr, :, i);
                if isnan(l(1))
                    continue;
                end
                Q(l(1), l(2), l(3), l(4)) = Q(l(1), l(2), l(3), l(4))...
                    + alpha*(R_tmp_(jtr)+ gamma*get_MaxQvalue(Q(:, : ,l(5), l(6)), l(1:2)) ...
                    - Q(l(1), l(2), l(3), l(4)));
                Qc(l(1), l(2), l(3), l(4)) = Qc(l(1), l(2), l(3), l(4)) + 1;
            end
        end
    end
    %% 仅保存最优的那部分进行 replay
%     tmp_score = sum(Rs_record);
%     
%     QLOC = Qloc;
%     QLOC_score = [Rs_record; tmp_score];
    
%     [current_min, ind_min] = min(QLOC_score(end, :));
%     flag = false;
%     for i = 1:Ne
%         if flag
%             [current_min, ind_min] = min(QLOC_score(end, :));
%             flag = false;
%         end
%         if tmp_score(i) > current_min
%             QLOC(:, :, ind_min) = Qloc(:, :, i);
%             QLOC_score(:, ind_min) = [Rs_record(:, i); tmp_score(i)];
%             flag = true;
%         end
%     end
%     
%     % experience replay
% %     [~, sort_ind] = sort(QLOC_score(end, :), 'descend');    % 按照最优回报从高到底replay
%     [~, sort_ind] = sort(rand(replay_buffer_size, 1));    % 随机 replay
%     for i = 1:replay_buffer_size*0.1
%         if QLOC_score(1, sort_ind(i)) > large_neg_reward
%             R_tmp_  = QLOC_score(1:in_epi_number, sort_ind(i));
%             for jtr = in_epi_number:-1:1       % 从后向前更新
%                 l = Qloc(jtr, :, i);
%                 Q(l(1), l(2), l(3), l(4)) = Q(l(1), l(2), l(3), l(4))...
%                     + alpha*(R_tmp_(jtr)+ gamma*get_MaxQvalue(Q(:, : ,l(5), l(6)), l(1:2)) ...
%                     - Q(l(1), l(2), l(3), l(4)));
%                 Qc(l(1), l(2), l(3), l(4)) = Qc(l(1), l(2), l(3), l(4)) + 1;
%             end
%         end
%     end
    
    %% 
    Rs_record4(etr, :) = Rsum';
    R_Recorder(etr, :) = sum(Rs_record);
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

col = 12
vl = 81 - 0

figure(2000+1); clf
subplot(211);
tmpQ = Q(:, :, col, vl);
tmpQ(tmpQ == 0) = NaN;
mesh(-tmpQ);
set(gca, 'XLim', [0 25], 'YLim', [0 25]);
subplot(212);
tmpQc = Qc(:, :, col, vl);
tmpQc(tmpQc == 0) = NaN;
mesh(tmpQc);
set(gca, 'XLim', [0 25], 'YLim', [0 25]);
























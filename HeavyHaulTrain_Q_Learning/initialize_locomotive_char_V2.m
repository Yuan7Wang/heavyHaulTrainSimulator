% 定义机车特性、空气制动特性   % initialize_locomotive_char.m

function [Ad, Bd, TBcl, AirTimeLim, Abr, LenTrain, mTrainGroup, CC0, CCa, KKK, DDD] = ...
    initialize_locomotive_char_V2(sampling_time, carType, Ne)
%% Load the locomotive traction/brake characteristics

load('TCL_force');   % 12 cells contain the locomotive traction characteristics Force(kN)-Velocity(km/h)
load('BCL_force');   % 12 cells contain the locomotive regenerative brake characteristics Force(kN)-Velocity(km/h)
load('MatrixTBCL_force');

% load('TCL_forceHXD3.mat');   % 12 cells contain the locomotive traction characteristics Force(kN)-Velocity(km/h)
% load('BCL_forceHXD3');   % 12 cells contain the locomotive regenerative brake characteristics Force(kN)-Velocity(km/h)
% load('MatrixTBCL_forceHXD3.mat')

% TBcl = MatrixTBCL_forceHXD3;
TBcl = MatrixTBCL_force;

% Tcl = cell(length(TCL_force), 1); %#ok<USENS>
% for itr = 1:length(TCL_force)
%     Tcl{itr} = griddedInterpolant(TCL_force{itr}(:, 1), TCL_force{itr}(:, 2), 'linear');
% end
% 
% TBcl0 = griddedInterpolant([-100 1000], [0 0], 'linear');    % 0 级情况下插值，均为0
% 
% Bcl = cell(length(BCL_force), 1); %#ok<USENS>
% for itr = 1:length(BCL_force)
%     Bcl{itr} = griddedInterpolant(BCL_force{itr}(1, :), BCL_force{itr}(2, :), 'linear');
% end
% 
% TBcl = [Bcl(end:-1:1); {TBcl0}; Tcl];    % 注意顺序



%% Parameter of Locomotives.
mass_locomotive = 150000;   % kg
leng_locomotive = 22.781;  % m
c0_locomotive = 12.441*10^(-3);  % N/kg
cv_locomotive = 0.744*10^(-4);    % N.s/(m.kg)
ca_locomotive = 0.3233*10^(-5);    % N.s^2/(m^2.kg)
k_locomotive = 112*10^6;    % N/m, this is the minimal value, the maximum value is 78-121*10^6
d_locomotive = 112*10^4;    % N.s/m, this is the minimal value, the maximum value is 78-121*10^4

%% Parameter of Wagons.
mass_wagon = 70000;  % kg
leng_wagon = 12.026;    % m
c0_wagon = 8.0365*10^(-3);   % N/kg
cv_wagon = 0.5232*10^(-4);     % N.s/(m.kg)
ca_wagon = 0.1123*10^(-5);   % N.s^2/(m^2.kg)
k_wagon = 98*10^6;     % N/m, this is the minimal value, the maximum value is 29.29-49*10^6
d_wagon = 98*10^4;     % kg/s, this is the minimal value, the maximum value is 29.29-49*10^4

%% Train makeup.


Nt = length(carType);
Nl = sum(carType);

% carType = zeros(Nt, 1) == 1;  % 车辆类型，机车还是拖车
% carType([1 end]) = true;        % 一头一尾两个机车

NumofLocoPerGroup = 1;
NumofWagonPerGroup = 1;
% LocoGroup1 = 1;
% LocoGroup2 = 202;
% WagonGroup1 = 2:201;

position_locomotive = find(carType);
position_wagon = find(~carType);

% position_LocoAir = [1 2 3 4 205 206];
% position_WagonAir = 5:204;

diff_carType = diff(carType) ~= 0;   % 计算差分，值为 0 则表示wagon车钩，非 0 为机车
position_locomotivecoupler = find(diff_carType); 
position_wagoncoupler = find(~diff_carType); 

% position_wagoncoupler = 2:200;
mass_TrainGroup = zeros(Nt, 1);
mass_TrainGroup(position_locomotive) = NumofLocoPerGroup*mass_locomotive;
mass_TrainGroup(position_wagon) = NumofWagonPerGroup*mass_wagon;
leng_TrainGroup(position_locomotive) = NumofLocoPerGroup*leng_locomotive;
leng_TrainGroup(position_wagon) = NumofWagonPerGroup*leng_wagon;
% leng_Cars(position_LocoAir) = leng_locomotive;
% leng_Cars(position_WagonAir) = leng_wagon;
num_CarGroup = length(mass_TrainGroup);
num_LocoGroup = length(position_locomotive);
num_locomotive = num_LocoGroup*NumofLocoPerGroup;
num_WagonGroup = length(position_wagon);
num_wagon = num_WagonGroup*NumofWagonPerGroup;
num_Car = num_locomotive+num_wagon;
distribution_loco = NumofLocoPerGroup*ones(1,num_LocoGroup);
distribution_wagon = NumofWagonPerGroup*ones(1,num_WagonGroup);

mTrainGroup = mass_TrainGroup*ones(1, Ne);

%% Simulation set up
simulation_time = 200;  % seconds
% sampling_time = 0.1;   % seconds
simulation_step = simulation_time/sampling_time;

%% Parameter transform to Vector
KK(position_locomotivecoupler) = k_locomotive;
KK(position_wagoncoupler) = k_wagon;
DD(position_locomotivecoupler) = d_locomotive;
DD(position_wagoncoupler) = d_wagon;
C0(position_locomotive) = c0_locomotive;
C0(position_wagon) = c0_wagon;
Cv(position_locomotive) = cv_locomotive;
Cv(position_wagon) = cv_wagon;
Ca(position_locomotive) = ca_locomotive;
Ca(position_wagon) = ca_wagon;

CC0 = C0'*ones(1, Ne);
CCa = Ca'*ones(1, Ne);
KKK = KK'*ones(1 ,Ne);
DDD = DD'*ones(1 ,Ne);

%% Random Variance for Parameter.
% KK = KK+rand(1,length(KK))*0.1.*KK;
% DD = DD+rand(1,length(DD))*0.1.*DD;
% C0 = C0+rand(1,length(C0))*0.1.*C0;
% Cv = Cv+rand(1,length(Cv))*0.1.*Cv;
% Ca = Ca+rand(1,length(Ca))*0.1.*Ca;

%% Transform the model to state space form.
[Ac, Bc] = TransParameterToStateSpace(mass_TrainGroup,num_CarGroup,KK,DD,Cv);
[Ad, Bd] = TransContinuousToDiscrete(Ac,Bc,sampling_time);


%% %% Load the air brake characteristics

% A = 20; % kN

alpha1 = linspace(-0.1, -0.04, 100)';
alpha2 = linspace(-0.02, -0.01, 100)';

Alpha1 = [alpha1; alpha1(end:-1:1)]*ones(1, Ne);
Alpha2 = [alpha2; alpha2(end:-1:1)]*ones(1, Ne);

lenTrain = cumsum(leng_TrainGroup(2:101)');
air_speed = 280; % m/s2

airTimeDelay = lenTrain/air_speed;
airTimeDelay = [airTimeDelay; airTimeDelay(end:-1:1)];
AirTimeLim = airTimeDelay*ones(1, Ne);

Abr.Alpha1 = Alpha1;
Abr.Alpha2 = Alpha2;

%% 

LenTrain = -cumsum(leng_TrainGroup);
LenTrain = LenTrain-LenTrain(1);




% airIndex = [0 1:100 100:-1:1 0];   % 车辆对应的空气制动编号

% AB_force = @(T) A*(1-exp(Alpha1.*T));
% AB_time = @(F) log(F/A+1)./Alpha1;
% 
% AR_force = @(T) -A*(exp(Alpha2.*T));
% AR_time = @(F) log(-F/A)./Alpha2;

% Abr.AB_force = AB_force;
% Abr.AB_time = AB_time;
% Abr.AR_force = AR_force;
% Abr.AR_time = AR_time;

% lenTrain/280










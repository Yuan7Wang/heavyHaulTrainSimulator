%% PART I:INITIALIZE THE TRAIN PARAMETERS
%% Load the infrastructure
load('RampPositionList');  % 1.Format:start(m) end(m) gradient, 2.Range: [-10000m,100000m]
load('RampPoint'); % 1.Format:position(m) Height(m),  2.Range:[9000m,37972m]
load('CurvePositionList'); % 1.Format:start(m) end(m) radius(m) *LIMIT 2.Range: [-10000m,100000m]

%% Load the locomotive traction/brake characteristics
load('TCL_force');   % 12 cells contain the locomotive traction characteristics Force(kN)-Velocity(km/h)
load('BCL_force');   % 12 cells contain the locomotive regenerative brake characteristics Force(kN)-Velocity(km/h)
load('MatrixTBCL_force');

%% Load the air brake characteristics
load('ABCL_Force');

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
NumofLocoPerGroup = 1;
NumofWagonPerGroup = 1;
LocoGroup1 = 1;
LocoGroup2 = 202;
WagonGroup1 = 2:201;
position_locomotive = [LocoGroup1 LocoGroup2];
position_wagon = WagonGroup1;
% position_LocoAir = [1 2 3 4 205 206];
% position_WagonAir = 5:204;
position_locomotivecoupler = [1 201];
position_wagoncoupler = 2:200;
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

%% Simulation set up
simulation_time = 200;  % seconds
sampling_time = 0.1;   % seconds
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

%% Random Variance for Parameter.
% KK = KK+rand(1,length(KK))*0.1.*KK;
% DD = DD+rand(1,length(DD))*0.1.*DD;
% C0 = C0+rand(1,length(C0))*0.1.*C0;
% Cv = Cv+rand(1,length(Cv))*0.1.*Cv;
% Ca = Ca+rand(1,length(Ca))*0.1.*Ca;

%% Transform the model to state space form.
[A_continuous,B_continuous] = TransParameterToStateSpace(mass_TrainGroup,num_CarGroup,KK,DD,Cv);
[A_discrete,B_discrete] = TransContinuousToDiscrete(A_continuous,B_continuous,sampling_time);
% v = vrho(A_discrete);

%% Air brake set.
TotalLevel = 40;
MaxBrakeForce(position_locomotive) = 0;  %kN
MaxBrakeForce(position_wagon) = 400;  %kN
VectorBrake{1} = TotalLevel;
VectorBrake{2} = MaxBrakeForce;
VectorBrake{3} = sampling_time;

%% Initialize the Train state.
TrainPosition = zeros(num_CarGroup,simulation_step);
TrainPosition(1,1) = 0;
temp = 0.5*(leng_TrainGroup(1:end-1)+leng_TrainGroup(2:end));
for i = 2:num_CarGroup
    TrainPosition(i,1) = TrainPosition(i-1,1)-temp(i-1);
end
position_ConnectLoco = [TrainPosition(position_locomotive(1),1)' TrainPosition(position_locomotive(2),1)'];
AirBrakeSignalTransferTime = zeros(1,num_CarGroup);
for i = 1:num_CarGroup
    AirBrakeSignalTransferTime(i) = min(abs(TrainPosition(i)-position_ConnectLoco))/280;
end
% plot(AirBrakeSignalTransferTime,'Linewidth',2,'color','b');
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Car Number','FontSize',12,'Fontname','Times New Roman')
% ylabel('Air Brake Signal Transfer Time(s)','FontSize',12,'Fontname','Times New Roman')

temp = ones(num_locomotive,num_CarGroup)*diag(1:num_CarGroup);
temp1 = diag(position_locomotive)*ones(num_locomotive,num_CarGroup);
CarBrakeCoefficient = min(abs(temp-temp1));
VectorBrake{4} = linspace(-0.1,-0.04,max(CarBrakeCoefficient));
VectorBrake{5} = linspace(-0.02,-0.01,max(CarBrakeCoefficient));

TimeFromAirBrakeSignal = zeros(1,num_CarGroup);
ActuatorAirBrakeTime = zeros(1,num_CarGroup);
ActuatorAirBrakeNotch = zeros(num_CarGroup,2);
TraceActuatorAirBrakeNotch = zeros(num_CarGroup,simulation_step);
AirBrakeNotch = zeros(num_CarGroup,simulation_step);
AirBrakeForce = zeros(num_CarGroup,simulation_step);
TrainDistance = zeros(num_CarGroup,simulation_step);
% StartDis = 0;
TrainVelocity = zeros(num_CarGroup,simulation_step);
% VelocityError = zeros(1,simulation_step);
TargetTrainVelocity = zeros(1,simulation_step);
TrainVelocity(:,1) = 60/3.6;
TargetTrainVelocity(1) = 60/3.6;
TrainAcceleration = zeros(num_CarGroup,simulation_step);
CouplerForce = zeros(num_CarGroup,simulation_step); % Coupler force per car
LocomotiveMotorForce = zeros(num_LocoGroup,simulation_step); % same velocity has the same motor force.
LocomotiveMotorNotch = zeros(num_LocoGroup,simulation_step);
TrainForce = zeros(num_CarGroup,simulation_step);
LinearizationFeedbackTrainForce = zeros(num_CarGroup,simulation_step);
BasicResistance = zeros(num_CarGroup,simulation_step);
AdditionalResistance = zeros(num_CarGroup,simulation_step);
RampResistance = zeros(num_CarGroup,simulation_step);
CurveResistance = zeros(num_CarGroup,simulation_step);
CouplerForcePerCoupler = zeros(num_CarGroup-1,simulation_step);
TrainCouplerElastic = zeros(num_CarGroup-1,simulation_step);
% DrivingSequenceChangePoint = TrainPosition(1,1)+300:300:30000;
AirBrakeSignal = 0;

function [CouplerForce,CouplerForcePerCoupler,TrainCouplerElastic] = GetCouplerForce(k_locomotive,d_locomotive,k_wagon,d_wagon,...
    TrainDistance,TrainVelocity,position_locomotivecoupler,position_wagoncoupler)

%% Hardening Coupler.
% epsilon = 0.5;
% ratio_kd = 0.01;
% KK(position_locomotive) = k_locomotive;
% KK(position_wagon) = k_wagon;
% delta_Distance = TrainDistance(1:end-1)-TrainDistance(2:end);
% delta_Velocity = TrainVelocity(1:end-1)-TrainVelocity(2:end);
% TrainCouplerElastic = KK.*(1+epsilon*delta_Distance.^2);
% CouplerForcePerCoupler = TrainCouplerElastic.*(delta_Distance+ratio_kd*delta_Velocity);
% CouplerForce = [0;CouplerForcePerCoupler]-[CouplerForcePerCoupler;0];

%% Linear Coupler.
    KK(position_locomotivecoupler) = k_locomotive;
    KK(position_wagoncoupler) = k_wagon;
    KK1 = [KK';0];
    KK2 = [0;KK'];
    DISTANCE = TrainDistance(1:end-1)-TrainDistance(2:end);
    DISTANCE1 = [DISTANCE;0];
    DISTANCE2 = [0;-DISTANCE];
    CouplerForce = -KK1.*DISTANCE1-KK2.*DISTANCE2;

    DD(position_locomotivecoupler) = d_locomotive;
    DD(position_wagoncoupler) = d_wagon;
    DD1 = [DD';0];
    DD2 = [0;DD'];
    VELOCITY = TrainVelocity(1:end-1)-TrainVelocity(2:end);
    VELOCITY1 = [VELOCITY;0];
    VELOCITY2 = [0;-VELOCITY];
    CouplerForce = CouplerForce-DD1.*VELOCITY1- DD2.*VELOCITY2;

    CouplerForcePerCoupler = KK'.*DISTANCE+DD'.*VELOCITY;
    TrainCouplerElastic = KK';
    
    
    
    
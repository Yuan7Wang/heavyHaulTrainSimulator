function [AdditionalResistance,RampResistance,CurveResistance] = ...
    GetAdditionalResistance(mass_TrainGroup,TrainPosition,RampPositionList,CurvePositionList,p)
NumofTrainGroup = length(mass_TrainGroup);
AdditionalResistance = zeros(NumofTrainGroup,1);

%% Calculate the ramp resistance. N
ReverseTrainPosition = TrainPosition(end:-1:1);
mass_TrainGroup = mass_TrainGroup(end:-1:1);
RampPositionListIndice = find(RampPositionList(:,2) >= ReverseTrainPosition(end),1);
RampPositionListIndice1 = find(RampPositionList(:,2) > ReverseTrainPosition(1),1);
gravitational_acceleration = 9.98;

if RampPositionListIndice-RampPositionListIndice1 >= 2
    for k = 1:RampPositionListIndice-RampPositionListIndice1+1
        if k == 1
            RampGradient = RampPositionList(RampPositionListIndice,3);
            TrainGroupFinalIndice = NumofTrainGroup;
            TrainGroupStartIndice = find(ReverseTrainPosition >= RampPositionList(RampPositionListIndice,1),1);
            AdditionalResistance(TrainGroupStartIndice:TrainGroupFinalIndice) = ...
                mass_TrainGroup(TrainGroupStartIndice:TrainGroupFinalIndice)*gravitational_acceleration*RampGradient;
        elseif k < RampPositionListIndice-RampPositionListIndice1+1
            RampGradient = RampPositionList(RampPositionListIndice-k+1,3);
            TrainGroupFinalIndice = TrainGroupStartIndice-1;
            TrainGroupStartIndice = find(ReverseTrainPosition >= RampPositionList(RampPositionListIndice-k+1,1),1);
            AdditionalResistance(TrainGroupStartIndice:TrainGroupFinalIndice) = ...
                mass_TrainGroup(TrainGroupStartIndice:TrainGroupFinalIndice)*gravitational_acceleration*RampGradient;
        else
            RampGradient = RampPositionList(RampPositionListIndice-k+1,3);
            TrainGroupFinalIndice = TrainGroupStartIndice-1;
            TrainGroupStartIndice = 1;
            AdditionalResistance(TrainGroupStartIndice:TrainGroupFinalIndice) = ...
                mass_TrainGroup(TrainGroupStartIndice:TrainGroupFinalIndice)*gravitational_acceleration*RampGradient;
        end
    end
elseif RampPositionListIndice-RampPositionListIndice1 == 1
    for k = 1:RampPositionListIndice-RampPositionListIndice1+1
        if k == 1
            RampGradient = RampPositionList(RampPositionListIndice,3);
            TrainGroupFinalIndice = NumofTrainGroup;
            TrainGroupStartIndice = find(ReverseTrainPosition >= RampPositionList(RampPositionListIndice,1),1);
            AdditionalResistance(TrainGroupStartIndice:TrainGroupFinalIndice) = ...
                mass_TrainGroup(TrainGroupStartIndice:TrainGroupFinalIndice)*gravitational_acceleration*RampGradient;
        else
            RampGradient = RampPositionList(RampPositionListIndice-k+1,3);
            TrainGroupFinalIndice = TrainGroupStartIndice-1;
            TrainGroupStartIndice = 1;
            AdditionalResistance(TrainGroupStartIndice:TrainGroupFinalIndice) = ...
                mass_TrainGroup(TrainGroupStartIndice:TrainGroupFinalIndice)*gravitational_acceleration*RampGradient;
        end
    end
else
    RampGradient = RampPositionList(RampPositionListIndice,3);
    TrainGroupFinalIndice = NumofTrainGroup;
    TrainGroupStartIndice = 1;
    AdditionalResistance(TrainGroupStartIndice:TrainGroupFinalIndice) = ...
        mass_TrainGroup(TrainGroupStartIndice:TrainGroupFinalIndice)*gravitational_acceleration*RampGradient;
end
RampResistance = AdditionalResistance(end:-1:1);

%% Calculate the Curve Resistance. N
%% This procedure is built based on the assumption that every curve section is longer than the 
%% distance between adjacent cars.

%% OPTION 1
CurveResistance = 0;

%% OPTION 2
% num_CurveSection = size(CurvePositionList,1);
% for k = 1:num_CurveSection
%     TrainCurveStartIndice = find(CurvePositionList(k,1) < ReverseTrainPosition,1);
%     TrainCurveEndIndice = find(CurvePositionList(k,2) < ReverseTrainPosition,1);
%     CurveRadius = CurvePositionList(k,3);
%     if ~isempty(TrainCurveStartIndice) && isempty(TrainCurveEndIndice)
%         TrainCurveEndIndice = NumofTrainGroup+1;
%         AdditionalResistance(TrainCurveStartIndice:TrainCurveEndIndice-1) = ...
%             AdditionalResistance(TrainCurveStartIndice:TrainCurveEndIndice-1)+...
%             0.004*0.5*8.310/CurveRadius*mass_TrainGroup(TrainCurveStartIndice:TrainCurveEndIndice-1)';
%     elseif ~isempty(TrainCurveStartIndice) && ~isempty(TrainCurveEndIndice) && ...
%             (TrainCurveStartIndice < TrainCurveEndIndice)
%         AdditionalResistance(TrainCurveStartIndice:TrainCurveEndIndice-1) = ...
%             AdditionalResistance(TrainCurveStartIndice:TrainCurveEndIndice-1)+...
%             0.004*0.5*8.310/CurveRadius*mass_TrainGroup(TrainCurveStartIndice:TrainCurveEndIndice-1)';
%     end
% end
% AdditionalResistance = AdditionalResistance(end:-1:1);
% CurveResistance = AdditionalResistance-RampResistance;



















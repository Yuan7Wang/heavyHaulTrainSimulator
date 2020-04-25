function [LocomotiveMotorNotch,AirBrakeNotch] = GetTrainNotches(num_CarGroup,CurrentNotch)

LocomotiveMotorNotch = [CurrentNotch(1);CurrentNotch(2)];
AirBrakeNotch = CurrentNotch(3)*ones(num_CarGroup,1);


%% Train driving sequence 2.(Based on Distance)
% if TrainPosition < DrivingSequenceChangePoint(1)
%     LocomotiveMotorNotch = CurrentNotch*ones(6,1);
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(2)
%     LocomotiveMotorNotch = CurrentNotch*ones(6,1);
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(3)
%     LocomotiveMotorNotch = [-10;-10;-10;-10;-10;-10];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(4)
%     LocomotiveMotorNotch = [5;5;5;5;0;0];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(5)
%     LocomotiveMotorNotch = [0;0;0;0;0;0];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(6)
%     LocomotiveMotorNotch = [0;0;0;0;0;0];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(7)
%     LocomotiveMotorNotch = [-10;-10;-10;-10;-10;-10];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(8)
%     LocomotiveMotorNotch = [-10;-10;-10;-10;-10;-10];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(9)
%     LocomotiveMotorNotch = [-10;-10;-10;-10;-10;-10];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(10)
%     LocomotiveMotorNotch = [-10;-10;-10;-10;-10;-10];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(11)
%     LocomotiveMotorNotch = [-10;-10;-10;-10;-10;-10];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(12)
%     LocomotiveMotorNotch = [-10;-10;-10;-10;-10;-10];
%     AirBrakeNotch = 0*ones(num_Car,1);
% elseif TrainPosition < DrivingSequenceChangePoint(13)
%     LocomotiveMotorNotch = [-10;-10;-10;-10;-10;-10];
%     AirBrakeNotch = 0*ones(num_Car,1);
% else
%     LocomotiveMotorNotch = [-10;-10;-10;-10;-10;-10];
%     AirBrakeNotch = -3*ones(num_Car,1);
% end











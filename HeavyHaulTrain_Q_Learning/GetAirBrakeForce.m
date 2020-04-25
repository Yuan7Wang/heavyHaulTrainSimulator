%% Obtain the Air Brake force for each locomotive and wagon.
%% ABCL_Force = [Time;BrakeForce;Gradient]: cell, 1,3,5....represent brake curve,2,4,6...represent release curve.
%% VectorBrake = {TotalLevel} {MaxBrakeForce} {sampling_time} {CarBrakeCoefficient} 
%% {CarReleaseCoefficient} 5 cells

function AirBrakeForce = GetAirBrakeForce(ActuatorAirBrakeNotch,AirBrakeForce,VectorBrake,CarBrakeCoefficient)

AirBrakeForce(AirBrakeForce > 0) = 0;

TargetForce = ActuatorAirBrakeNotch/VectorBrake{1}.*VectorBrake{2}';
temp = (AirBrakeForce > TargetForce);
AirBrakeForce(temp) = AirBrakeForce(temp)+VectorBrake{4}(CarBrakeCoefficient(temp))*(AirBrakeForce(temp)+20)*VectorBrake{3};
temp = (AirBrakeForce < TargetForce);
AirBrakeForce(temp) = AirBrakeForce(temp)+VectorBrake{5}(CarBrakeCoefficient(temp))*AirBrakeForce(temp)*VectorBrake{3};







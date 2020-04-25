function TrainForce = GetTrainForce(LocomotiveMotorForce,AirBrakeForce,...
    position_locomotive,position_wagon)

TrainForce(position_locomotive) = LocomotiveMotorForce+AirBrakeForce(position_locomotive);
TrainForce(position_wagon) = AirBrakeForce(position_wagon);

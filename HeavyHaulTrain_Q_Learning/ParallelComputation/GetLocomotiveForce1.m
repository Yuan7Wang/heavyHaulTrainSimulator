%% obtain the force for each locomotive group.
function LocomotiveMotorForce = GetLocomotiveForce(TrainVelocity,LocomotiveMotorNotch,...
    TCL_force,BCL_force,num_LocoGroup)

%% Change the unit of Velocity.
TrainVelocity = TrainVelocity*3.6;
LocomotiveMotorForce = zeros(num_LocoGroup,1);
% LocomotiveMotorNotch = LocomotiveMotorNotch+11;
%% Option 3:Matrix computation for each car.
% if LocomotiveMotorNotch(1) ~= LocomotiveMotorNotch(5)
%     pause(0.1);
% end
% temp = TrainVelocity-MatrixTBCL_force(LocomotiveMotorNotch,:,1);
% temp(temp >= 0) = 1;
% temp(temp < 0) = 0;
% temp = sum(temp,2);
% temp1 = MatrixTBCL_force(LocomotiveMotorNotch,temp,1); %velocity.
% temp2 = MatrixTBCL_force(LocomotiveMotorNotch,temp,2); %Force.
% temp3 = MatrixTBCL_force(LocomotiveMotorNotch,temp,3); %gradient.
% temp1 = temp1(1:1+size(temp1,1):end)';
% temp2 = temp2(1:1+size(temp2,1):end)';
% temp3 = temp3(1:1+size(temp3,1):end)';
% LocomotiveMotorForce = temp2+temp3.*(TrainVelocity-temp1);

%% Option 1: All the speeds are regarded as the same.
% i = 1;
% if LocomotiveMotorNotch(i) > 0
%     temp_TCL_velocity = TCL_force{LocomotiveMotorNotch(i)}(:,1);
%     temp_TCL_force = TCL_force{LocomotiveMotorNotch(i)}(:,2);
%     
%     indice = find(temp_TCL_velocity > TrainVelocity(i),1)-1;
%     temp_gradient = (temp_TCL_force(indice+1)-temp_TCL_force(indice))/(temp_TCL_velocity(indice+1)-temp_TCL_velocity(indice));
%     LocomotiveMotorForce = temp_TCL_force(indice)+temp_gradient*(TrainVelocity(i)-temp_TCL_velocity(indice));
% elseif LocomotiveMotorNotch(i) < 0
%     temp_BCL_velocity = BCL_force{-LocomotiveMotorNotch(i)}(:,1);
%     temp_BCL_force = BCL_force{-LocomotiveMotorNotch(i)}(:,2);
%     
%     indice = find(temp_BCL_velocity > TrainVelocity(i),1)-1;
%     temp_gradient = (temp_BCL_force(indice+1)-temp_BCL_force(indice))/(temp_BCL_velocity(indice+1)-temp_BCL_velocity(indice));
%     LocomotiveMotorForce = temp_BCL_force(indice)+temp_gradient*(TrainVelocity(i)-temp_BCL_velocity(indice));
%     LocomotiveMotorForce = -LocomotiveMotorForce(i);
% else
%     LocomotiveMotorForce = 0;
% end

%% Option 2: Calculate the Force for Locomotive Group 1.
for i = 1:num_LocoGroup
    if LocomotiveMotorNotch(i) > 0
        temp_TCL_velocity = TCL_force{LocomotiveMotorNotch(i)}(:,1);
        temp_TCL_force = TCL_force{LocomotiveMotorNotch(i)}(:,2);

        indice = find(temp_TCL_velocity > TrainVelocity(i),1)-1;
        temp_gradient = (temp_TCL_force(indice+1)-temp_TCL_force(indice))/(temp_TCL_velocity(indice+1)-temp_TCL_velocity(indice));
        LocomotiveMotorForce(i) = temp_TCL_force(indice)+temp_gradient*(TrainVelocity(i)-temp_TCL_velocity(indice));
    elseif LocomotiveMotorNotch(i) < 0
        temp_BCL_velocity = BCL_force{-LocomotiveMotorNotch(i)}(:,1);
        temp_BCL_force = BCL_force{-LocomotiveMotorNotch(i)}(:,2);

        indice = find(temp_BCL_velocity > TrainVelocity(i),1)-1;
        temp_gradient = (temp_BCL_force(indice+1)-temp_BCL_force(indice))/(temp_BCL_velocity(indice+1)-temp_BCL_velocity(indice));
        LocomotiveMotorForce(i) = temp_BCL_force(indice)+temp_gradient*(TrainVelocity(i)-temp_BCL_velocity(indice));
        LocomotiveMotorForce(i) = -LocomotiveMotorForce(i);
    else
        LocomotiveMotorForce(i) = 0;
    end
end












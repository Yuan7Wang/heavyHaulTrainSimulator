%% Obtain the Air Brake force for each locomotive and wagon.
%% ABCL_Force = [Time;BrakeForce;Gradient]: cell, 1,3,5....represent brake curve,2,4,6...represent release curve.
%% VectorBrake = {TotalLevel} {MaxBrakeForce} {sampling_time} 3 cells

function AirBrakeForce = GetAirBrakeForce(ActuatorAirBrakeNotch,AirBrakeForce,...
    num_CarGroup,position_wagon,position_locomotive,ABCL_Force,VectorBrake)

AirBrakeForce(AirBrakeForce > 0) = 0;

%% OPTION 3: Only Wagons have air brake force.
% if (sum(ActuatorAirBrakeNotch) == 0) && (sum(AirBrakeForce) >= -0.00001)
%     AirBrakeForce = zeros(num_CarGroup,1);
% else
%     i = 1;
%     TargetForce = 4*ActuatorAirBrakeNotch(i)/VectorBrake{1}*VectorBrake{2}(i);
%     if AirBrakeForce(i) > TargetForce
%         temp = find(ABCL_Force{1}(2,:) >= AirBrakeForce(i));
%         AirBrakeForce(position_wagon) = AirBrakeForce(i)-ABCL_Force{1}(3,temp(end))*VectorBrake{3};
%     elseif AirBrakeForce(i) < TargetForce
%         temp = find(ABCL_Force{2}(2,:) >= AirBrakeForce(i),1);
%         AirBrakeForce(position_wagon) = AirBrakeForce(i)+ABCL_Force{2}(3,temp)*VectorBrake{3};
%     end
% end
% AirBrakeForce(position_locomotive) = 0;

%% OPTION 1: All cars' brake force is the same as the first locomotive.
% if (sum(ActuatorAirBrakeNotch) == 0) && (sum(AirBrakeForce) >= -0.00001)
%     AirBrakeForce = zeros(num_CarGroup,1);
% else
%     i = 1;
%     temp = find(i == position_locomotive, 1);
%     if ~isempty(temp)
%         TargetForce = ActuatorAirBrakeNotch(i)/VectorBrake{1}*VectorBrake{2}(i);
%         if AirBrakeForce(i) > TargetForce
%             temp = find(ABCL_Force{1}(2,:) >= AirBrakeForce(i));
%             %             if isempty(temp)
%             %                 pause(0.1);
%             %             end
%             AirBrakeForce = AirBrakeForce(i)-ABCL_Force{1}(3,temp(end))*VectorBrake{3};
%         elseif AirBrakeForce(i) < TargetForce
%             temp = find(ABCL_Force{2}(2,:) >= AirBrakeForce(i),1);
%             AirBrakeForce = AirBrakeForce(i)+ABCL_Force{2}(3,temp)*VectorBrake{3};
%         end
%     else
%         TargetForce = 4*ActuatorAirBrakeNotch(i)/VectorBrake{1}*VectorBrake{2}(i);
%         if AirBrakeForce(i) > TargetForce
%             temp = find(ABCL_Force{1}(2,:) >= AirBrakeForce(i));
%             AirBrakeForce = AirBrakeForce(i)-ABCL_Force{1}(3,temp(end))*VectorBrake{3};
%         elseif AirBrakeForce(i) < TargetForce
%             temp = find(ABCL_Force{2}(2,:) >= AirBrakeForce(i),1);
%             AirBrakeForce = AirBrakeForce(i)+ABCL_Force{2}(3,temp)*VectorBrake{3};
%         end
%     end
% end

%% OPTION 2
if (sum(ActuatorAirBrakeNotch) == 0) && (sum(AirBrakeForce) >= -0.00001)
    AirBrakeForce = zeros(num_CarGroup,1);
else
    for i = 1:num_CarGroup
        temp = find(i == position_locomotive, 1);
        if ~isempty(temp)
            AirBrakeForce(i) = 0;
%             TargetForce = ActuatorAirBrakeNotch(i)/VectorBrake{1}*VectorBrake{2}(i);
%             if AirBrakeForce(i) > TargetForce
%                 temp = find(ABCL_Force{1}(2,:) >= AirBrakeForce(i));
%                 %             if isempty(temp)
%                 %                 pause(0.1);
%                 %             end
%                 AirBrakeForce(i) = AirBrakeForce(i)-ABCL_Force{1}(3,temp(end))*VectorBrake{3};
%             elseif AirBrakeForce(i) < TargetForce
%                 temp = find(ABCL_Force{2}(2,:) >= AirBrakeForce(i),1);
%                 AirBrakeForce(i) = AirBrakeForce(i)+ABCL_Force{2}(3,temp)*VectorBrake{3};
%             end
        else
            TargetForce = ActuatorAirBrakeNotch(i)/VectorBrake{1}*VectorBrake{2}(i);
            if AirBrakeForce(i) > TargetForce
                temp = find(ABCL_Force{1}(2,:) >= AirBrakeForce(i));
                AirBrakeForce(i) = AirBrakeForce(i)-ABCL_Force{1}(3,temp(end))*VectorBrake{3};
            elseif AirBrakeForce(i) < TargetForce
                temp = find(ABCL_Force{2}(2,:) >= AirBrakeForce(i),1);
                AirBrakeForce(i) = AirBrakeForce(i)+ABCL_Force{2}(3,temp)*VectorBrake{3};
            end
        end
    end
end







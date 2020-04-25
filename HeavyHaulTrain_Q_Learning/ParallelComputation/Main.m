
%% This function is a revision for Watkin's Q(lambda) on velocity regulation
%% *LIMIT means not cover all the distance.
%% All the force list offer positive values. Brake force multiply negative(-) when used.
%% Group means connected by rigid bar.
%% Pneumatic brake systems choice is permitted.
%% Change to the discrete form for computer analysis.
%% The simulation step-size can be extended to 0.5 seconds. A's sepctral radius determins the convergence.
%% Curve Resistance do not matter.
%% Train make up can be changed adaptively.
%% Air brake is computed by car group. The script before calculate the force by single car.
%% Do not waste time on air brake force calculation if ActuatorAirBrakeNotch == 0 and AirBrakeForce = 0
%% Realize completely exploring start.

%% Optimize the in-train force.
%% Table look-up methods Q learning.

clc;
clear all

%% PART I:INITIALIZE THE TRAIN PARAMETERS
InitializeTheTrainParameters();

%% PART II:INITIALIZE Q Learning Parameters.
InitializeTheQLearningParameters();

%% Create the Action Number Table.
InitializeActionChangeTable();
InitializeActionNumberTable();

%% Build the foundation for the display of Velocity-Distance and the Q value.
BuildTheDisplay();

%% Determine a state-action pair.
CurrentState = [TrainVelocity(1,1) TrainDistance(1,1) 0 0 0];
CurrentStateFeature = GetStateFeature(NumOfGridDistance,ActionNumberTable,...
    GridVelocityMatrix,GridDistanceMatrix,CurrentState);
ActionValue = [CurrentStateFeature zeros(1,NumOfActions)];
ListRankCurrentState = find(ActionValue(:,1) == CurrentStateFeature,1);
[MaxActionValue,MaxActionValueIndice] = max(ActionValue(ListRankCurrentState,2:28));
CurrentActionIndice = MaxActionValueIndice;
CurrentAction = ActionSet(CurrentActionIndice,:);
CurrentNotch(1) = min(CurrentState(3)+CurrentAction(1),12);
CurrentNotch(1) = max(-12,CurrentNotch(1));
CurrentNotch(2) = min(CurrentState(4)+CurrentAction(2),12);
CurrentNotch(2) = max(-12,CurrentNotch(2));
CurrentNotch(3) = min(CurrentState(5)+CurrentAction(3),0);
CurrentNotch(3) = max(-2,CurrentNotch(3));
epsilons = linspace(0.01,1e-5,NumofEpisodes/2);
epsilons = [epsilons zeros(1,NumofEpisodes/2)];
% epsilons = zeros(1,NumofEpisodes);

for p = 1:NumofEpisodes
    %% Display the episodes by 1000.
    if ~rem(p,20)
        p
    end
    StepEndTimePoint = 0;
    
    %% Fixed starts
    if p > 1 % ¸³Öµ³õÊ¼µã
        %         TrainVelocity(:,1) = StartVelocity(mod(p,NumofStartVelocity)+1);
        %         TrainDistance(:,1) = StartDistance(mod(floor(p/NumofStartDistance)+1,NumofStartDistance)+1);
        CurrentState = [TrainVelocity(1,1) TrainDistance(1,1) 0 0 0];
        CurrentStateFeature = GetStateFeature(NumOfGridDistance,ActionNumberTable,...
            GridVelocityMatrix,GridDistanceMatrix,CurrentState);
        %         CurrentActionIndice = mod(floor(p/NumofStartPoint)+1,NumOfActions)+1;
        ListRankCurrentState = find(ActionValue(:,1) == CurrentStateFeature,1);
        [MaxActionValue,MaxActionValueIndice] = max(ActionValue(ListRankCurrentState,2:28));
        CurrentActionIndice = MaxActionValueIndice;
        CurrentAction = ActionSet(CurrentActionIndice,:);
        CurrentNotch(1) = min(CurrentState(3)+CurrentAction(1),12);
        CurrentNotch(1) = max(-12,CurrentNotch(1));
        CurrentNotch(2) = min(CurrentState(4)+CurrentAction(2),12);
        CurrentNotch(2) = max(-12,CurrentNotch(2));
        CurrentNotch(3) = min(CurrentState(5)+CurrentAction(3),0);
        CurrentNotch(3) = max(-2,CurrentNotch(3));
        %         CurrentActionSetValue = CurrentParameter'*CurrentStateFeature;
        %         [~,CurrentActionIndice] = max(CurrentActionSetValue);
        %         CurrentAction = ActionSet(CurrentActionIndice);
        %         CurrentNotch = min(CurrentNotch+CurrentAction,12);
        %         CurrentNotch = max(CurrentNotch,-10);
    end
    alpha = 0.8;
    epsilon = epsilons(p);
    RecordCurrentVelocity(1) = CurrentState(1);
    RecordCurrentDistance(1) = CurrentState(2);
    
    %% Enter into a new episode.
    for j = 1:NumofStepsPerEpisode
        CurrentTime = CurrentTime+1;
        
        if j > 1
            CurrentState = AfterState;
            CurrentStateFeature = AfterStateFeature;
            ListRankCurrentState = ListRankAfterState;
            MaxActionValue = AfterMaxActionValue;
            MaxActionValueIndice = AfterMaxActionValueIndice;
            CurrentNotch(1) = min(CurrentState(3)+CurrentAction(1),12);
            CurrentNotch(1) = max(-12,CurrentNotch(1));
            CurrentNotch(2) = min(CurrentState(4)+CurrentAction(2),12);
            CurrentNotch(2) = max(-12,CurrentNotch(2));
            CurrentNotch(3) = min(CurrentState(5)+CurrentAction(3),0);
            CurrentNotch(3) = max(-2,CurrentNotch(3));
        end
        
        %% Enter into a new step.
        for i = StepEndTimePoint+1:StepEndTimePoint+simulation_step-1
            %% Give the target velocity.
            %             if TrainDistance(1,i) > 1000
            %                 TargetTrainVelocity(i) = 60/3.6;
            %             else
            %                 TargetTrainVelocity(i) = 70/3.6;
            %             end
            TargetTrainVelocity(i) = 70/3.6;
            %% Indicator Calculation.
            VelocityError(i) = (TargetTrainVelocity(i)-TrainVelocity(:,i))'*(TargetTrainVelocity(i)-TrainVelocity(:,i));
            CouplerForcePerCoupler(:,i) = diag(KK)*(TrainDistance(1:end-1,i)-TrainDistance(2:end,i))+...
                diag(DD)*(TrainVelocity(1:end-1,i)-TrainVelocity(2:end,i));
            
            %% Get the train handling by the driver.
            [LocomotiveMotorNotch(:,i),AirBrakeNotch(:,i)] = GetTrainNotches(num_CarGroup,CurrentNotch);
            
            %% Over Speed Protection.
            if TrainVelocity(1,i)*3.6 >= 120
                AirBrakeSignal = 1;
            elseif TrainVelocity(1,i)*3.6 < 100
                AirBrakeSignal = 0;
            end
            
            if AirBrakeSignal == 1
                LocomotiveMotorNotch(:,i) = -12*ones(num_LocoGroup,1);
                AirBrakeNotch(:,i) = -2*ones(num_CarGroup,1);
            end
            
            %% Low Speed Limit
            if TrainVelocity(1,i)*3.6 <= 10
                LowSpeedSignal = 1;
            elseif TrainVelocity(1,i)*3.6 > 20
                LowSpeedSignal = 0;
            end
            
            if LowSpeedSignal == 1
                LocomotiveMotorNotch(:,i) = 12*ones(num_LocoGroup,1);
                AirBrakeNotch(:,i) = 0*ones(num_CarGroup,1);
            end
            
            %% Calculate the force of locomotive. kN
            LocomotiveMotorForce(:,i) = GetLocomotiveForce(TrainVelocity(position_locomotive,i),...
                LocomotiveMotorNotch(:,i),MatrixTBCL_force,TCL_force,BCL_force,num_LocoGroup,i);
            
            %% Calculate the Air Brake Force. kN
            if i > 1
                temp = find(AirBrakeNotch(:,i)~=AirBrakeNotch(:,i-1));
                if ~isempty(temp)
                    TimeFromAirBrakeSignal(temp) = 0;
                end
                temp = find(TimeFromAirBrakeSignal >= AirBrakeSignalTransferTime);
                if ~isempty(temp)
                    ActuatorAirBrakeNotch(temp,1) = ActuatorAirBrakeNotch(temp,2);
                    ActuatorAirBrakeNotch(temp,2) = AirBrakeNotch(temp,i);
                end
                TraceActuatorAirBrakeNotch(:,i) = ActuatorAirBrakeNotch(:,2);
                AirBrakeForce(:,i) = GetAirBrakeForce(ActuatorAirBrakeNotch(:,2),AirBrakeForce(:,i-1),...
                    VectorBrake,CarBrakeCoefficient);
            else
                temp = find(AirBrakeNotch(:,i) < 0);
                if ~isempty(temp)
                    TimeFromAirBrakeSignal(temp) = 0;
                end
                temp = find(TimeFromAirBrakeSignal >= AirBrakeSignalTransferTime);
                if ~isempty(temp)
                    ActuatorAirBrakeNotch(temp,1) = ActuatorAirBrakeNotch(temp,2);
                    ActuatorAirBrakeNotch(temp,2) = AirBrakeNotch(temp,i);
                end
                TraceActuatorAirBrakeNotch(:,i) = ActuatorAirBrakeNotch(:,2);
                AirBrakeForce(:,i) = GetAirBrakeForce(ActuatorAirBrakeNotch(:,2),AirBrakeForce(:,1),...
                    VectorBrake,CarBrakeCoefficient);
            end
            
            TimeFromAirBrakeSignal = TimeFromAirBrakeSignal+sampling_time;
            
            %% Calculate the addtional resistance. N
            [AdditionalResistance(:,i),RampResistance(:,i),CurveResistance(:,i)] = ...
                GetAdditionalResistance(mass_TrainGroup,TrainPosition(:,i),...
                RampPositionList,CurvePositionList,p);
            
            %% Force produced by train calculation. kN.
            TrainForce(:,i) = GetTrainForce(LocomotiveMotorForce(:,i),AirBrakeForce(:,i),...
                position_locomotive,position_wagon);
            
            %% Get Linearization Feedback Input. N
            LinearizationFeedbackTrainForce(:,i) = GetLinearizationFeedbackTrainForce(TrainForce(:,i),...
                mass_TrainGroup,TrainVelocity(:,i),C0,Ca,AdditionalResistance(:,i));
            
            %% Update the state variables.
            temp = A_discrete*[TrainVelocity(:,i);TrainDistance(:,i)]+...
                B_discrete*LinearizationFeedbackTrainForce(:,i);
            TrainVelocity(:,i+1) = temp(1:num_CarGroup);
            TrainDistance(:,i+1) = temp(num_CarGroup+1:2*num_CarGroup);
            TrainPosition(:,i+1) = TrainPosition(:,1)+TrainDistance(:,i+1);
            
            %% Overtake distance.
            if (TrainDistance(1,i+1)-TrainDistance(1,StepEndTimePoint+1) >= 300)||...
                    (TrainPosition(1,i+1)>= FullStepsPerEpisode*300)
                break;
            end
            
        end
        
        %% Update the parameter according to the reward.
        RealCurrentNotch = [LocomotiveMotorNotch(1,i) LocomotiveMotorNotch(2,i) AirBrakeNotch(5,i)];
        VelocityReward(j) = -0.5*sum(VelocityError(StepEndTimePoint+1:i));
        CouplerForceReward(j) = -0.25*10^(-10)*sum(sum(CouplerForcePerCoupler(:,StepEndTimePoint+1:i).*...
            CouplerForcePerCoupler(:,StepEndTimePoint+1:i)));
        if j > 1
            SmoothReward(j) = -(RealCurrentNotch-RecordCurrentNotch(j-1,:))*...
                (RealCurrentNotch-RecordCurrentNotch(j-1,:))'*100;
        else
            SmoothReward(j) = 0;
        end
        
        temp = LocomotiveMotorForce(:,StepEndTimePoint+1:i);
        temp(temp<0) = 0;
        EnergyReward(j) = -10^(-2)*sum(sum(temp.*temp));
        
        AirBrakeReward(j) = -10^(-4)*(sum(sum(AirBrakeForce(:,StepEndTimePoint+1:i).*...
            AirBrakeForce(:,StepEndTimePoint+1:i))));
        %         CurrentReward = VelocityReward(j)+CouplerForceReward(j)+SmoothReward(j);
        StepEndTimePoint = i;
        AfterState = [TrainVelocity(1,i) TrainDistance(1,i) RealCurrentNotch];
        
        %         RecordCurrentVelocity(j+1) = AfterState(1);
        %         RecordCurrentDistance(j+1) = AfterState(2);
        RecordCurrentNotch(j,1:3) = RealCurrentNotch;
        
        %         if  p == 7
        %             pause(0.1);
        %         end
        AfterStateFeature = GetStateFeature(NumOfGridDistance,ActionNumberTable,...
            GridVelocityMatrix,GridDistanceMatrix,AfterState);
        ListRankAfterState = find(ActionValue(:,1) == AfterStateFeature,1);
        if isempty(ListRankAfterState)
            ActionValue = [ActionValue;AfterStateFeature zeros(1,27)];
            ListRankAfterState = size(ActionValue,1);
        end
        [AfterMaxActionValue,AfterMaxActionValueIndice] = max(ActionValue(ListRankAfterState,2:28));
        temp = CurrentActionIndice+1;
        ActionValue(ListRankCurrentState,temp) = ActionValue(ListRankCurrentState,temp)+...
            alpha*(VelocityReward(j)+CouplerForceReward(j)+SmoothReward(j)+EnergyReward(j)+AirBrakeReward(j)...
            +RewardDiscount*AfterMaxActionValue-ActionValue(ListRankCurrentState,temp));
        
        
        %% End the episode when terminal is arrived.
        if TrainPosition(1,i+1)>= FullStepsPerEpisode*300
            break;
        end
        
        %% Choose next action.
        if epsilon > 0      % epsilon-greedy policy
            ps = [0:epsilon/NumOfActions:epsilon*(AfterMaxActionValueIndice-1)/NumOfActions...
                1-epsilon+epsilon*AfterMaxActionValueIndice/NumOfActions:epsilon/NumOfActions:1];
            CurrentActionIndice = find(ps > Seed(1,CurrentTime),1)-1;
        else                % greedy policy
            CurrentActionIndice = AfterMaxActionValueIndice;
        end
        
        CurrentAction = ActionSet(CurrentActionIndice,:);
        
        %% Plot the velocity at the real time.
        %         PlotRecordCurrentDistance(1:2:j*2-1) = RecordCurrentDistance(1:j);
        %         PlotRecordCurrentDistance(2:2:j*2) = RecordCurrentDistance(2:j+1);
        %         PlotRecordCurrentNotch(1:2:j*2-1) = RecordCurrentNotch(1:j);
        %         PlotRecordCurrentNotch(2:2:j*2) = RecordCurrentNotch(1:j);
        %         if ~rem(j,1)
        %             subplot(2,1,1)
        %             set(hplot,'XData',RecordCurrentDistance(1:j+1)/1000,'YData',RecordCurrentVelocity(1:j+1)*3.6);
        %             subplot(2,1,2)
        %             set(hplot1,'XData', PlotRecordCurrentDistance(1:j*2)/1000,'YData',PlotRecordCurrentNotch(1:j*2));
        %             drawnow;
        %         end
        %% Plot the -max_a Q(s,a) at the real time.
        %         if ~rem(j,2)
        %             for q = 1:PlotNumOfVelocity
        %                 for w = 1:PlotNumOfDistance
        %                     PlotActionSetValue = CurrentParameter'*PlotStateFeature(:,(q-1)*PlotNumOfDistance+w);
        %                     PlotActionValue(q,w) = max(PlotActionSetValue);
        %                 end
        %             end
        %             mesh(PlotVelocity1,PlotDistance1,-PlotActionValue);
        %             view([0 90]);
        %             drawnow;
        %             %                 pause(0.001);
        %         end
    end
    EpisodeReturn1(p) = sum(VelocityReward);
    EpisodeReturn2(p) = sum(CouplerForceReward);
    EpisodeReturn3(p) = sum(SmoothReward);
    EpisodeReturn4(p) = sum(EnergyReward);
    EpisodeReturn5(p) = sum(AirBrakeReward);
end

%% Plot -max_a Q(s,a).
% figure(21)
% for q = 1:PlotNumOfVelocity
%     for w = 1:PlotNumOfDistance
%         temp = find(ActionValue(:,1) == PlotStateFeature((q-1)*PlotNumOfDistance+w));
%         if ~isempty(temp)
%             PlotActionValue(q,w) = max(ActionValue(temp,2:28));
%         else
%             PlotActionValue(q,w) = 0;
%         end
%         %         PlotActionValue(q,w) = ActionValue(PlotStateFeature((q-1)*PlotNumOfDistance+w),1);
%     end
% end
% mesh(PlotVelocity1,PlotDistance1,-PlotActionValue);
% axis tight;
% 
% %% plot the total reward vary with more episode.
% figure(26)
% plot(-EpisodeReturn1-EpisodeReturn2-EpisodeReturn3-EpisodeReturn4-EpisodeReturn5,'color','b','LineWidth',2);
% hold on
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
% ylabel('Objective Function','FontSize',12,'Fontname','Times New Roman')
% axis tight;
% grid on
% 
% %% plot the velocity tracking error vary with more episode.
% figure(27)
% plot(-EpisodeReturn1,'color','b','LineWidth',2);
% hold on
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
% ylabel('Velocity Error Indicator','FontSize',12,'Fontname','Times New Roman')
% axis tight;
% grid on
% 
% %% plot the Coupler Force Per Coupler vary with more episode.
% figure(28)
% plot(-EpisodeReturn2,'color','b','LineWidth',2);
% hold on
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
% ylabel('In-train Force Iindicator (kN)^2','FontSize',12,'Fontname','Times New Roman')
% axis tight;
% grid on
% 
% %% plot the notch change vary with more episode.
% figure(40)
% plot(-EpisodeReturn3,'color','b','LineWidth',2);
% hold on
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
% ylabel('Driving Smooth Indicator','FontSize',12,'Fontname','Times New Roman')
% axis tight;
% grid on
% 
% %% plot the Energy Cost vary with more episode.
% figure(41)
% plot(-EpisodeReturn4,'color','b','LineWidth',2);
% hold on
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
% ylabel('Energy Consumption Indicator','FontSize',12,'Fontname','Times New Roman')
% axis tight;
% grid on
% 
% %% plot the Brake Cost vary with more episodes.
% figure(42)
% plot(-EpisodeReturn5,'color','b','LineWidth',2);
% hold on
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
% ylabel('Air Brake Indicator','FontSize',12,'Fontname','Times New Roman')
% axis tight;
% grid on
% 
% %% plot the velocity.
% figure(1)
% plot(TrainPosition(1,1:i)/1000,TrainVelocity(20,1:i)*3.6,'color','b','LineWidth',2);
% hold on
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Position of the first locomotive group(km)','FontSize',12,'Fontname','Times New Roman')
% ylabel('Velocity(km/h)','FontSize',12,'Fontname','Times New Roman')
% legend('Locomotive1')
% axis tight;
% grid on
% 
% %% plot the coupler force.(2-Dimensions)
% figure(2)
% plot(TrainPosition(1,2:i)/1000,CouplerForcePerCoupler(29,2:i)/1000,'color','g','LineWidth',2); % pull is +
% hold on
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Position of the first locomotive group(km)','FontSize',12,'Fontname','Times New Roman')
% ylabel('Coupler Force(kN)','FontSize',12,'Fontname','Times New Roman')
% %legend('LG1 and WG1','WG50 and LG2')
% axis tight;
% grid on
% 
% %% plot the coupler force.(3-Dimensions)
% temp = 1;
% [num_coupler,distance] = meshgrid(TrainPosition(1,1:temp:i)/1000,1:num_CarGroup-1);
% coupler_force = CouplerForcePerCoupler(:,1:temp:i)/1000;
% figure(3)
% mesh(num_coupler,distance,coupler_force);
% %mesh(num_coupler,distance,coupler_force);
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Position of the first locomotive(km)','FontSize',12,'Fontname','Times New Roman')
% ylabel('Coupler Number','FontSize',12,'Fontname','Times New Roman')
% zlabel('Coupler Force(kN)','FontSize',12,'Fontname','Times New Roman')
% %legend('LG1','LG2','Wagon')
% axis tight;
% grid on
% 
% %% plot the Notch.
% figure(18)
% plot(TrainPosition(1,1:i)/1000,LocomotiveMotorNotch(1,1:i),'color','b','LineWidth',2);
% hold on
% plot(TrainPosition(1,1:i)/1000,LocomotiveMotorNotch(2,1:i),'color','r','LineWidth',2);
% hold on
% plot(TrainPosition(1,1:i)/1000,AirBrakeNotch(5,1:i),'color','g','LineWidth',2);
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Position of the first locomotive(km)','FontSize',12,'Fontname','Times New Roman')
% ylabel('Train handling','FontSize',12,'Fontname','Times New Roman')
% legend('LG1','LG2','Wagon')
% axis tight;
% grid on
% 
% %% plot the Force.
% figure(19)
% plot(TrainPosition(1,1:i)/1000,sum(LocomotiveMotorForce(1,1:i)),'color','b','LineWidth',2);
% hold on
% plot(TrainPosition(1,1:i)/1000,sum(LocomotiveMotorForce(2,1:i)),'color','r','LineWidth',2);
% hold on
% plot(TrainPosition(1,1:i)/1000,sum(AirBrakeForce(:,1:i)),'color','g','LineWidth',2);
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Position of the first locomotive(km)','FontSize',12,'Fontname','Times New Roman')
% ylabel('Train Force(kN)','FontSize',12,'Fontname','Times New Roman')
% legend('LG1','LG2','Wagon')
% axis tight;
% grid on
% 
% %% plot the Force with respect to Time.
% figure(31)
% tt = 1:i;
% plot(tt*0.5,LocomotiveMotorForce(1,1:i),'color','b','LineWidth',2);
% hold on
% plot(tt*0.5,LocomotiveMotorForce(5,1:i),'color','r','LineWidth',2);
% hold on
% plot(tt*0.5,AirBrakeForce(5,1:i),'color','g','LineWidth',2);
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Position of the first locomotive(km)','FontSize',12,'Fontname','Times New Roman')
% ylabel('Train Force(kN)','FontSize',12,'Fontname','Times New Roman')
% legend('LG1','LG2','Wagon')
% axis tight;
% grid on
% 
% %% plot the Ramp.
% figure(13)
% % subplot(2,1,1)
% plot(RampPoint(:,1)/1000,RampPoint(:,2),'color','b','LineWidth',2);
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Position of the first locomotive(km)','FontSize',12,'Fontname','Times New Roman')
% ylabel('Height(m)','FontSize',12,'Fontname','Times New Roman')
% axis([0 TrainPosition(1,i+1)/1000 -30 10]);
% grid on;
% 
% %% plot the Reward Ratio at the final episode.
% figure(29)
% plot(EpisodeReturn1./(EpisodeReturn1+EpisodeReturn2+EpisodeReturn3+EpisodeReturn4+EpisodeReturn5),'color','b','LineWidth',2);
% hold on
% plot(EpisodeReturn2./(EpisodeReturn1+EpisodeReturn2+EpisodeReturn3+EpisodeReturn4+EpisodeReturn5),'color','r','LineWidth',2);
% hold on
% plot(EpisodeReturn3./(EpisodeReturn1+EpisodeReturn2+EpisodeReturn3+EpisodeReturn4+EpisodeReturn5),'color','g','LineWidth',2);
% hold on
% plot(EpisodeReturn4./(EpisodeReturn1+EpisodeReturn2+EpisodeReturn3+EpisodeReturn4+EpisodeReturn5),'color','y','LineWidth',2);
% hold on
% plot(EpisodeReturn5./(EpisodeReturn1+EpisodeReturn2+EpisodeReturn3+EpisodeReturn4+EpisodeReturn5),'color','k','LineWidth',2);
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
% ylabel('Objective Indicator Ratio','FontSize',12,'Fontname','Times New Roman')
% legend('Velocity','CouplerForce','Smooth','Energy','Air Brake')
% axis tight;
% grid on
















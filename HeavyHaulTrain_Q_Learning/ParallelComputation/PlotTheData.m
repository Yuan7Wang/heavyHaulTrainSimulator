%% plot the data
%% plot the total reward vary with more episode.
total = -EpisodeReturn1-EpisodeReturn2-EpisodeReturn3-EpisodeReturn4-EpisodeReturn5;
temp = max(total)-min(total);
figure(1)
subplot(3,2,1)
plot(total,'color','b','LineWidth',2);
hold on
% set(gca,'yscale','log');
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
ylabel('Objective Function','FontSize',12,'Fontname','Times New Roman')
axis ([0 length(total) min(total)-temp*0.1 max(total)+temp*0.05]);
grid on

%% plot the velocity tracking error vary with more episode.
total = -EpisodeReturn1;
temp = max(total)-min(total);
subplot(3,2,2)
plot(total,'color','b','LineWidth',2);
hold on
% set(gca,'yscale','log');
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
ylabel('Velocity Error Indicator','FontSize',12,'Fontname','Times New Roman')
axis ([0 length(total) min(total)-temp*0.1 max(total)+temp*0.05]);
grid on

%% plot the Coupler Force Per Coupler vary with more episode.
total = -EpisodeReturn2;
temp = max(total)-min(total);
subplot(3,2,3)
plot(total,'color','b','LineWidth',2);
hold on
% set(gca,'yscale','log');
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
ylabel('In-train Force Iindicator (kN)^2','FontSize',12,'Fontname','Times New Roman')
axis ([0 length(total) min(total)-temp*0.1 max(total)+temp*0.05]);
grid on

%% plot the notch change vary with more episode.
total = -EpisodeReturn3;
temp = max(total)-min(total);
subplot(3,2,4)
plot(total,'color','b','LineWidth',2);
hold on
% set(gca,'yscale','log');
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
ylabel('Driving Smooth Indicator','FontSize',12,'Fontname','Times New Roman')
axis ([0 length(total) min(total)-temp*0.1 max(total)+temp*0.05]);
grid on

%% plot the Loco force indicator
total = -EpisodeReturn4;
temp = max(total)-min(total);
subplot(3,2,5)
plot(total,'color','b','LineWidth',2);
hold on
% set(gca,'yscale','log');
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
ylabel('Traction Energy Indicator','FontSize',12,'Fontname','Times New Roman')
axis ([0 length(total) min(total)-temp*0.1 max(total)+temp*0.05]);
grid on

%% plot the Brake Cost vary with more episodes.
total = -EpisodeReturn5;
temp = max(total)-min(total);
subplot(3,2,6)
plot(total,'color','b','LineWidth',2);
hold on
% set(gca,'yscale','log');
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Episode','FontSize',12,'Fontname','Times New Roman')
ylabel('Air Brake Indicator','FontSize',12,'Fontname','Times New Roman')
axis ([0 length(total) min(total)-temp*0.1 max(total)+temp*0.05]);
grid on


figure(2)
%% plot the Ramp.
subplot(3,2,1)
% subplot(2,1,1)
plot(RampPoint(:,1)/1000,RampPoint(:,2),'color','b','LineWidth',2);
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Position of the first locomotive(km)','FontSize',12,'Fontname','Times New Roman')
ylabel('Height(m)','FontSize',12,'Fontname','Times New Roman')
axis([0 TrainPosition(1,i+1)/1000 -30 10]);
grid on;

%% plot the velocity.
subplot(3,2,2)
plot(TrainPosition(1,1:i)/1000,TrainVelocity(1,1:i)*3.6,'color','b','LineWidth',2);
hold on
plot(TrainPosition(1,1:i)/1000,TrainVelocity(30,1:i)*3.6,'color','r','LineWidth',2);
hold on
plot(TrainPosition(1,1:i)/1000,TrainVelocity(56,1:i)*3.6,'color','g','LineWidth',2);
hold on
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Position of the first locomotive group(km)','FontSize',12,'Fontname','Times New Roman')
ylabel('Velocity(km/h)','FontSize',12,'Fontname','Times New Roman')
legend('Car1','Car30','Car56')
axis tight;
grid on

%% plot the Notch.
subplot(3,2,3)
plot(TrainPosition(1,1:i)/1000,LocomotiveMotorNotch(1,1:i),'color','b','LineWidth',2);
hold on
plot(TrainPosition(1,1:i)/1000,LocomotiveMotorNotch(5,1:i),'color','r','LineWidth',2);
hold on
plot(TrainPosition(1,1:i)/1000,AirBrakeNotch(5,1:i),'color','g','LineWidth',2);
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Position of the first locomotive(km)','FontSize',12,'Fontname','Times New Roman')
ylabel('Train handling','FontSize',12,'Fontname','Times New Roman')
legend('LG1','LG2','Wagon')
axis tight;
grid on

%% plot the Force.
subplot(3,2,4)
plot(TrainPosition(1,1:i)/1000,sum(LocomotiveMotorForce(1:4,1:i)),'color','b','LineWidth',2);
hold on
plot(TrainPosition(1,1:i)/1000,sum(LocomotiveMotorForce(5:6,1:i)),'color','r','LineWidth',2);
hold on
plot(TrainPosition(1,1:i)/1000,sum(AirBrakeForce(:,1:i)),'color','g','LineWidth',2);
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Position of the first locomotive(km)','FontSize',12,'Fontname','Times New Roman')
ylabel('Train Force(kN)','FontSize',12,'Fontname','Times New Roman')
legend('LG1','LG2','Wagon')
axis tight;
grid on

%% plot the coupler force.(3-Dimensions) 
% title('Coupler Force Distribution')
temp = 1;
[num_coupler,distance] = meshgrid(TrainPosition(1,1:temp:i)/1000,1:num_CarGroup-1);
coupler_force = CouplerForcePerCoupler(:,1:temp:i)/1000;
subplot(3,2,5)
mesh(num_coupler,distance,coupler_force);
%mesh(num_coupler,distance,coupler_force);
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Position of the first locomotive(km)','FontSize',12,'Fontname','Times New Roman')
ylabel('Coupler Number','FontSize',12,'Fontname','Times New Roman')
zlabel('Coupler Force(kN)','FontSize',12,'Fontname','Times New Roman')
%legend('LG1','LG2','Wagon')
axis tight;
grid on
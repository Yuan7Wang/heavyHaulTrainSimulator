clc;
clear all
%% plot a circle.
% rectangle('Position',[0,0,1,1],'Curvature',[1,1]),axis tight
% grid on
%% load the data from the graph.
load('AirCharacteristic');
load('ReleaseCharacteristic');
AirCharacteristic(1,1) = 0;
AirCharacteristic(1,2) = 0;
AirCharacteristic(end,1) = 180;
AirCharacteristic(end,2) = 100;
ReleaseCharacteristic(1,1) = 0;
ReleaseCharacteristic(1,2) = 0;
ReleaseCharacteristic(end,1) = 540;
ReleaseCharacteristic(end,2) = 100;
% plot(AirCharacteristic(:,1),AirCharacteristic(:,2),ReleaseCharacteristic(:,1),ReleaseCharacteristic(:,2));
AirCharacteristic = AirCharacteristic';
ReleaseCharacteristic = ReleaseCharacteristic';
%%
ABCL_Force = cell(1,2);
k = (AirCharacteristic(2,2:end)-AirCharacteristic(2,1:end-1))./...
    (AirCharacteristic(1,2:end)-AirCharacteristic(1,1:end-1));
AirCharacteristic(3,:) = [k 0];
AirCharacteristic(2,:) = -AirCharacteristic(2,:);
ABCL_Force{1} = AirCharacteristic;

k = (ReleaseCharacteristic(2,2:end)-ReleaseCharacteristic(2,1:end-1))./...
    (ReleaseCharacteristic(1,2:end)-ReleaseCharacteristic(1,1:end-1));
ReleaseCharacteristic(3,:) = [0 k]; 
ReleaseCharacteristic(2,:) = ReleaseCharacteristic(2,:)-100;
ABCL_Force{2} = ReleaseCharacteristic;
plot(ABCL_Force{1}(1,:),ABCL_Force{1}(2,:),ABCL_Force{2}(1,:),ABCL_Force{2}(2,:))
set(gca,'FontSize',12,'Fontname','Times New Roman');
%%
xlabel('Brake or Release Time(seconds)','FontSize',12,'Fontname','Times New Roman')
ylabel('Air Brake Force(kN)','FontSize',12,'Fontname','Times New Roman')
legend('Brake Curve','Release Curve');
% save('ABCL_Force','ABCL_Force');










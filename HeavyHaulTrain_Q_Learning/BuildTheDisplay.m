%% Build the Display.

%% Variables for Plot the -max_a Q(s,a).
Resolution1 = 0.1;
Resolution2 = 100;
PlotVelocity = InfimumOfVelocity:Resolution1:SupremumOfVelocity;
PlotDistance = InfimumOfDistance:Resolution2:SupremumOfDistance;
PlotNumOfVelocity = length(PlotVelocity);
PlotNumOfDistance = length(PlotDistance);
PlotActionValue = zeros(PlotNumOfVelocity,PlotNumOfDistance);
for q = 1:PlotNumOfVelocity
    for w = 1:PlotNumOfDistance
        PlotState = [PlotVelocity(q) PlotDistance(w) 0 0 0];
        PlotStateFeature((q-1)*PlotNumOfDistance+w) = GetStateFeature(...
            NumOfGridDistance,ActionNumberTable,GridVelocityMatrix,GridDistanceMatrix,PlotState);
    end
end
[PlotVelocity1,PlotDistance1] = meshgrid(PlotDistance,3.6*PlotVelocity);

%% Variables for Plot the velocity-distance curve.
temp = [InfimumOfDistance SupremumOfDistance];
figure(100)
clf;
subplot(2,1,1)
plot(temp/1000,SupremumOfVelocity*3.6*ones(1,length(temp)),'color','r','LineWidth',2);
hold on
hplot = plot(0,70,'color','b','LineWidth',2);
subplot(2,1,2)
plot(temp/1000,0,'color','r','LineWidth',2);
hold on
hplot1 = plot(0,0,'color','b','LineWidth',2);
% axis([temp(1) temp(end) 0 110]);
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Position of the first locomotive group(km)','FontSize',12,'Fontname','Times New Roman')
% ylabel('Velocity(km/h)','FontSize',12,'Fontname','Times New Roman')
% grid on


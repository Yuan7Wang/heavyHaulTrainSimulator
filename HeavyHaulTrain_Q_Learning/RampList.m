%% Generate Ramp List by change point.
clc;
clear all
load('RampGIS')

%% The data identified by eyes.
% RampPoint = [12000 1920
%              12500 1920
%              16000 1875
%              18000 1885
%              21100 1875
%              25000 1810
%              30000 1750
%              34000 1690
%              37000 1680];

%% The data identified by computer.
%% Ramp small change.
RampPoint = [RampGIS(:,1)*1000 RampGIS(:,2)];
% RampPoint(2,2) = RampPoint(1,2);
% RampPoint(3,2) = RampPoint(3,2)+0.07*(RampPoint(2,2)-RampPoint(3,2));
% RampPoint(4,2) = RampPoint(4,2)-0.25*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(5,2) = RampPoint(5,2)-0.25*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(6,2) = RampPoint(6,2)-0.1*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(7,2) = RampPoint(7,2)-0.1*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(9,2) = RampPoint(9,2)+0.1*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(10,2) = RampPoint(10,2)+0.1*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(11,2) = RampPoint(11,2)+0.1*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(13,2) = RampPoint(13,2)+0.3*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(14,2) = RampPoint(14,2)-0.3*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(15,2) = RampPoint(15,2)+0.3*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(16,2) = RampPoint(16,2)-0.3*(RampPoint(5,2)-RampPoint(4,2));
% RampPoint(18,2) = RampPoint(18,2)+0.3*(RampPoint(5,2)-RampPoint(4,2));
for i = 1:length(RampPoint)-1   
RampPositionList(i,:) = [RampPoint(i,1) RampPoint(i+1,1)...
                        (RampPoint(i+1,2)-RampPoint(i,2))/(RampPoint(i+1,1)-RampPoint(i,1))];                
end

%% include all the possible position in this list.
RampPositionList = [-10000 12000 0.002;RampPositionList;RampPoint(end,1) 100000 0];
RampPoint = [9000 1923;RampPoint];
% save('RampPositionList','RampPositionList');
% save('RampPoint','RampPoint');






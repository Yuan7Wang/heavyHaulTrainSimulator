clc;
clear all

RampPositionList(:,1) = 0:500:30000;
RampPositionList(4:end,1) = RampPositionList(4:end,1)-400;
RampPositionList(7:end,1) = RampPositionList(7:end,1)-200;
RampPositionList(10:end,1) = RampPositionList(10:end,1)-200;
RampPositionList(:,2) = [RampPositionList(2:end,1);30800];

RampPositionList(1,3) = -0.01;
RampPositionList(2,3) = -0.006;
RampPositionList(3,3) = 0;
RampPositionList(4,3) = 0.008;
RampPositionList(5,3) = 0.01;
RampPositionList(6,3) = 0.002;
RampPositionList(7,3) = 0.0004;
RampPositionList(8,3) = -0.006;
RampPositionList(9,3) = -0.007;
RampPositionList(10,3) = 0;
RampPositionList(11,3) = 0.004;
RampPositionList(12,3) = 0.005;
RampPositionList(13,3) = 0;
RampPositionList(14,3) = -0.001;
RampPositionList(15,3) = -0.003;
RampPositionList(16,3) = 0.004;
RampPositionList(17,3) = 0.008;
RampPositionList(18,3) = 0;
RampPositionList(19,3) = -0.005;
RampPositionList(20,3) = 0;
RampPositionList(21,3) = 0.005;
RampPositionList(22,3) = 0;
RampPositionList(23,3) = -0.007;
RampPositionList(24,3) = -0.009;
RampPositionList(25,3) = -0.005;
RampPositionList(26,3) = 0;
RampPositionList(27,3) = 0.008;
RampPositionList(28,3) = 0.004;
RampPositionList(29,3) = 0;
RampPositionList(30,3) = -0.009;
RampPositionList(31,3) = -0.003;
RampPositionList(32,3) = 0;
RampPositionList(33,3) = 0.004;
RampPositionList(34,3) = -0.003;
RampPositionList(35,3) = 0;
RampPositionList(36,3) = 0.004;

RampPositionList = [-10000 0 0;RampPositionList;RampPositionList(end,2) 100000 0];
VectorDeltaHeight = (RampPositionList(:,2)-RampPositionList(:,1)).*RampPositionList(:,3);
RampPoint(:,1) = [RampPositionList(:,1);RampPositionList(end,2)];
RampPoint(1,2) = 0;
NumRampPoint = size(RampPoint,1);
for i = 1:NumRampPoint-1
  RampPoint(i+1,2) = RampPoint(i,2)+VectorDeltaHeight(i);
end
plot(RampPoint(:,1),RampPoint(:,2));
axis([0 6000 -20 10])
save('RampPoint','RampPoint');
save('RampPositionList','RampPositionList');















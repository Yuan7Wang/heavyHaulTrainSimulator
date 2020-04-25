clc;
clear all
load('LocoNotch1');
load('LocoNotch2');
load('WagonNotch');
LocoNotch1 = [LocoNotch1(:,1)*1000 LocoNotch1(:,2)];
LocoNotch2 = [LocoNotch2(:,1)*1000 LocoNotch2(:,2)];
WagonNotch = [WagonNotch(:,1)*1000 WagonNotch(:,2)];
LocoNotch1 = round(LocoNotch1);
LocoNotch2 = round(LocoNotch2);

temp = (WagonNotch(1:2:end-1,2)+WagonNotch(2:2:end,2))/2;
WagonNotch(1:2:end-1,2) = temp;
WagonNotch(2:2:end,2) = temp;

temp = WagonNotch(:,2);
temp(temp>0) = 0;
WagonNotch(:,2) = temp;

temp = WagonNotch(:,1);
temp = round(temp);
WagonNotch(:,1) = temp;

WagonNotch(end,1) = 38000;

plot(LocoNotch1(:,1),LocoNotch1(:,2));
hold on
plot(LocoNotch2(:,1),LocoNotch2(:,2));
hold on
plot(WagonNotch(:,1),WagonNotch(:,2))


DrivingSequenceChangePoint1 = LocoNotch1(2:2:end,1);
DrivingSequenceChangePoint2 = LocoNotch2(2:2:end,1);
DrivingSequenceChangePoint3 = WagonNotch(2:2:end,1);
DrivingSequenceChangePoint = [12926 14398 17091 19269 20567 20766 24592 27584 30575 ...
    32561 34818 36069 38000];

temp = [12000 DrivingSequenceChangePoint];
num_temp = length(temp)-1;
temp = (temp(1:end-1)+temp(2:end))/2;
for i = 1:num_temp
   LocoNotchIndice1 = find(LocoNotch1 > temp(i),1);
   LocoNotchIndice2 = find(LocoNotch2 > temp(i),1);
   WagonNotchIndice = find(WagonNotch > temp(i),1);
   LocoNotchSequence1(i) = LocoNotch1(LocoNotchIndice1,2);
   LocoNotchSequence2(i) = LocoNotch2(LocoNotchIndice2,2);
   WagonNotchSequence(i) = WagonNotch(WagonNotchIndice,2);
end
LocoNotchSequence1
LocoNotchSequence2
WagonNotchSequence







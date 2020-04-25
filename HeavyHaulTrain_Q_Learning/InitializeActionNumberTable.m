NotchSet = 12:-1:-12;
NumofNotches = length(NotchSet);
AirNotchSet = [0 -1 -2];
NumofAirNotches = length(AirNotchSet);
NumofTotalRow = NumofNotches*NumofNotches*NumofAirNotches;
ActionNumberTable = zeros(NumofTotalRow,3);

ActionNumberTable(1:3:NumofTotalRow-2,3) = AirNotchSet(1);
ActionNumberTable(2:3:NumofTotalRow-1,3) = AirNotchSet(2);
ActionNumberTable(3:3:NumofTotalRow,3) = AirNotchSet(3);

ActionNumberTable(1:3:NumofNotches*NumofAirNotches-2,2) = NotchSet;
ActionNumberTable(2:3:NumofNotches*NumofAirNotches-1,2) = NotchSet;
ActionNumberTable(3:3:NumofNotches*NumofAirNotches,2) = NotchSet;

for i = 2:NumofNotches
   ActionNumberTable((i-1)*NumofNotches*NumofAirNotches+1:i*NumofNotches*NumofAirNotches,2) = ...
       ActionNumberTable(1:NumofNotches*NumofAirNotches,2);
end
% plot(ActionNumberTable(:,2))
for i = 1:NumofNotches
   ActionNumberTable((i-1)*NumofNotches*NumofAirNotches+1:i*NumofNotches*NumofAirNotches,1) = ...
       NotchSet(i)*ones(NumofNotches*NumofAirNotches,1);
end

for i = 1:10
ActionNumberTable(ActionNumberTable(:,1) == -i,1) = 30+i;
ActionNumberTable(ActionNumberTable(:,2) == -i,2) = 30+i;
end
ActionNumberTable(:,3) = -ActionNumberTable(:,3);
ActionNumberTable = ActionNumberTable(:,1)*1000+ActionNumberTable(:,2)*10+ActionNumberTable(:,3);

[b,m,n] = unique(ActionNumberTable);








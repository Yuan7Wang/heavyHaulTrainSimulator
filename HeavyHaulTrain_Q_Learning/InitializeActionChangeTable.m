ActionSet = zeros(27,3);
NumOfActions = size(ActionSet,1);

ActionSet(1:3:NumOfActions-2,3) = 1;
ActionSet(2:3:NumOfActions-1,3) = 0;
ActionSet(3:3:NumOfActions,3) = -1;

Vector = [1:3 10:12 19:21];
ActionSet(Vector,2) = 1;
Vector = [4:6 13:15 22:24];
ActionSet(Vector,2) = 0;
Vector = [7:9 16:18 25:27];
ActionSet(Vector,2) = -1;

Vector = 1:9;
ActionSet(Vector,1) = 1;
Vector = 10:18;
ActionSet(Vector,1) = 0;
Vector = 19:27;
ActionSet(Vector,1) = -1;

% plot(ActionSet(:,3))


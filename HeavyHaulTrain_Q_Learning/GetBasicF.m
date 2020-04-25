function F = GetBasicF(mTrainGroup, TrainVelocity, C0, Ca)

F = -mTrainGroup.*C0;
F(1, :) = F(1, :)-Ca(1, :).*sum(mTrainGroup).*TrainVelocity(1, :).^2;
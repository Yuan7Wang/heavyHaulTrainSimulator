
%%
function BasicResistance = GetBasicResistance(mass_TrainGroup,c0_locomotive,cv_locomotive,ca_locomotive,...
    c0_wagon,cv_wagon,ca_wagon,TrainVelocity,position_locomotive,position_wagon)
C0(position_locomotive) = c0_locomotive;
C0(position_wagon) = c0_wagon;
Cv(position_locomotive) = cv_locomotive;
Cv(position_wagon) = cv_wagon;
Ca(position_locomotive) = ca_locomotive;
Ca(position_wagon) = ca_wagon;
BasicResistance = (C0'+Cv'.*TrainVelocity).*mass_TrainGroup';
BasicResistance(1) = BasicResistance(1)+ca_locomotive*sum(mass_TrainGroup)*TrainVelocity(1)^2;


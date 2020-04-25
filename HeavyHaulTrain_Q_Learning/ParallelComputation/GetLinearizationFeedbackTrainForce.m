function LinearizationFeedbackTrainForce = GetLinearizationFeedbackTrainForce(TrainForce,...
        mass_TrainGroup,TrainVelocity,C0,Ca,AdditionalResistance)
    
LinearizationFeedbackTrainForce = TrainForce*10^3-mass_TrainGroup'.*C0'-AdditionalResistance;
LinearizationFeedbackTrainForce(1) = LinearizationFeedbackTrainForce(1)-Ca(1)*sum(mass_TrainGroup)*TrainVelocity(1)^2;
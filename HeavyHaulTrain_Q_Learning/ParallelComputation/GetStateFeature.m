function StateFeature = GetStateFeature(NumOfGridDistance,ActionNumberTable,...
    GridVelocityMatrix,GridDistanceMatrix,State)
                                        
GridVelocityMatrix(GridVelocityMatrix < State(1)) = -2;
GridVelocityMatrix(GridVelocityMatrix > -1) = 0;
GridVelocityMatrix(GridVelocityMatrix < -1) = 1;
GridIndiceVelocity = sum(GridVelocityMatrix,2);

GridDistanceMatrix(GridDistanceMatrix < State(2)) = -2;
GridDistanceMatrix(GridDistanceMatrix > -1) = 0;
GridDistanceMatrix(GridDistanceMatrix < -1) = 1;
GridIndiceDistance = sum(GridDistanceMatrix,2);

StateFeature1 = (GridIndiceVelocity-1)*NumOfGridDistance+GridIndiceDistance;

if State(3) < 0
    State(3) = 30-State(3);
end
if State(4) < 0
    State(4) = 30-State(4);
end
State(5) = -State(5);

StateFeature2 = find(ActionNumberTable == State(3)*1000+State(4)*10+State(5));

StateFeature = StateFeature1*10000+StateFeature2;


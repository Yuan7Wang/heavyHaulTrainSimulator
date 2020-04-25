function [A_continuous,B_continuous] = TransParameterToStateSpace(mass_TrainGroup,num_CarGroup,KK,DD,Cv)

temp1 = diag(1./mass_TrainGroup);
NumKK = length(KK);
NumDD = length(DD);

%% Computation for A_continuous.
A_continuous(1:num_CarGroup,1:num_CarGroup) = -diag(Cv);
A_continuous(num_CarGroup+1:2*num_CarGroup,1:num_CarGroup) = eye(num_CarGroup);
A_continuous(num_CarGroup+1:2*num_CarGroup,num_CarGroup+1:2*num_CarGroup) = zeros(num_CarGroup);
temp = zeros(NumKK+1);
temp(1:end-1,2:end) = diag(KK);
temp(1:end-1,1:end-1) = temp(1:end-1,1:end-1)-diag(KK);
temp(2:end,:) = temp(2:end,:)-temp(1:end-1,:);
A_continuous(1:num_CarGroup,num_CarGroup+1:2*num_CarGroup) = temp1*temp;
temp = zeros(NumDD+1);
temp(1:end-1,2:end) = diag(DD);
temp(1:end-1,1:end-1) = temp(1:end-1,1:end-1)-diag(DD);
temp(2:end,:) = temp(2:end,:)-temp(1:end-1,:);
A_continuous(1:num_CarGroup,1:num_CarGroup) = A_continuous(1:num_CarGroup,1:num_CarGroup)+temp1*temp;

%% Computation for B_continuous.
B_continuous(1:num_CarGroup,1:num_CarGroup) = temp1;
B_continuous(num_CarGroup+1:2*num_CarGroup,1:num_CarGroup) = 0;

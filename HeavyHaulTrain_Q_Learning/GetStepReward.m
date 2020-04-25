function StepReward = GetStepReward()
StepReward = [1 1 1 0;1 1 2 0;1 1 3 0;2 1 1 0;2 1 2 0;2 1 3 0;...
    2 2 4 0;2 2 5 0;2 2 6 0;2 3 7 0;2 3 8 0;2 3 9 0;3 1 1 0;3 1 2 0;...
    3 1 3 0;3 2 4 0;3 2 5 0;3 2 6 0;3 3 7 0;3 3 8 0;3 3 9 0;3 4 10 0;...
    3 4 11 0;3 4 12 0;3 5 13 0;3 5 14 0;3 5 15 0;3 6 16 0;3 6 17 0;3 6 18 0;...
    3 7 19 0;3 7 20 0;3 7 21 0;3 8 22 0;3 8 23 0;3 8 24 0;3 9 25 0;3 9 26 0;...
    3 9 27 0];
StepReward = [StepReward zeros(size(StepReward,1),1)];

i = 1;
tempStart = 3^(i-1);
tempEnd = 3^i;
StepReward(tempStart:tempEnd,5) = [4;5;6];
for i = 2:3
    for j = tempStart:tempEnd
        if j == tempStart
            temp = [StepReward(j,5)-1;StepReward(j,5);StepReward(j,5)+1];
        else
            temp = [temp;StepReward(j,5)-1;StepReward(j,5);StepReward(j,5)+1];
        end       
    end
    
    tempStart = tempEnd+1;
    tempEnd = tempEnd+3^i;

    StepReward(tempStart:tempEnd,5) = temp;
end



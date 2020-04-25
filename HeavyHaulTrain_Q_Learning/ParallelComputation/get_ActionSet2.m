function ActionSet = get_ActionSet2()

ActionSet = zeros(9,2);

count = 0;
count = count+1; ActionSet(count, :) = [0 0]; 
count = count+1; ActionSet(count, :) = [0 -1]; 
count = count+1; ActionSet(count, :) = [0 1]; 
count = count+1; ActionSet(count, :) = [-1 0]; 
count = count+1; ActionSet(count, :) = [-1 -1]; 
count = count+1; ActionSet(count, :) = [-1 1]; 
count = count+1; ActionSet(count, :) = [1 0]; 
count = count+1; ActionSet(count, :) = [1 -1]; 
count = count+1; ActionSet(count, :) = [1 1]; 


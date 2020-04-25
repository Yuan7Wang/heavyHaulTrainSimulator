% tile coding for locaiton-speed space

alpha = 0.2;  % learning rate 

%% Parameters

detaS = 200; % unit m
detaV = 20;  % unit km/h
nTiling = 10;  % number of tiling 
tile_rand = rand(nTiling, 2).*(ones(nTiling, 1)*[detaS detaV]);
% % input range(S) = [0 2000]; range(V) = [0 120];

maxS = 2000; % unit m
maxV = 120;  % unit km/h

nS = ceil(maxS/detaS);
nV = ceil(maxV/detaV);

code = zeros(nS*nV, nTiling);

nl = 2;
W = rand(nS*nV*nTiling, 25^nl)';  % initialization 
%% 

W_obj.detaS = detaS; % unit m
W_obj.detaV = detaV;  % unit km/h

W_obj.tile_rand = tile_rand;
W_obj.W = W;
W_obj.alpha = alpha;

%% coding
p_num = 20;
current_state = rand(p_num, 2).*(ones(p_num, 1)*[maxS maxV]);
current_state = current_state';
current_ind_s = ceil((current_state(1, :) - tile_rand(:, 1))/detaS);
current_ind_v = ceil((current_state(2, :) - tile_rand(:, 2))/detaV);
current_ind_sv = current_ind_s*nS + current_ind_v + 1;

next_state = rand(p_num, 2).*(ones(p_num, 1)*[maxS maxV]);
next_state = next_state';
next_ind_s = ceil((next_state(1, :) - tile_rand(:, 1))/detaS);
next_ind_v = ceil((next_state(2, :) - tile_rand(:, 2))/detaV);
next_ind_sv = next_ind_s*nS + next_ind_v + 1;

action_ind = 10;  % current action 
current_r = -0.1; % current reward 

for i = 1:p_num
    
    for jtr = 1:nTiling
        code(current_ind_sv(jtr, i), jtr) = 1;
    end
    
    %% calculate current_action_value
    
    current_action_value = sum(W(action_ind, current_ind_sv(:, i)));
    
    
    %% estimate target_action_value
    
    target_action_value = current_r + max(sum(W(:, next_ind_sv(:, i)), 2));
    
    %% update the weight W
    
    W(action_ind, current_ind_sv(:, i)) = W(action_ind, current_ind_sv(:, i)) + ...
        alpha * (target_action_value - current_action_value);
    
end



% code = zeros(













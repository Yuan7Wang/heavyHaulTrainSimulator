% tile coding for locaiton-speed space
function W_obj = update_ActionFunction_linear(W_obj, Qloc, alpha)
%% input

% detaS = W_obj.detaS; % unit m
% detaV = W_obj.detaV;  % unit km/h

p_num = size(Qloc, 1);
current_state = Qloc(:, 1:2)';
action_ind = Qloc(:, 3);
next_state = Qloc(:, 4:5)';
current_r = Qloc(:, 6)';

%% coding
% current_state = rand(p_num, 2).*(ones(p_num, 1)*[maxS maxV]);
% current_state = current_state';
% current_ind_s = ceil((current_state(1, :) - W_obj.tile_rand(:, 1))/W_obj.detaS);
% current_ind_v = ceil((current_state(2, :) - W_obj.tile_rand(:, 2))/W_obj.detaV);

current_ind_sv = coding_index_transform(W_obj, current_state);

% current_ind_sv = current_ind_s*W_obj.nV + current_ind_v + 1;
% deta_ = W_obj.nS*W_obj.nV;
% current_ind_sv = current_ind_sv + (0:9)'*deta_*ones(1, p_num);

% next_state = rand(p_num, 2).*(ones(p_num, 1)*[maxS maxV]);
% next_state = next_state';

% next_ind_sv = coding_index_transform(W_obj, next_ind_s, next_ind_v);
next_ind_sv = coding_index_transform(W_obj, next_state);


% next_ind_sv = next_ind_s*W_obj.nV + next_ind_v + 1;
% next_ind_sv = next_ind_sv + (0:9)'*deta_*ones(1, p_num);

% action_ind = 10;  % current action 
% current_r = -0.1; % current reward 

%% 
for i = 1:p_num
    %% calculate current_action_value
    
    current_action_value = sum(W_obj.W(action_ind(i), current_ind_sv(:, i)));
    
    %% estimate target_action_value
    
    target_action_value = current_r(i) + max(sum(W_obj.W(get_possible_action_set(W_obj, action_ind(i)), next_ind_sv(:, i)), 2));
    
    %% update the weight W
    
    W_obj.W(action_ind(i), current_ind_sv(:, i)) = W_obj.W(action_ind(i), current_ind_sv(:, i)) + ...
        alpha * (target_action_value - current_action_value);
end













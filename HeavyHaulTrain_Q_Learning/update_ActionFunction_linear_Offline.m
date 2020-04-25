% tile coding for locaiton-speed space
function [W_obj, err_norm] = update_ActionFunction_linear_Offline(W_obj, Qloc, alpha, gamma)
%% input

% detaS = W_obj.detaS; % unit m
% detaV = W_obj.detaV;  % unit km/h

p_num = size(Qloc, 1);
current_state = Qloc(:, 1:2)';
action_ind = Qloc(:, 3);
next_state = Qloc(:, 4:5)';
current_r = Qloc(:, 6)';

%% coding

current_ind_sv = coding_index_transform(W_obj, current_state);

next_ind_sv = coding_index_transform(W_obj, next_state);

%%

% DTheta = zeros(W_obj.nTheta, p_num);

% dW1s = zeros(W_obj.nNodes, p_num);
% dW2s = zeros(p_num, W_obj.nNodes);
% db2s = zeros(1, p_num);
% 
% Current_action_value = zeros(1, p_num);
% Target_action_value = zeros(1, p_num);

% Detas = zeros(W_obj.nTheta, p_num);

err_ = zeros(1, p_num);
for i = 1:p_num
    %% current action (notch)
    %     notch = [10; 10];
%     N_ind = action_ind(i);
    
    current_action_value = sum(W_obj.W(action_ind(i), current_ind_sv(:, i)));
    
    %% estimate target_action_value
    if current_r(i) < 0
        notch_set = get_nearby_action_set(W_obj, action_ind(i));
        target_action_value = current_r(i) + gamma*max(sum(W_obj.W(notch_set, next_ind_sv(:, i)), 2));
    else
        target_action_value = -current_r(i);
    end
    %% update the weight W
    
    W_obj.W(action_ind(i), current_ind_sv(:, i)) = W_obj.W(action_ind(i), current_ind_sv(:, i)) + ...
        alpha * (target_action_value - current_action_value);
    
    err_(i) = target_action_value - current_action_value;
end

err_norm = norm(err_);

%% %% update the weight W

% % norm_ = norm(Detas);
% % Detas = Detas/norm_;
% 
% for i = 1:p_num
%     N_ind = action_ind(i); 
%     
%     W_obj.W1(:, current_ind_sv(:, i)) = W_obj.W1(:, current_ind_sv(:, i)) + alpha * repmat(dW1s(:, i), 1, W_obj.nTiling);
%     W_obj.W2(N_ind, :) =  W_obj.W2(N_ind, :) + alpha * dW2s(i, :);
%     W_obj.b1 = W_obj.b1 + alpha * dW1s(:, i);    % because dW1s is the same as db1s
%     W_obj.b2 = W_obj.b2 + alpha * db2s(:, i); 
% end













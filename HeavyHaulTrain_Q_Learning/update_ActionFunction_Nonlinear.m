% tile coding for locaiton-speed space
function W_obj = update_ActionFunction_Nonlinear(W_obj, W_obj_copy, Qloc, alpha)
%% input

% detaS = W_obj.detaS; % unit m
% detaV = W_obj.detaV;  % unit km/h

p_num = size(Qloc, 1);
current_state = Qloc(:, 1:2)';
action_ind = Qloc(:, 3:2+W_obj.nl)';
next_state = Qloc(:, 3+W_obj.nl:4+W_obj.nl)';
current_r = Qloc(:, 5+W_obj.nl)';

%% coding

current_ind_sv = coding_index_transform(W_obj, current_state);

next_ind_sv = coding_index_transform(W_obj, next_state);

%%

% DTheta = zeros(W_obj.nTheta, p_num);

dW1s = zeros(W_obj.nNodes, p_num);
dW2s = zeros(p_num, W_obj.nNodes);
db2s = zeros(1, p_num);

Current_action_value = zeros(1, p_num);
Target_action_value = zeros(1, p_num);

% Detas = zeros(W_obj.nTheta, p_num);

for i = 1:p_num
    %% current action (notch)
    %     notch = [10; 10];
    notch = action_ind(:, i);
    N_ind = W_obj.ind_t(W_obj, notch);
    
    %% calculate current_action_value
    % 使用新的 W_obj 来计算 target 值
    
    out1 = W_obj.cal_hidden_output(W_obj, current_ind_sv(:, i));
    current_action_value = W_obj.cal_final_output(W_obj, out1, N_ind);
    
%     current_theta = W_obj.getTheta(W_obj, current_ind_sv(:, i));
%     
%     current_action_value = W_obj.funTheta(W_obj, current_theta, notch);
    
    %% calculate gradient
    
%     gradient_ = W_obj.cal_gradient_Ntheta(W_obj, notch, current_theta);
    [dW1, dW2, db2] = W_obj.cal_gradient(W_obj, N_ind , out1);
    
    %% estimate target_action_value
    % 使用旧的 W_obj_copy 来计算 target 值
    
    [~, notch_set] = get_possible_notch_set(W_obj, notch);
    
    vals = W_obj.cal_final_output(W_obj_copy, W_obj.cal_hidden_output(W_obj_copy, next_ind_sv(:, i)), notch_set);
    
    target_action_value = current_r(i) + max(vals);
    
    %% 
    
    Current_action_value(1, i) = current_action_value;
    Target_action_value(1, i) = target_action_value;
    
    dW1s(:, i)  = (target_action_value - current_action_value)*dW1;
    dW2s(i, :)  = (target_action_value - current_action_value)*dW2;
    db2s(:, i)  = (target_action_value - current_action_value)*db2;
%     tmp = (target_action_value - current_action_value)*gradient_;
%     Detas(:, i) = tmp/norm(tmp);
end

%% %% update the weight W

% norm_ = norm(Detas);
% Detas = Detas/norm_;

for i = 1:p_num
    notch = action_ind(:, i);
    N_ind = W_obj.ind_t(W_obj, notch);
    
    W_obj.W1(:, current_ind_sv(:, i)) = W_obj.W1(:, current_ind_sv(:, i)) + alpha * repmat(dW1s(:, i), 1, W_obj.nTiling);
    W_obj.W2(N_ind, :) =  W_obj.W2(N_ind, :) + alpha * dW2s(i, :);
    W_obj.b1 = W_obj.b1 + alpha * dW1s(:, i);    % because dW1s is the same as db1s
    W_obj.b2 = W_obj.b2 + alpha * db2s(:, i); 
end













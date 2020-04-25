% tile coding for locaiton-speed space
%% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
%% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
%% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
%% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
%% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
function W_obj = update_ActionFunction_linear_Fit(W_obj, W_obj_copy, Qloc, alpha)
%% input

% detaS = W_obj.detaS; % unit m
% detaV = W_obj.detaV;  % unit km/h

p_num = size(Qloc, 1);
current_state = Qloc(:, 1:2)';
action_ind = Qloc(:, 3:4)';
next_state = Qloc(:, 5:6)';
current_r = Qloc(:, 7)';

%% coding

current_ind_sv = coding_index_transform(W_obj, current_state);

next_ind_sv = coding_index_transform(W_obj, next_state);

%%

DTheta = zeros(W_obj.nTheta, p_num);
Current_action_value = zeros(1, p_num);
Target_action_value = zeros(1, p_num);

Detas = zeros(W_obj.nTheta, p_num);

for i = 1:p_num
    %% current action (notch)
    %     notch = [10; 10];
    notch = action_ind(:, i);
    
    %% calculate current_action_value
    % 使用新的 W_obj 来计算 target 值
    
    current_theta = W_obj.getTheta(W_obj, current_ind_sv(:, i));
    
    current_action_value = W_obj.funTheta(W_obj, current_theta, notch);
    
    %% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
    %% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
    %% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
    %% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
    %% calculate gradient ERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERRORERROR
    
    dTheta = cal_gradient_Ntheta(W_obj, notch, current_theta);
    
    %% estimate target_action_value
    % 使用旧的 W_obj_copy 来计算 target 值
    
    next_theta = W_obj_copy.getTheta(W_obj_copy, next_ind_sv(:, i));
    
    notch_set = get_possible_notch_set(W_obj_copy, notch);
    
    vals = W_obj_copy.funTheta(W_obj_copy, next_theta, notch_set);
    
    target_action_value = current_r(i) + max(vals);
    
    %% 
    
    DTheta(:, i) = dTheta;
    Current_action_value(1, i) = current_action_value;
    Target_action_value(1, i) = target_action_value;
    tmp = (target_action_value - current_action_value)*dTheta;
    Detas(:, i) = tmp/norm(tmp);
end

%% %% update the weight W

% norm_ = norm(Detas);
% Detas = Detas/norm_;

for i = 1:p_num
    W_obj.W(:, current_ind_sv(:, i)) = W_obj.W(:, current_ind_sv(:, i)) + ...
        alpha * Detas(:, i);
end













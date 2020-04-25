function action_set = get_action_by_Wobj_linear_Fit(W_obj, current_state, current_action, epsilon)
%% coding 

current_ind_sv = coding_index_transform(W_obj, current_state);
action_set = zeros(size(current_action));

%% 
for i = 1:size(action_set, 2)
    
    notch = current_action(:, i);
    
    notch_set = get_possible_notch_set(W_obj, notch);
        
    if rand > epsilon    % ------ greedy
        current_theta = W_obj.getTheta(W_obj, current_ind_sv(:, i));
        vals = W_obj.funTheta(W_obj, current_theta, notch_set);
        [~, ind] = max(vals);
        ind = ind(1);
    else                 % ------ random
        ind = ceil(rand*size(notch_set, 2));
    end
    
    action_set(:, i) = notch_set(:, ind); 
end













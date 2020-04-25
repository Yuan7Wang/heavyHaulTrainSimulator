function action_set = get_action_by_Wobj_linear(W_obj, current_state, current_action, epsilon)
%% coding 

current_ind_sv = coding_index_transform(W_obj, current_state);
action_set = zeros(size(current_action));

%% 
for i = 1:size(action_set, 2)
    
    MotorNotch = current_action(:, i);
    
    a_101 = [-1; 0; 1];
    
    a = a_101 + MotorNotch(1) + W_obj.nNegNotch + 1;
    b = a_101 + MotorNotch(2) + W_obj.nNegNotch + 1;
    
    a(a > W_obj.nNotch) = W_obj.nNotch;
    a(a < 1) = 1;
    b(b > W_obj.nNotch) = W_obj.nNotch;
    b(b < 1) = 1;
    
    if rand > epsilon    % ------ greedy
        q_ = sum(W_obj.W(get_possible_action_set(W_obj, current_action(1, i),current_action(2, i)), current_ind_sv(:, i)), 2);
        [~, ind] = max(q_);
        ind = ind(1);
    else                 % ------ random
        ind = ceil(rand*9);
    end
    
    ind1 = rem(ind, 3); ind1(ind1 == 0) = 3;
    ind2 = ceil(ind/3);
    
    action_set(:, i) = [a(ind1); b(ind2)] - (W_obj.nNegNotch + 1); 
end













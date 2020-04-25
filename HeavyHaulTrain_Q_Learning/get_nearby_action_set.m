function action_set = get_nearby_action_set(W_obj, action)
%% coding

action_set = W_obj.nearby_index + action;
action_set(action_set > W_obj.nAction) = W_obj.nAction;
action_set(action_set < 1) = 1;





function W_obj = get_W_obj_LitleCar_linear(rangV, rangS, detaS, detaV, nTiling, ActionSet)

% detaS = 200; % unit m
% detaV = 20;  % unit km/h
% maxS = 2000; % unit m
% maxV = 120;  % unit km/h
% nTiling = 10;  % number of tiling 
% nl = 2;
% nNotch = 25;

% W_obj = get_Wobj_linear(maxS, maxV, detaS, detaV, nTiling, nl, nNotch)

%% 

tile_rand = linspace(0, 1, nTiling+1)'*[1 1];

tile_rand = (tile_rand(1:end-1, :) + tile_rand(2:end, :))/2;
tile_rand = tile_rand + (rand(size(tile_rand))*2-1)/nTiling/2;
tile_rand(1, :) = [0 0];
tile_rand(end, :) = [1 1]-1e-5;

tile_rand(:, 1) = tile_rand(:, 1)*detaS;
tile_rand(:, 2) = tile_rand(:, 2)*detaV;

% % input range(S) = [0 2000]; range(V) = [0 120];

nS = ceil(diff(rangS)/detaS)+1;
nV = ceil(diff(rangV)/detaV)+1;
nAction = length(ActionSet);

% nl = 2;
W = rand(nAction, nS*nV*nTiling)*0e-1;  % initialization 

%% 

W_obj.rangS = rangS; % unit m
W_obj.rangV = rangV;  % unit km/h

W_obj.bound = [rangS; rangV];

W_obj.detaS = detaS; % unit m
W_obj.detaV = detaV;  % unit km/h
W_obj.nTiling = nTiling;
W_obj.tile_rand = tile_rand;
W_obj.W = W;
W_obj.nS = nS;
W_obj.nV = nV;
W_obj.nSV = nS*nV;
W_obj.tilingIndex = (0:nTiling-1)';
W_obj.nAction = nAction;
W_obj.ActionSet = ActionSet;

% W_obj.nV = (0:nTiling-1)'*detaS*detaV;
%% operation

W_obj.coding_index_transform = @coding_index_transform;
W_obj.state_update = @state_update;
W_obj.choose_next_action = @choose_next_action;
W_obj.update_ActionFunction_linear = @update_ActionFunction_linear;
W_obj.max_Q_value_of_State = @max_Q_value_of_State;

function ind_sv = coding_index_transform(W_obj, state)

s = ceil((state(1, :) - W_obj.tile_rand(:, 1) - W_obj.rangS(1))/W_obj.detaS);
v = ceil((state(2, :) - W_obj.tile_rand(:, 2) - W_obj.rangV(1))/W_obj.detaV);

ind_sv = W_obj.tilingIndex*ones(1, size(s, 2)) * W_obj.nSV + s*W_obj.nV + v + 1;

%%

function x = state_update(W_obj, x, a)

x(2) = x(2) + 0.001*a-0.0025*cos(3*x(1));
x(1) = x(1) + x(2);
bo = x < W_obj.bound(:, 1);
x(bo) = W_obj.bound(bo, 1);
bo = x > W_obj.bound(:, 2);
x(bo) = W_obj.bound(bo, 2);

%%

function action = choose_next_action(W_obj, state, epsilon)

if rand > epsilon
    ind_sv = W_obj.coding_index_transform(W_obj, state);
    vals = sum(W_obj.W(:, ind_sv), 2);
    [~, ind] = max(vals);
else
    ind = ceil(rand*W_obj.nAction);
end

action = W_obj.ActionSet(ind);

%%

function [v, action] = max_Q_value_of_State(W_obj, state)

ind_sv = W_obj.coding_index_transform(W_obj, state);
vals = sum(W_obj.W(:, ind_sv), 2);
[v, ind] = max(vals);
action = W_obj.ActionSet(ind);

%%
function W_obj = update_ActionFunction_linear(W_obj, state, ation, state_, r)

ation = ation+2;

current_ind_sv = W_obj.coding_index_transform(W_obj, state);

%% current action (notch)

current_action_value = sum(W_obj.W(ation, current_ind_sv));

%% estimate target_action_value

target_action_value = r + W_obj.gamma*W_obj.max_Q_value_of_State(W_obj, state_);

%% update the weight W

W_obj.W(ation, current_ind_sv) = W_obj.W(ation, current_ind_sv) + ...
    W_obj.alpha * (target_action_value - current_action_value);





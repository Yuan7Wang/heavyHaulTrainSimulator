function W_obj = get_W_obj_LitleCar_NN1(nHidden, rangV, rangS, detaS, detaV, nTiling, ActionSet)

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
%%
% W = rand(nAction, nS*nV*nTiling)*0e-1;  % initialization

W1 = -rand(nHidden, nS*nV*nTiling)*1e-2;  % initialization weight
W2 = -rand(nAction, nHidden)*1e-2;  % initialization weight

b1 = -rand(nHidden, 1)*1e-2;  % initialization bais
b2 = -rand(nAction, 1)*1e-2;  % initialization bais

%%

W_obj.rangS = rangS; % unit m
W_obj.rangV = rangV;  % unit km/h

W_obj.bound = [rangS; rangV];

W_obj.detaS = detaS; % unit m
W_obj.detaV = detaV;  % unit km/h
W_obj.nTiling = nTiling;
W_obj.tile_rand = tile_rand;
W_obj.W1 = W1;
W_obj.W2 = W2;
W_obj.b1 = b1;
W_obj.b2 = b2;
W_obj.nS = nS;
W_obj.nV = nV;
W_obj.nSV = nS*nV;
W_obj.tilingIndex = (0:nTiling-1)';
W_obj.nAction = nAction;
W_obj.ActionSet = ActionSet;
W_obj.nHidden = nHidden;
% W_obj.nV = (0:nTiling-1)'*detaS*detaV;
%% operation

W_obj.coding_index_transform = @coding_index_transform;
W_obj.state_update = @state_update;
W_obj.choose_next_action = @choose_next_action;
W_obj.update_ActionFunction = @update_ActionFunction;
W_obj.max_Q_value_of_State = @max_Q_value_of_State;

W_obj.cal_hidden_output = @cal_hidden_output;
W_obj.cal_final_output = @cal_final_output;
W_obj.cal_gradient = @cal_gradient;


%%
function out = cal_hidden_output(W_obj, x_code)
out = sigmod(sum(W_obj.W1(:, x_code), 2) + W_obj.b1);


%%

function out = cal_final_output(W_obj, hidden_out, N)
if nargin == 2
    out = W_obj.W2*hidden_out + W_obj.b2;
elseif nargin == 3
    out = W_obj.W2(N, :)*hidden_out + W_obj.b2(N);
end


function [dW1, dW2, db2] = cal_gradient(W_obj, N , out1)

dW2 = out1';
db2 = 1;
s_1 = out1.*(1-out1).*W_obj.W2(N, :)';
dW1 = s_1;
% db1 = s_1;


%%

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
%     vals = sum(W_obj.W(:, ind_sv), 2);
    vals = W_obj.cal_final_output(W_obj, W_obj.cal_hidden_output(W_obj, ind_sv));
    [~, ind] = max(vals);
else
    ind = ceil(rand*W_obj.nAction);
end

action = W_obj.ActionSet(ind);

%%

function [v, action] = max_Q_value_of_State(W_obj, state)

ind_sv = W_obj.coding_index_transform(W_obj, state);
vals = W_obj.cal_final_output(W_obj, W_obj.cal_hidden_output(W_obj, ind_sv));
[v, ind] = max(vals);
action = W_obj.ActionSet(ind);

%%


function v = sigmod(x)
v = 1./(1+exp(-x));

%%
function W_obj = update_ActionFunction(W_obj, W_obj_copy, state, action, state_, r)

action = action + 2;

current_ind_sv = coding_index_transform(W_obj, state);
next_ind_sv = coding_index_transform(W_obj, state_);

%% current action (notch)
%     notch = [10; 10];
% notch = action_ind(:, i);
% N_ind = W_obj.ind_t(W_obj, notch);

%% calculate current_action_value
% 使用新的 W_obj 来计算 target 值

out1 = cal_hidden_output(W_obj, current_ind_sv);
current_action_value = W_obj.cal_final_output(W_obj, out1, action);

%% calculate gradient

[dW1, dW2, db2] = cal_gradient(W_obj, action , out1);

%% estimate target_action_value
% 使用旧的 W_obj_copy 来计算 target 值

vals = W_obj.cal_final_output(W_obj_copy, W_obj.cal_hidden_output(W_obj_copy, next_ind_sv));

target_action_value = r + W_obj.gamma*max(vals);

%% 

delta = (target_action_value - current_action_value);
delta(delta > 1) = 1;
delta(delta < -1) = -1;

%%

W_obj.W1(:, current_ind_sv) = W_obj.W1(:, current_ind_sv) + W_obj.alpha * delta * dW1; %; repmat(dW1, 1, W_obj.nTiling);
W_obj.W2(action, :) =  W_obj.W2(action, :) + W_obj.alpha * delta* dW2;
W_obj.b1 = W_obj.b1 + W_obj.alpha * delta* dW1;    % because dW1s is the same as db1s
W_obj.b2 = W_obj.b2 + W_obj.alpha * delta* db2;
    
%%


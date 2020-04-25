function W_obj = get_W_obj_LitleCar_DNN(nHiddens, rangV, rangS, detaS, detaV, nTiling, ActionSet)

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

nLayer = length(nHiddens);

Wb = cell(nLayer + 1, 2);
Wb{1, 1} = randn(nHiddens(1), nS*nV*nTiling)*1e-1;  % initialization weight
Wb{1, 2} = randn(nHiddens(1), 1)*1e-1;  % initialization bais

for ntr = 2:nLayer
    Wb{ntr, 1} = randn(nHiddens(ntr), nHiddens(ntr-1))*1e-1;  % initialization weight
    Wb{ntr, 2} = -randn(nHiddens(ntr), 1)*1e-2;  % initialization bais
end
Wb{nLayer + 1, 1} = randn(nAction, nHiddens(nLayer))*1e-1;  % initialization weight
Wb{nLayer + 1, 2} = randn(nAction, 1)*1e-1;  % initialization bais

W_obj.nLayer = nLayer; % unit m
W_obj.Wb = Wb;  % unit km/h

%%

W_obj.rangS = rangS; % unit m
W_obj.rangV = rangV;  % unit km/h

W_obj.bound = [rangS; rangV];

W_obj.detaS = detaS; % unit m
W_obj.detaV = detaV;  % unit km/h
W_obj.nTiling = nTiling;
W_obj.tile_rand = tile_rand;
W_obj.Wb = Wb;
% W_obj.W1 = W1;
% W_obj.W2 = W2;
% W_obj.b1 = b1;
% W_obj.b2 = b2;
W_obj.nS = nS;
W_obj.nV = nV;
W_obj.nSV = nS*nV;
W_obj.tilingIndex = (0:nTiling-1)';
W_obj.nAction = nAction;
W_obj.ActionSet = ActionSet;
W_obj.nHidden = nHiddens;
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

W_obj.action_Q_value_of_State = @action_Q_value_of_State;

%%
function out = cal_hidden_output(W_obj, x_code)

nLayer = W_obj.nLayer;
out = cell(nLayer, 1);

out{1} = activeF(sum(W_obj.Wb{1, 1}(:, x_code), 2) + W_obj.Wb{1, 2});
for ntr = 2:nLayer
    out{ntr} = activeF(W_obj.Wb{ntr, 1}*out{ntr-1} + W_obj.Wb{ntr, 2});
end


%%

function out = cal_final_output(W_obj, hidden_out, N)
if nargin == 2
    out = W_obj.Wb{end, 1}*hidden_out + W_obj.Wb{end, 2};
elseif nargin == 3
    out = W_obj.Wb{end, 1}(N, :)*hidden_out + W_obj.Wb{end, 2}(N); 
end

%%
function gradient_ = cal_gradient(W_obj, N , hidden_out)

nLayer = W_obj.nLayer;
gradient_ = cell(nLayer + 1, 2);

gradient_{nLayer + 1, 1} = hidden_out{end};
gradient_{nLayer + 1, 2} = 1;

Sm = 1;
dF = dActiveF(hidden_out{nLayer});
% dF = hidden_out{nLayer} > 0;  % ReLu 的 导数， 大于0等于1， 小于0等于0
Sm = dF.*W_obj.Wb{nLayer + 1, 1}(N, :)'*Sm;
% Sm = Sm/norm(Sm)*0.01;
if nLayer == 1
    gradient_{nLayer, 1} = Sm;
    gradient_{nLayer, 2} = Sm;
    return;
else
    gradient_{nLayer, 1} = Sm * hidden_out{nLayer - 1}';
    gradient_{nLayer, 2} = Sm;
end

for ntr = nLayer-1:-1:2
    dF = dActiveF(hidden_out{ntr});
%     dF = hidden_out{ntr} > 0;  % ReLu 的 导数， 大于0等于1， 小于0等于0
    Sm = diag(dF)*W_obj.Wb{ntr+1, 1}'*Sm;
%     Sm = Sm/norm(Sm)*0.01;
    gradient_{ntr, 1} = Sm * hidden_out{ntr - 1}';
    gradient_{ntr, 2} = Sm;
end

dF = dActiveF(hidden_out{1});
% dF = hidden_out{1} > 0;  % ReLu 的 导数， 大于0等于1， 小于0等于0
Sm = diag(dF)*W_obj.Wb{1 + 1, 1}'*Sm;
% Sm = Sm/norm(Sm)*0.01;
% Sm = W_obj.Wb{1, 1}'*Sm;
gradient_{1, 1} = Sm;
gradient_{1, 2} = Sm;

% dW2 = hidden_out';
% db2 = 1;
% s_1 = hidden_out.*(1-hidden_out).*W_obj.W2(N, :)';
% dW1 = s_1;
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
    hidden_out_ = cal_hidden_output(W_obj, ind_sv);
    vals = W_obj.cal_final_output(W_obj, hidden_out_{end});
%     vals = sum(W_obj.W(:, ind_sv), 2);
%     vals = W_obj.cal_final_output(W_obj, W_obj.cal_hidden_output(W_obj, ind_sv));
    [~, ind] = max(vals);
else
    ind = ceil(rand*W_obj.nAction);
end

action = W_obj.ActionSet(ind);

%%

function [v, action] = max_Q_value_of_State(W_obj, state)

ind_sv = W_obj.coding_index_transform(W_obj, state);
hidden_out_ = cal_hidden_output(W_obj, ind_sv);
vals = W_obj.cal_final_output(W_obj, hidden_out_{end});
% vals = W_obj.cal_final_output(W_obj, W_obj.cal_hidden_output(W_obj, ind_sv));
[v, ind] = max(vals);
action = W_obj.ActionSet(ind);

%%

function val = action_Q_value_of_State(W_obj, state, N)

ind_sv = W_obj.coding_index_transform(W_obj, state);
hidden_out_ = cal_hidden_output(W_obj, ind_sv);
val = W_obj.cal_final_output(W_obj, hidden_out_{end}, N);
% vals = W_obj.cal_final_output(W_obj, W_obj.cal_hidden_output(W_obj, ind_sv));
% [v, ind] = max(vals);
% action = W_obj.ActionSet(ind);

%%

function x = activeF(x)
% x(x < 0) = 0;
x(x < 0) = 0.01*x(x < 0);

% x(x < 0) = 0.01*(exp(x(x < 0)) - 1);

% x = sigmod(x);
% x = tanh(x);

% v = 1./(1+exp(-x));

function d = dActiveF(x)
d = x > 0;
d(x < 0) = 0.01;
% d(x < 0) = x(x < 0) + 0.01;

% d = x.*(1-x);
% d = 1-x.^2;

% x(x < 0) = 0;
% x = sigmod(x);
% v = 1./(1+exp(-x));

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

hidden_out = cal_hidden_output(W_obj, current_ind_sv);
current_action_value = W_obj.cal_final_output(W_obj, hidden_out{end}, action);

%% calculate gradient

gradient_ = cal_gradient(W_obj, action , hidden_out);

%% estimate target_action_value
% 使用旧的 W_obj_copy 来计算 target 值

hidden_out_ = cal_hidden_output(W_obj_copy, next_ind_sv);
vals = W_obj.cal_final_output(W_obj_copy, hidden_out_{end});

if r == 0
    target_action_value = r;
else
    target_action_value = r + W_obj.gamma*max(vals);
end


%% 

delta = (target_action_value - current_action_value);
% delta(delta > 1) = 1;
% delta(delta < -1) = -1;
delta = W_obj.alpha * delta;

%% 随机梯度下降

W_obj.Wb{1, 1}(:, current_ind_sv) = W_obj.Wb{1, 1}(:, current_ind_sv) + delta * gradient_{1, 1}; %; repmat(dW1, 1, W_obj.nTiling);
W_obj.Wb{1, 2} = W_obj.Wb{1, 2} + delta* gradient_{1, 1};    % because dW1s is the same as db1s

for ntr = 2:W_obj.nLayer
    W_obj.Wb{ntr, 1} = W_obj.Wb{ntr, 1} + delta * gradient_{ntr, 1};
    W_obj.Wb{ntr, 2} = W_obj.Wb{ntr, 2} + delta * gradient_{ntr, 2};
end

W_obj.Wb{W_obj.nLayer + 1, 1}(action, :) =  W_obj.Wb{W_obj.nLayer + 1, 1}(action, :) ...
    + delta* gradient_{W_obj.nLayer + 1, 1}';
W_obj.Wb{W_obj.nLayer + 1, 2} = W_obj.Wb{W_obj.nLayer + 1, 2} + delta * gradient_{W_obj.nLayer + 1, 2};



% W_obj.W1(:, current_ind_sv) = W_obj.W1(:, current_ind_sv) + W_obj.alpha * delta * dW1; %; repmat(dW1, 1, W_obj.nTiling);
% W_obj.W2(action, :) =  W_obj.W2(action, :) + W_obj.alpha * delta* dW2;
% W_obj.b1 = W_obj.b1 + W_obj.alpha * delta* dW1;    % because dW1s is the same as db1s
% W_obj.b2 = W_obj.b2 + W_obj.alpha * delta* db2;
    
%%


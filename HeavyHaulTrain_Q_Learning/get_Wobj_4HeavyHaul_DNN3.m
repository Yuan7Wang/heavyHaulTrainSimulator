function W_obj = get_Wobj_4HeavyHaul_DNN3(nHiddens, rangS, rangV, detaS, detaV, nTiling, nl, nPosNotch, nNegNotch, mini_batch)

W_obj.mini_batch = mini_batch;

%% state_coding

tile_rand = linspace(0, 1, nTiling+1)'*[1 1];

tile_rand = (tile_rand(1:end-1, :) + tile_rand(2:end, :))/2;
tile_rand = tile_rand + (rand(size(tile_rand))*2-1)/nTiling/2;
tile_rand(1, :) = [0 0];
tile_rand(end, :) = [1 1]-1e-3;

tile_rand(:, 1) = tile_rand(:, 1)*detaS;
tile_rand(:, 2) = tile_rand(:, 2)*detaV;

[~, ind] = sort(rand(size(tile_rand, 1), 1));
tile_rand(:, 2) = tile_rand(ind, 2);


%%  ationSet

nS = ceil(diff(rangS)/detaS) + 1;
nV = ceil(diff(rangV)/detaV) + 1;
nSV = nS*nV;

action_ = -nPosNotch:nNegNotch;
[Ax, Ay] = meshgrid(action_);
ActionSet = [Ax(:) Ay(:)]';
nAction = size(ActionSet, 2);

nNotch = length(action_); 

W_obj.Ax = Ax;
W_obj.Ay = Ay;
%% Action Coding

rangA = [-nPosNotch nNegNotch]; 
% detaA = 5;
% nTile = 5;
detaA = 25;
nTile = detaA;
nAcodeGrid = ceil(nNotch/detaA)+1;
% n_action_dim = 2;
nAG = nAcodeGrid^nl;
code_offset_ = (0:nTile-1)' + 0.5;
code_offset = [code_offset_ code_offset_(end:-1:1)];

W_obj.rangA = rangA;
W_obj.detaA = detaA;
W_obj.nTile = nTile;
% W_obj.n_action_dim = n_action_dim;
W_obj.nAG = nAG;
W_obj.nAcodeGrid = nAcodeGrid;
W_obj.code_offset = code_offset;
W_obj.action_tilingIndex = (0:nTile-1)';

AllActionCode = zeros(nTile, nAction);
for itr = 1:nAction
    AllActionCode(:, itr) = coding_action_transform(W_obj, ActionSet(:, itr));
end

W_obj.AllActionCode = AllActionCode;

%% Weight 

% nl = 2;
% W = rand(nAG*nTile, nSV*nTiling)*0e-2;  % initialization 

nLayer = length(nHiddens);

Wb = cell(nLayer + 1, 2);
Wb{1, 1} = randn(nHiddens(1), nSV*nTiling)*1e-3;  % initialization weight
Wb{1, 2} = randn(nHiddens(1), 1)*1e-3;  % initialization bais

for ntr = 2:nLayer
    Wb{ntr, 1} = randn(nHiddens(ntr), nHiddens(ntr-1))*1e-3;  % initialization weight
    Wb{ntr, 2} = -randn(nHiddens(ntr), 1)*1e-3;  % initialization bais
end
Wb{nLayer + 1, 1} = randn(nAG*nTile, nHiddens(nLayer))*1e-3;  % initialization weight
Wb{nLayer + 1, 2} = randn(nAG*nTile, 1)*1e-3;  % initialization bais

W_obj.nHiddens = nHiddens; % unit m
W_obj.nLayer = nLayer; % unit m
W_obj.Wb = Wb;  % unit km/h

W_obj.gradients = cell(nLayer + 1, 2);
ntr = 1;
W_obj.gradients{ntr, 1} = zeros(nHiddens(ntr), mini_batch);
W_obj.gradients{ntr, 2} = zeros(nHiddens(ntr), mini_batch);
if nLayer > 2
    for ntr = 2:-1:nLayer
        W_obj.gradients{ntr, 1} = zeros(nHiddens(ntr), nHiddens(ntr-1), mini_batch);
        W_obj.gradients{ntr, 2} = zeros(nHiddens(ntr), mini_batch);
    end
end
W_obj.gradients{nLayer+1, 1} = 0;
W_obj.gradients{nLayer+1, 2} = 0;

%% 
W_obj.G = my_ndgrid(nl, 1:3);
n_p_actions = size(W_obj.G, 1);
W_obj.n_p_actions = n_p_actions;

%% 

W_obj.rangS = rangS; % unit m
W_obj.rangV = rangV;  % unit km/h

W_obj.detaS = detaS; % unit m
W_obj.detaV = detaV;  % unit km/h
W_obj.nTiling = nTiling;
W_obj.tile_rand = tile_rand;
% W_obj.W = W;
W_obj.nS = nS;
W_obj.nV = nV;
W_obj.nSV = nSV;
W_obj.nl = nl;
W_obj.tilingIndex = (0:nTiling-1)';

W_obj.nPosNotch = nPosNotch;
W_obj.nNegNotch = nNegNotch;
W_obj.nNotch = nNotch;
W_obj.nAction = nAction;
W_obj.ActionSet = ActionSet;

% W_obj.nV = (0:nTiling-1)'*detaS*detaV;

%% 创建一个与 action 相关的字典，记录每一个可选aciton、对应下一步action集合，以及对应的编码

W_obj.ActionCoding_Set = zeros(nTile, nNotch, nNotch);
W_obj.NextAction_Set = zeros(nl, n_p_actions, nNotch, nNotch);
W_obj.NextActionCoding_Set = zeros(nTile, n_p_actions, nNotch, nNotch);
for itr = 1:size(ActionSet, 2)
    notch = ActionSet(:, itr);
    notch_ = notch + nNegNotch + 1;
    W_obj.ActionCoding_Set(:, notch_(1), notch_(2)) = coding_action_transform(W_obj, notch);
    W_obj.NextAction_Set(: ,: , notch_(1), notch_(2)) = get_possible_notch_set(W_obj, notch);
    W_obj.NextActionCoding_Set(: ,: , notch_(1), notch_(2)) = ...
        coding_action_transform(W_obj, W_obj.NextAction_Set(: ,: , notch_(1), notch_(2)));
end


%% operations

W_obj.coding_index_transform = @coding_index_transform;
W_obj.coding_action_transform = @coding_action_transform;

W_obj.index_tranform = @index_tranform;
W_obj.choose_next_action = @choose_next_action;
W_obj.update_ActionFunction = @update_ActionFunction;

W_obj.update_ActionFunction_batch = @update_ActionFunction_batch;
W_obj.update_ActionFunction_half_batch = @update_ActionFunction_half_batch;
W_obj.cal_hidden_output = @cal_hidden_output;
W_obj.cal_final_output = @cal_final_output;
W_obj.cal_gradient = @cal_gradient;

W_obj.get_state_value_by_speed = @get_state_value_by_speed;
W_obj.max_Q_value_of_State = @max_Q_value_of_State;
W_obj.action_Q_value_of_State = @action_Q_value_of_State;


W_obj.evaluate_Qnetwork = @evaluate_Qnetwork;
W_obj.evaluate_Tnetwork = @evaluate_Tnetwork;

W_obj.obtain_speed_limit = @obtain_speed_limit;

%%

% function out = evaluate_Qnetwork(W_obj, Scode, Acode)
% % Scode and Acode must share the same batch_size (columns)
% % 向量化计算
% 
% batch_size = size(Scode, 2);
% 
% out = cell(W_obj.nLayer+1, 1);
% % 输入层
% ntr = 1;
% out{ntr} = activeF(sum(reshape(W_obj.Wb{1, 1}(:, Scode), W_obj.nHiddens(1), W_obj.nTiling, batch_size), 2) + W_obj.Wb{1, 2});
% out{ntr} = reshape(activeF(sum(reshape(W_obj.Wb{1, 1}(:, Scode), ...
%     W_obj.nHiddens(1), W_obj.nTiling, batch_size), 2) + W_obj.Wb{1, 2}), ...
%     [W_obj.nHiddens(1) batch_size]);
% 
% % 隐含层
% for ntr = 2:W_obj.nLayer
%     out{ntr} = activeF(W_obj.Wb{ntr, 1}*out{ntr-1} + W_obj.Wb{ntr, 2});
% end
% 
% % 输出层， 线性输出
% ntr = ntr+1;
% TMPw = reshape(sum(reshape(W_obj.Wb{ntr, 1}(Acode, :)', W_obj.nHiddens(ntr), ...
%     W_obj.nTile, batch_size), 2), [W_obj.nHiddens(ntr), batch_size]); 
% 
% out{ntr} = sum(TMPw.*out{ntr-1}) + sum(W_obj.Wb{end, 2}(Acode));  % 注意，这里是对应相乘
% 
% out{ntr} = sum(TMPw'*out{ntr-1}) + sum(W_obj.Wb{end, 2}(Acode));  % 注意，这里是对应相乘

%%

function out = evaluate_Qnetwork(W_obj, Scode, Acode)
% Scode and Acode must share the same batch_size (columns)
% 向量化计算

batch_size1 = size(Scode, 2);
batch_size2 = size(Acode, 2);

out = cell(W_obj.nLayer+1, 1);
% 输入层
ntr = 1;
out{ntr} = reshape(activeF(sum(reshape(W_obj.Wb{1, 1}(:, Scode), ...
    W_obj.nHiddens(1), W_obj.nTiling, batch_size1), 2) + W_obj.Wb{1, 2}), ...
    [W_obj.nHiddens(1) batch_size1]);

% 隐含层
if W_obj.nLayer > 2
    for ntr = 2:W_obj.nLayer
        out{ntr} = activeF(W_obj.Wb{ntr, 1}*out{ntr-1} + W_obj.Wb{ntr, 2});
    end
end

% 输出层， 线性输出
out{ntr+1} = zeros(1, batch_size2);
for itr = 1:batch_size2
    tmp = sum(W_obj.Wb{end, 1}(Acode(:, itr), :)*out{ntr}(:, itr) + W_obj.Wb{end, 2}(Acode(:, itr)), 2);
    out{ntr+1}(:, itr) = sum(tmp);
end

% TMPw = reshape(sum(reshape(W_obj.Wb{ntr+1, 1}(Acode, :)', W_obj.nHiddens(ntr), ...
%     W_obj.nTile, batch_size2), 2), [W_obj.nHiddens(ntr), batch_size2]); 
% 
% out{ntr+1} = sum(TMPw.*out{ntr}) + sum(W_obj.Wb{end, 2}(Acode));  % 注意，这里是对应相乘
% 
% tmp = sum(W_obj.Wb{ntr+1, 1}(Ncode(:), :)*hidden_out + W_obj.Wb{ntr+1, 2}(Ncode(:)), 2);
% out = sum(reshape(tmp, size(Ncode)));


%%

function out = evaluate_Tnetwork(W_obj, Scode, Acode)
% Scode and Acode must share the same batch_size (columns)
% 向量化计算

batch_size1 = size(Scode, 2);
sizeA = size(Acode);

% 输入层
out = reshape(activeF(sum(reshape(W_obj.Wb{1, 1}(:, Scode), ...
    W_obj.nHiddens(1), W_obj.nTiling, batch_size1), 2) + W_obj.Wb{1, 2}), ...
    [W_obj.nHiddens(1) batch_size1]);

% 隐含层
for ntr = 2:W_obj.nLayer
    out = activeF(W_obj.Wb{ntr, 1}*out + W_obj.Wb{ntr, 2});
end

% 输出层， 线性输出
TMPw = zeros(sizeA(2), sizeA(3));
for itr = 1:sizeA(3)
    tmp = sum(W_obj.Wb{end, 1}(Acode(:, :, itr), :)*out(:, itr) + W_obj.Wb{end, 2}(Acode(:, :, itr), :), 2);
    TMPw(:, itr) = sum(reshape(tmp, sizeA(1:2)));
end
out = TMPw;
    
%% 

% function res = evaluate_Tnetwork(W_obj, Scode, Acode)


%%

function Lspeed = obtain_speed_limit(v, speed_ChangePoint, hHpeed_Limits)

Lspeed = zeros(size(v));

itr = 1;
bo_tmp = v < speed_ChangePoint(itr);
Lspeed(bo_tmp) = hHpeed_Limits(itr);
% Lspeed(~bo_tmp) = hHpeed_Limits(2);
for itr = 2:length(speed_ChangePoint)
    bo_tmp = v >= speed_ChangePoint(itr-1) & v < speed_ChangePoint(itr); 
    Lspeed(bo_tmp) = hHpeed_Limits(itr);
end
bo_tmp = v >= speed_ChangePoint(itr);
Lspeed(bo_tmp) = hHpeed_Limits(itr+1);

% Lspeed(bo_tmp) = hHpeed_Limits(1);
% Lspeed(~bo_tmp) = hHpeed_Limits(2);


%% 
function action_set = choose_next_action(W_obj, current_state, current_action, epsilon)
%% coding 

current_ind_sv = coding_index_transform(W_obj, current_state);
action_set = zeros(size(current_action));
%% 
for i = 1:size(action_set, 2)
        
    current_sv_coding = current_ind_sv(:, i); 
    
    notch = current_action(:, i);
    
    [notch_set, N_ind] = get_possible_notch_set(W_obj, notch);
        
    if rand > epsilon    % ------ greedy
        
        hidden_out_ = cal_hidden_output(W_obj, current_sv_coding);
        vals = cal_final_output(W_obj, hidden_out_{end}, N_ind);
        
%         vals = cal_final_output(W_obj, current_ind_sv(:, i), N_ind);
        [~, ind] = max(vals);
        ind = ind(1);
    else                 % ------ random
        ind = ceil(rand*size(notch_set, 2));
    end
    
    action_set(:, i) = notch_set(:, ind); 
end

%% 
function [notch_set, ind_] = get_possible_notch_set(W_obj, notch)
%% notch is a colume vector

notch_set = zeros(W_obj.nl, 3^W_obj.nl);

%%
a_101 = [-1; 0; 1];
for itr = 1:size(notch, 1)
    a = a_101 + notch(itr);
    a(a > W_obj.nPosNotch) = W_obj.nPosNotch;
    a(a < -W_obj.nNegNotch) = -W_obj.nNegNotch;
    notch_set(itr, :) = a(W_obj.G(:, itr));
end

if nargout == 2
    ind_ = coding_action_transform(W_obj, notch_set);
end

%%
function out = cal_hidden_output(W_obj, x_code)

out = cell(W_obj.nLayer, 1);

out{1} = activeF(sum(W_obj.Wb{1, 1}(:, x_code), 2) + W_obj.Wb{1, 2});
for ntr = 2:W_obj.nLayer
    out{ntr} = activeF(W_obj.Wb{ntr, 1}*out{ntr-1} + W_obj.Wb{ntr, 2});
end

%%
function out = cal_final_output(W_obj, hidden_out, Ncode)
% 只能单个单个算了

if nargin == 2
    Ncode = W_obj.AllActionCode;
end
% sn = size(Ncode);

% n = size(Ncode);
% out = zeros(n(2), 1);
% for itr = 1:n(2)
%     out(itr) = sum(sum(W_obj.Wb{end, 1}(Ncode(:, itr), :)*hidden_out + W_obj.Wb{end, 2}(Ncode(:, itr))));
% end

tmp = sum(W_obj.Wb{end, 1}(Ncode(:), :)*hidden_out + W_obj.Wb{end, 2}(Ncode(:)), 2);
out = sum(reshape(tmp, size(Ncode)));

% for itr = 1:n
%     out(itr) = sum(W_obj.Wb{end, 1}(Ncode(:, itr), :)*hidden_out + W_obj.Wb{end, 2}(Ncode(:, itr)));
% end

% if nargin == 2
%     out = W_obj.Wb{end, 1}*hidden_out + W_obj.Wb{end, 2};
% elseif nargin == 3
%     out = W_obj.Wb{end, 1}(N, :)*hidden_out + W_obj.Wb{end, 2}(N); 
% end

% if nargin == 2
%     out = sum(W_obj.W(: ,code_), 2);
% elseif nargin == 3
%     out = sum(W_obj.W(N, code_), 2); 
% end

%%
function gradient_ = cal_gradient(W_obj, hidden_out, action_coding)

nLayer = W_obj.nLayer;
gradient_ = cell(nLayer + 1, 2);

gradient_{nLayer + 1, 1} = hidden_out{end};
gradient_{nLayer + 1, 2} = 1;

Sm = ones(W_obj.nTile, 1);
dF = dActiveF(hidden_out{nLayer});
% dF = hidden_out{nLayer} > 0;  % ReLu 的 导数， 大于0等于1， 小于0等于0
Sm = dF.*(W_obj.Wb{nLayer + 1, 1}(action_coding, :)'*Sm);
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
    Sm = dF.*(W_obj.Wb{ntr+1, 1}'*Sm);
%     Sm = Sm/norm(Sm)*0.01;
    gradient_{ntr, 1} = Sm * hidden_out{ntr - 1}';
    gradient_{ntr, 2} = Sm;
end

dF = dActiveF(hidden_out{1});
% dF = hidden_out{1} > 0;  % ReLu 的 导数， 大于0等于1， 小于0等于0
Sm = dF.*(W_obj.Wb{1 + 1, 1}'*Sm);
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
function gradients = batch_gradient(W_obj, hidden_out, action_codeing)

batch_size = size(action_codeing, 2);
nLayer = W_obj.nLayer;
% gradient_ = cell(nLayer + 1, 2);

% 最后一层的 梯度
W_obj.gradients{nLayer + 1, 1} = hidden_out{end};
W_obj.gradients{nLayer + 1, 2} = 1;

% Sm = ones(W_obj.nTile, batch_size);
dF = dActiveF(hidden_out{nLayer});
Sm = dF.*reshape(sum(reshape(W_obj.Wb{nLayer + 1, 1}(action_codeing, :)', ...
    [W_obj.nHiddens(end), W_obj.nTile, batch_size]), 2), [W_obj.nHiddens(end), batch_size]); 

% 倒数第二层-正数第二层的梯度
% Sm = Sm/norm(Sm)*0.01;
if nLayer == 1
    W_obj.gradients{nLayer, 1} = Sm;
    W_obj.gradients{nLayer, 2} = Sm;
    gradients = W_obj.gradients;
    return;
else
    for i = 1:W_obj.mini_batch
        W_obj.gradients{nLayer, 1}(:, :, i) = Sm(:, i)*hidden_out{nLayer - 1}(:, i)';
    end
    W_obj.gradients{nLayer, 2} = Sm;
end

% 倒数第三层，如果存在的话
for ntr = nLayer-1:-1:2
    dF = dActiveF(hidden_out{ntr});
    Sm = dF.*(W_obj.Wb{ntr+1, 1}'*Sm);
    for i = 1:W_obj.mini_batch
        W_obj.gradients{ntr, 1}(:, :, i) = Sm(:, i)*hidden_out{ntr - 1}(:, i)';
    end
    W_obj.gradients{ntr, 2} = Sm;
end

% 第一层的梯度
dF = dActiveF(hidden_out{1}); 
Sm = dF.*(W_obj.Wb{1 + 1, 1}'*Sm); 
W_obj.gradients{1, 1} = Sm;
W_obj.gradients{1, 2} = Sm;

gradients = W_obj.gradients;

% dW2 = hidden_out';
% db2 = 1;
% s_1 = hidden_out.*(1-hidden_out).*W_obj.W2(N, :)';
% dW1 = s_1;
% db1 = s_1;

%%

function x = activeF(x)
x(x < 0) = 0;
% x(x < 0) = 0.01*x(x < 0);

% x(x < 0) = 0.01*(exp(x(x < 0)) - 1);

% x = sigmod(x);
% x = tanh(x);

% v = 1./(1+exp(-x));

function d = dActiveF(x)
d = x > 0;
% d(x < 0) = 0.01;
% d(x < 0) = x(x < 0) + 0.01;

% d = x.*(1-x);
% d = 1-x.^2;

% x(x < 0) = 0;
% x = sigmod(x);
% v = 1./(1+exp(-x));

%% relu
function x = relu(x) %#ok<DEFNU>
x(x < 0) = 0;

function x = drelu(x) %#ok<DEFNU>
x = x > 0;

%% elu
function x = elu(x) %#ok<DEFNU>
x(x < 0) = 0.01*x(x < 0);

function x = delu(x) %#ok<DEFNU>
x = x > 0;
x(x < 0) = 0.01;

%% sigmod
function x = sigmod(x) %#ok<DEFNU>
x = 1./(1+exp(-x));

function x = dsigmod(x) %#ok<DEFNU>
x = x.*(1-x);

%% tanh
% function x = tanh(x)
% x = tanh(x);

function x = dtanh(x) %#ok<DEFNU>
x = 1 - x.^2;

%%

function ind_sv = coding_index_transform(W_obj, state)

s = ceil((state(1, :) - W_obj.tile_rand(:, 1) - W_obj.rangS(1))/W_obj.detaS);
v = ceil((state(2, :) - W_obj.tile_rand(:, 2) - W_obj.rangV(1))/W_obj.detaV);

ind_sv = W_obj.tilingIndex*ones(1, size(s, 2)) * W_obj.nSV + s*W_obj.nV + v + 1;

%%

function ind_action = coding_action_transform(W_obj, Np)

s = floor((Np(1, :) + W_obj.code_offset(:, 1) - W_obj.rangA(1))/W_obj.detaA);
v = floor((Np(2, :) + W_obj.code_offset(:, 2) - W_obj.rangA(1))/W_obj.detaA);

ind_action = W_obj.action_tilingIndex*ones(1, size(s, 2)) * W_obj.nAG + s*W_obj.nAcodeGrid + v + 1;

%% 

function [val, action] = max_Q_value_of_State(W_obj, state_coding, Ncode)
if nargin == 2
    Ncode = W_obj.AllActionCode;
end
tmp = zeros(size(Ncode, 2), 1);
for itr = 1:size(Ncode, 2)
    tmp(itr) = max(action_Q_value_of_State(W_obj, state_coding, Ncode(:, itr)));
end

[val , action] = max(tmp);

% val = reshape(val, W_obj.nTile, W_obj.nTile);
% val = reshape(val, W_obj.nTile, W_obj.nTile);

% ind_sv = W_obj.coding_index_transform(W_obj, state); 
% 
% vals = W_obj.cal_final_output(W_obj, ind_sv);
% % vals = W_obj.cal_final_output(W_obj, W_obj.cal_hidden_output(W_obj, ind_sv));
% if nargin == 1
%     v = max(vals);
% else
%     [v, ind] = max(vals);
%     action = W_obj.ActionSet(ind);
% end

%%

function Val = get_state_value_by_speed(W_obj, s_position, vs)

% s_position = 8000;
% v = 30:1:85; 
n_v = length(vs);
s = ones(1, n_v)*s_position;
current_ind_sv = W_obj.coding_index_transform(W_obj, [s; vs]);

Val = zeros(n_v, 1);
for itr = 1:n_v
    Np = ones(2, 1)*(round(vs(itr)/10) + (-1:1));
    Np(Np > 12 | Np < -12) = []; 
    Ncode = W_obj.coding_action_transform(W_obj, Np);
    Val(itr) = max(W_obj.action_Q_value_of_State(W_obj, current_ind_sv(:, itr), Ncode)); 
end
% figure; plot(v, Val)

%% 

function val = action_Q_value_of_State(W_obj, state_coding, Ncode)

% ind_sv = W_obj.coding_index_transform(W_obj, state);
hidden_out_ = cal_hidden_output(W_obj, state_coding);
val = W_obj.cal_final_output(W_obj, hidden_out_{end}, Ncode);

% vals = W_obj.cal_final_output(W_obj, W_obj.cal_hidden_output(W_obj, ind_sv));
% [v, ind] = max(vals);
% action = W_obj.ActionSet(ind);


%%
function ind_ = index_tranform(W_obj, notch_set)
ind_ = (notch_set(1, :) + W_obj.nNegNotch)*W_obj.nNotch + notch_set(2, :)+ W_obj.nNegNotch + 1;

%%
% tile coding for locaiton-speed space
function W_obj = update_ActionFunction(W_obj, W_obj_copy, Qloc)

p_num = size(Qloc, 1);
% current_state = Qloc(:, 1:2)';
action_ind = Qloc(:, 3:2+W_obj.nl)';
% next_state = Qloc(:, 3+W_obj.nl:4+W_obj.nl)';
rs = Qloc(:, 5+W_obj.nl)';

%% coding

current_ind_sv_ = coding_index_transform(W_obj, Qloc(:, 1:2)');

next_ind_sv_ = coding_index_transform(W_obj, Qloc(:, 3+W_obj.nl:4+W_obj.nl)');

%% % record for debug only

Recordr = zeros(p_num, 3); 

%%

for i = 1:p_num
    %% current action (notch)
    current_sv_coding = current_ind_sv_(:, i);
    next_sv_coding = next_ind_sv_(:, i);
    
    action = action_ind(:, i);
    action_codeing = coding_action_transform(W_obj, action);
    
    %% calculate current_action_value
    % 使用新的 W_obj 来计算 target 值
    
    hidden_out = cal_hidden_output(W_obj, current_sv_coding);
    current_action_value = cal_final_output(W_obj, hidden_out{end}, action_codeing); 
    
    %% calculate gradient 
    gradient_ = cal_gradient(W_obj, hidden_out, action_codeing);
    
    %% estimate target_action_value
    % 使用旧的 W_obj_copy 来计算 target 值
    
    if rs(i) > 0
        target_action_value = -rs(i);
    else
        [~, notch_set] = get_possible_notch_set(W_obj_copy, action);
        hidden_out_ = cal_hidden_output(W_obj_copy, next_sv_coding);
        vals = cal_final_output(W_obj_copy, hidden_out_{end}, notch_set);
        target_action_value = rs(i) + W_obj_copy.gamma * max(vals);
    end
    
    %% calculate delta
    
    delta = (target_action_value - current_action_value);
    % delta(delta > 1) = 1;
    % delta(delta < -1) = -1;
    delta = W_obj.alpha * delta;
    
    Recordr(i, :) = [target_action_value, current_action_value, delta];
    
    %% 随机梯度下降
    
    W_obj.Wb{1, 1}(:, current_sv_coding) = W_obj.Wb{1, 1}(:, current_sv_coding) + delta * gradient_{1, 1}; %; repmat(dW1, 1, W_obj.nTiling);
    W_obj.Wb{1, 2} = W_obj.Wb{1, 2} + delta* gradient_{1, 1};    % because dW1s is the same as db1s
    
    for ntr = 2:W_obj.nLayer
        W_obj.Wb{ntr, 1} = W_obj.Wb{ntr, 1} + delta * gradient_{ntr, 1};
        W_obj.Wb{ntr, 2} = W_obj.Wb{ntr, 2} + delta * gradient_{ntr, 2};
    end
    
    W_obj.Wb{W_obj.nLayer + 1, 1}(action_codeing, :) =  W_obj.Wb{W_obj.nLayer + 1, 1}(action_codeing, :) ...
        + delta* gradient_{W_obj.nLayer + 1, 1}';
    W_obj.Wb{W_obj.nLayer + 1, 2} = W_obj.Wb{W_obj.nLayer + 1, 2} + delta * gradient_{W_obj.nLayer + 1, 2};
    
    %     W_obj.W(N_ind, current_ind_sv(:, i)) = W_obj.W(N_ind, current_ind_sv(:, i)) + ...
    %         W_obj.alpha * (target_action_value - current_action_value);
end

if nargout == 2 
end

%%

% tile coding for locaiton-speed space
function W_obj = update_ActionFunction_batch(W_obj, W_obj_copy, Qloc)

batch_size = size(Qloc, 1); 
actions = Qloc(:, 3:2+W_obj.nl)'; 
rs = Qloc(:, 5+W_obj.nl)';

%% state coding

Scode = coding_index_transform(W_obj, Qloc(:, 1:2)');
Scode_ = coding_index_transform(W_obj, Qloc(:, 3+W_obj.nl:4+W_obj.nl)');

%% action coding

actions = actions + W_obj.nNegNotch;    % 转化为 index 
actions_index = actions(1, :)*W_obj.nNotch + actions(2, :) + 1;

% 当前 action 编码
Acode = W_obj.ActionCoding_Set(:, actions_index);

% 下一步可选 action 编码
Acode_ = W_obj.NextActionCoding_Set(:, :, actions_index);



%% calculate current_action_value, with a batch

Res = evaluate_Qnetwork(W_obj, Scode, Acode); 

%% calculate gradient , with a batch

gradients = batch_gradient(W_obj, Res, Acode);

%% estimate target_action_value
% 使用旧的 W_obj_copy 来计算 target 值

Res_ = evaluate_Tnetwork(W_obj_copy, Scode_, Acode_); 
% Res_ = reshape(Res_, [], batch_size);
target_action_value = rs + W_obj_copy.gamma * max(Res_);
target_action_value(rs > 0) = -rs((rs > 0));

delta = W_obj.alpha * (target_action_value - Res{end});

%% 随机梯度下降，更新所有相关权重系数 with a batch

% 第一层和最后一层
for i = 1:batch_size
    W_obj.Wb{1, 1}(:, Scode(:, i)) = W_obj.Wb{1, 1}(:, Scode(:, i)) + delta(i) * gradients{1, 1}(:, i); %; repmat(dW1, 1, W_obj.nTiling);
    W_obj.Wb{1, 2} = W_obj.Wb{1, 2} + delta(i)* gradients{1, 1}(:, i);    % because dW1s is the same as db1s
    
    W_obj.Wb{W_obj.nLayer + 1, 1}(Acode(:, i), :) =  W_obj.Wb{W_obj.nLayer + 1, 1}(Acode(:, i), :) ...
        + delta(i)* gradients{W_obj.nLayer + 1, 1}(:, i)';
    W_obj.Wb{W_obj.nLayer + 1, 2} = W_obj.Wb{W_obj.nLayer + 1, 2} + delta(i) * gradients{W_obj.nLayer + 1, 2};
    
    for ntr = 2:W_obj.nLayer
        W_obj.Wb{ntr, 1} = W_obj.Wb{ntr, 1} + delta(i) * gradients{ntr, 1}(:, :, i);
        W_obj.Wb{ntr, 2} = W_obj.Wb{ntr, 2} + delta(i) * gradients{ntr, 2}(:, i);
    end
end
%%

% tile coding for locaiton-speed space
function W_obj = update_ActionFunction_half_batch(W_obj, W_obj_copy, Qloc)

batch_size = size(Qloc, 1); 
actions = Qloc(:, 3:2+W_obj.nl)'; 
rs = Qloc(:, 5+W_obj.nl)';

%% state coding

Scode = coding_index_transform(W_obj, Qloc(:, 1:2)');
Scode_ = coding_index_transform(W_obj, Qloc(:, 3+W_obj.nl:4+W_obj.nl)');

%% action coding

actions = actions + W_obj.nNegNotch;    % 转化为 index 
actions_index = actions(1, :)*W_obj.nNotch + actions(2, :) + 1;

% 当前 action 编码
Acode = W_obj.ActionCoding_Set(:, actions_index);

% 下一步可选 action 编码
Acode_ = W_obj.NextActionCoding_Set(:, :, actions_index);

for i = 1:batch_size
    %% current action (notch)
    current_sv_coding = Scode(:, i);
    next_sv_coding = Scode_(:, i);
    
    action = Acode(:, i);
    action_coding = coding_action_transform(W_obj, action);
    
    %% calculate current_action_value
    % 使用新的 W_obj 来计算 target 值
    
    hidden_out = cal_hidden_output(W_obj, current_sv_coding);
    current_action_value = cal_final_output(W_obj, hidden_out{end}, action_coding); 
    
    %% calculate gradient 
    gradient_ = cal_gradient(W_obj, hidden_out, action_coding);
    
    %% estimate target_action_value
    % 使用旧的 W_obj_copy 来计算 target 值
    
    if rs(i) > 0
        target_action_value = -rs(i);
    else
%         [~, notch_set] = get_possible_notch_set(W_obj_copy, action);
        hidden_out_ = cal_hidden_output(W_obj_copy, next_sv_coding);
        vals = cal_final_output(W_obj_copy, hidden_out_{end}, Acode_(:, :, i));
        target_action_value = rs(i) + W_obj_copy.gamma * max(vals);
    end
    
    %% calculate delta
    
    delta = (target_action_value - current_action_value);
    % delta(delta > 1) = 1;
    % delta(delta < -1) = -1;
    delta = W_obj.alpha * delta;
    
%     Recordr(i, :) = [target_action_value, current_action_value, delta];
    
    %% 随机梯度下降
    
    W_obj.Wb{1, 1}(:, current_sv_coding) = W_obj.Wb{1, 1}(:, current_sv_coding) + delta * gradient_{1, 1}; %; repmat(dW1, 1, W_obj.nTiling);
    W_obj.Wb{1, 2} = W_obj.Wb{1, 2} + delta* gradient_{1, 1};    % because dW1s is the same as db1s
    
    for ntr = 2:W_obj.nLayer
        W_obj.Wb{ntr, 1} = W_obj.Wb{ntr, 1} + delta * gradient_{ntr, 1};
        W_obj.Wb{ntr, 2} = W_obj.Wb{ntr, 2} + delta * gradient_{ntr, 2};
    end
    
    W_obj.Wb{W_obj.nLayer + 1, 1}(action_coding, :) =  W_obj.Wb{W_obj.nLayer + 1, 1}(action_coding, :) ...
        + delta* gradient_{W_obj.nLayer + 1, 1}';
    W_obj.Wb{W_obj.nLayer + 1, 2} = W_obj.Wb{W_obj.nLayer + 1, 2} + delta * gradient_{W_obj.nLayer + 1, 2};
    
    %     W_obj.W(N_ind, current_ind_sv(:, i)) = W_obj.W(N_ind, current_ind_sv(:, i)) + ...
    %         W_obj.alpha * (target_action_value - current_action_value);
end



%% 
























function W_obj = get_Wobj_4HeavyHaul_linear(rangS, rangV, detaS, detaV, nTiling, nl, nPosNotch, nNegNotch)
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
detaA = 5;
nTile = 5;
nAcodeGrid = round(nNotch/detaA)+1;
nAG = nAcodeGrid^2;
code_offset = (0:nTile-1)' + 0.5;

W_obj.rangA = rangA;
W_obj.detaA = detaA;
W_obj.nTile = nTile;
W_obj.nAG = nAG;
W_obj.nAcodeGrid = nAcodeGrid;
W_obj.code_offset = code_offset;
W_obj.action_tilingIndex = (0:nTile-1)';

AllActionCode = zeros(nTile, nAction);
for itr = 1:nAction
    AllActionCode(:, itr) = coding_action_transform(W_obj, ActionSet(:, itr));
end

W_obj.AllActionCode = AllActionCode;

%% 
% nl = 2;
W = rand(nAG*nTile, nSV*nTiling)*0e-2;  % initialization 

%% 
W_obj.G = my_ndgrid(nl, 1:3);

%% 

W_obj.rangS = rangS; % unit m
W_obj.rangV = rangV;  % unit km/h

W_obj.detaS = detaS; % unit m
W_obj.detaV = detaV;  % unit km/h
W_obj.nTiling = nTiling;
W_obj.tile_rand = tile_rand;
W_obj.W = W;
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

%% operations

W_obj.coding_index_transform = @coding_index_transform;
W_obj.coding_action_transform = @coding_action_transform;

W_obj.index_tranform = @index_tranform;
W_obj.choose_next_action = @choose_next_action;
W_obj.update_ActionFunction = @update_ActionFunction;
W_obj.max_Q_value_of_State = @max_Q_value_of_State;

% W_obj.cal_hidden_output = @cal_hidden_output;
W_obj.cal_final_output = @cal_final_output;
% W_obj.cal_gradient = @cal_gradient;

W_obj.action_Q_value_of_State = @action_Q_value_of_State;

%%
function action_set = choose_next_action(W_obj, current_state, current_action, epsilon)
%% coding 

current_ind_sv = coding_index_transform(W_obj, current_state);
action_set = zeros(size(current_action));

%% 
for i = 1:size(action_set, 2)
    
    notch = current_action(:, i);
    
    [notch_set, N_ind] = get_possible_notch_set(W_obj, notch);
        
    if rand > epsilon    % ------ greedy
        vals = cal_final_output(W_obj, current_ind_sv(:, i), N_ind);
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
function out = cal_final_output(W_obj, code_, Ncode)
% 只能单个单个算了

if nargin == 2
    Ncode = W_obj.AllActionCode;
end
n = size(Ncode, 2);
if n == 1
    out = sum(sum(W_obj.W(Ncode, code_)));
else
    out = zeros(1, n);
    for itr = 1:n
        out(itr) = sum(sum(W_obj.W(Ncode(:, itr), code_)));
    end
end

% if nargin == 2
%     out = sum(W_obj.W(: ,code_), 2);
% elseif nargin == 3
%     out = sum(W_obj.W(N, code_), 2); 
% end

%%
function ind_sv = coding_index_transform(W_obj, state)

s = ceil((state(1, :) - W_obj.tile_rand(:, 1) - W_obj.rangS(1))/W_obj.detaS);
v = ceil((state(2, :) - W_obj.tile_rand(:, 2) - W_obj.rangV(1))/W_obj.detaV);

ind_sv = W_obj.tilingIndex*ones(1, size(s, 2)) * W_obj.nSV + s*W_obj.nV + v + 1;


function ind_action = coding_action_transform(W_obj, Np)

s = floor((Np(1, :) + W_obj.code_offset - W_obj.rangA(1))/W_obj.detaA);
v = floor((Np(2, :) + W_obj.code_offset - W_obj.rangA(1))/W_obj.detaA);

ind_action = W_obj.action_tilingIndex*ones(1, size(s, 2)) * W_obj.nAG + s*W_obj.nAcodeGrid + v + 1;
%% 

function [v, action] = max_Q_value_of_State(W_obj, state)

ind_sv = W_obj.coding_index_transform(W_obj, state); 

vals = W_obj.cal_final_output(W_obj, ind_sv);
% vals = W_obj.cal_final_output(W_obj, W_obj.cal_hidden_output(W_obj, ind_sv));
if nargin == 1
    v = max(vals);
else
    [v, ind] = max(vals);
    action = W_obj.ActionSet(ind);
end

%%
function ind_ = index_tranform(W_obj, notch_set)
ind_ = (notch_set(1, :) + W_obj.nNegNotch)*W_obj.nNotch + notch_set(2, :)+ W_obj.nNegNotch + 1;

%%
% tile coding for locaiton-speed space
function W_obj = update_ActionFunction(W_obj,  Qloc)

p_num = size(Qloc, 1);
% current_state = Qloc(:, 1:2)';
action_ind = Qloc(:, 3:2+W_obj.nl)';
% next_state = Qloc(:, 3+W_obj.nl:4+W_obj.nl)';
current_r = Qloc(:, 5+W_obj.nl)';

%% coding

current_ind_sv = coding_index_transform(W_obj, Qloc(:, 1:2)');

next_ind_sv = coding_index_transform(W_obj, Qloc(:, 3+W_obj.nl:4+W_obj.nl)');

%%

Recordr = zeros(p_num, 2);

%%

for i = 1:p_num
    %% current action (notch)
    %     notch = [10; 10];
    notch = action_ind(:, i);
    N_ind = coding_action_transform(W_obj, notch);
    
    %% calculate current_action_value
    % 使用新的 W_obj 来计算 target 值
    
    % out1 = W_obj.cal_hidden_output(W_obj, current_ind_sv(:, i));
    current_action_value = cal_final_output(W_obj, current_ind_sv(:, i), N_ind); 
    
    %% calculate gradient 
    
    %% estimate target_action_value
    % 使用旧的 W_obj_copy 来计算 target 值
    
    if current_r(i) > 0
        target_action_value = -current_r(i);
    else
        [~, notch_set] = get_possible_notch_set(W_obj, notch);
        vals = cal_final_output(W_obj, next_ind_sv(:, i), notch_set);
%         vals = cal_final_output(W_obj_copy, next_ind_sv(:, i), notch_set);
        target_action_value = current_r(i) + W_obj.gamma * max(vals);
    end
    
    %% 
    
    Recordr(i, :) = [target_action_value, current_action_value];
    
    W_obj.W(N_ind, current_ind_sv(:, i)) = W_obj.W(N_ind, current_ind_sv(:, i)) + ...
        W_obj.alpha * (target_action_value - current_action_value);
end

if nargout == 2 
end


%% 
























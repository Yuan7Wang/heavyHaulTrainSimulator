function W_obj = get_Wobj_linear_Nonlinear(nNodes, maxS, maxV, detaS, detaV, nTiling, nl, nPosNotch, nNegNotch)

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
tile_rand(end, :) = [1 1];

tile_rand(:, 1) = tile_rand(:, 1)*detaS;
tile_rand(:, 2) = tile_rand(:, 2)*detaV;

% % input range(S) = [0 2000]; range(V) = [0 120];

nS = ceil(maxS/detaS);
nV = ceil(maxV/detaV);
nNotch = nPosNotch + nNegNotch + 1;

% nl = 2;
W1 = rand(nNodes, nS*nV*nTiling)*1e-2;  % initialization weight
W2 = rand(nNotch^nl, nNodes)*1e-2;  % initialization weight

b1 = rand(nNodes, 1)*1e-2;  % initialization bais
b2 = rand(nNotch^nl, 1)*1e-2;  % initialization bais
%% 

W_obj.G = my_ndgrid(nl, 1:3);

%% 

W_obj.maxS = maxS; % unit m
W_obj.maxV = maxV;  % unit km/h

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
W_obj.nl = nl;
W_obj.nSV = nS*nV;
W_obj.nNodes = nNodes;
W_obj.tilingIndex = (0:nTiling-1)';

W_obj.nPosNotch = nPosNotch;
W_obj.nNegNotch = nNegNotch;
W_obj.nNotch = nNotch;
W_obj.nAction = nNotch^nl;

% W_obj.nV = (0:nTiling-1)'*detaS*detaV;


%% operation

W_obj.cal_hidden_output = @cal_hidden_output;
W_obj.cal_final_output = @cal_final_output;
W_obj.cal_gradient = @cal_gradient;
W_obj.ind_t = @index_tranform;
% W_obj.funTheta = @funTheta;

function v = sigmod(x)
v = 1./(1+exp(-x));


%%
function out = cal_hidden_output(W_obj, x_code)
out = sigmod(sum(W_obj.W1(:, x_code), 2) + W_obj.b1);


%%

function out = cal_final_output(W_obj, hidden_out, N)
out = W_obj.W2(N, :)*hidden_out + W_obj.b2(N);


%%
function ind_ = index_tranform(W_obj, notch_set)
ind_ = (notch_set(1, :) + W_obj.nNegNotch)*W_obj.nNotch + notch_set(2, :)+ W_obj.nNegNotch + 1;


%%  

function [dW1, dW2, db2] = cal_gradient(W_obj, N , out1)

dW2 = out1;
db2 = 1;
s_1 = diag(out1.*(1-out1))*W_obj.W2(N, :)';
dW1 = s_1;
% db1 = s_1;














%%



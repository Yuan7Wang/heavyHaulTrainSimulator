function W_obj = get_Wobj_linear(maxS, maxV, detaS, detaV, nTiling, nl, nPosNotch, nNegNotch)

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
W = rand(nS*nV*nTiling, nNotch^nl)'*0e-1;  % initialization 

%% 

W_obj.maxS = maxS; % unit m
W_obj.maxV = maxV;  % unit km/h

W_obj.detaS = detaS; % unit m
W_obj.detaV = detaV;  % unit km/h
W_obj.nTiling = nTiling;
W_obj.tile_rand = tile_rand;
W_obj.W = W;
W_obj.nS = nS;
W_obj.nV = nV;
W_obj.nSV = nS*nV;
W_obj.tilingIndex = (0:nTiling-1)';

W_obj.nPosNotch = nPosNotch;
W_obj.nNegNotch = nNegNotch;
W_obj.nNotch = nNotch;
W_obj.nAction = nNotch^nl;

% W_obj.nV = (0:nTiling-1)'*detaS*detaV;



function W_obj = get_Wobj_linear_Fit(maxS, maxV, detaS, detaV, nTiling, nl, nPosNotch, nNegNotch)

% detaS = 200; % unit m
% detaV = 20;  % unit km/h
% maxS = 2000; % unit m
% maxV = 120;  % unit km/h
% nTiling = 10;  % number of tiling 
% nPosNotch = 12;  % number of tiling 
% nNegNotch = 12;  % number of tiling 
% nl = 3;
% nNotch = 25;

% W_obj = get_Wobj_linear(maxS, maxV, detaS, detaV, nTiling, nl, nNotch)

%% 
tile_rand = rand(nTiling, 2).*(ones(nTiling, 1)*[detaS detaV]);
% % input range(S) = [0 2000]; range(V) = [0 120];

nS = ceil(maxS/detaS);
nV = ceil(maxV/detaV);
nNotch = nPosNotch + nNegNotch + 1;

W_obj.nTheta = nl + nl + 2;

% Aind = zeros(nl, 1);

% W_obj.A = reshape((1:16), 4, []);

W_obj.Aind = (1:(nl+1):nl^2)';
W_obj.m_ind = (1:nl)';
W_obj.A_ind = ((nl+1) : nl*2)';
W_obj.c_ind = (nl*2+1:nl*2+2)';

% disp(reshape((1:16), 4, []))

%% 

% nl = 3;
W = (rand(nS*nV*nTiling, W_obj.nTheta)' - 0.5)*1e-5;  % initialization 

W_obj.G = my_ndgrid(nl, 1:3);

%% 

W_obj.detaS = detaS; % unit m
W_obj.detaV = detaV;  % unit km/h
W_obj.nTiling = nTiling;
W_obj.tile_rand = tile_rand;
W_obj.W = W;
W_obj.nl = nl;
W_obj.nS = nS;
W_obj.nV = nV;
W_obj.nSV = nS*nV;
W_obj.tilingIndex = (0:nTiling-1)';

W_obj.nPosNotch = nPosNotch;
W_obj.nNegNotch = nNegNotch;
W_obj.nNotch = nNotch;
W_obj.nAction = nNotch^nl;

% W_obj.nV = (0:nTiling-1)'*detaS*detaV;


%% operation

W_obj.getTheta = @getTheta;
W_obj.funTheta = @funTheta;
W_obj.cal_gradient_Ntheta = @cal_gradient_Ntheta;

function theta = getTheta(W_obj, x_code)
theta = sum(W_obj.W(:, x_code), 2);

function val = funTheta(W_obj, theta, N)
% N 可以为矩阵，每一列一个Notch组

miu = theta(W_obj.m_ind);
ad = theta(W_obj.A_ind);
c = theta(W_obj.c_ind);

c(1) = 1 + c(1)^2;

A = diag(ad.^2);

val = zeros(size(N, 2), 1);
for itr = 1:length(val)
    val(itr) = c(1)*exp(-0.001*(N(:, itr) - miu)'*A*(N(:, itr)-miu)) + c(2);
end
% val = (N - miu)'*A*(N-miu) + c;

%%  
function dTheta = cal_gradient_Ntheta(W_obj, N, theta)

% 特指对 size(N) = [2, 1] 的情况
% N = [10 10]';
% theta = (1:6)';

% k = size(N, 1);
miu = theta(W_obj.m_ind);
ad = theta(W_obj.A_ind);
c = theta(W_obj.c_ind);

c_ = 1 + c(1)^2;

A = diag(ad.^2);

dTheta = zeros(size(theta));

N_ = N-miu;

expf_N = exp(-0.001*N_'*A*N_);

dTheta(W_obj.m_ind) = 0.001*2*A*N_*c_*expf_N;

dTheta(W_obj.A_ind) =  -0.001*c_*expf_N*N_.^2*2;     % 非对角线上 

dTheta(W_obj.c_ind) = [expf_N * 2 * c(1); 1];





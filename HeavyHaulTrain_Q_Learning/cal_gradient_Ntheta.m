function dTheta = cal_gradient_Ntheta(W_obj, N, theta)

% N = [10 10]';
% theta = (1:6)';

%% 
% k = size(N, 1);
miu = theta(W_obj.m_ind);
as = theta(W_obj.A_ind);

A = zeros(W_obj.nl);
A(W_obj.Aind) = as;
A = A + A' - diag(diag(A));

%% 

dTheta = zeros(size(theta));
dTheta(W_obj.m_ind) = -2*A*(N-miu);

dA = (N-miu)*(N-miu)';
dTheta(W_obj.Aind_0) = 2 * dA(W_obj.Aind0);     % 非对角线上
dTheta(W_obj.Aind_1) = dA(W_obj.Aind1);         % 对角线上

dTheta(W_obj.c_ind) = 1;


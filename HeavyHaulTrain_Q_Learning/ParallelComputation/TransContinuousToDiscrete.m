function [A_discrete,B_discrete] = TransContinuousToDiscrete(A_continuous,B_continuous,sampling_time)

%% Computation for A_discrete.
A_discrete = zeros(size(A_continuous));
F = eye(size(A_continuous));
k = 1;
while norm(A_discrete+F-A_discrete,1)>0
    A_discrete = A_discrete+F;
    F = sampling_time*A_continuous*F/k;
    k = k+1;
end

%% Computation for B_discrete.
B_discrete = zeros(size(A_continuous));
F = sampling_time*eye(size(A_continuous));
k = 2;
while norm(B_discrete+F-B_discrete,1)>0
    B_discrete = B_discrete+F;
    F = sampling_time*A_continuous*F/k;
    k = k+1;
end
B_discrete = B_discrete*B_continuous;



theta = [7; 8; 2; 3; 1;0];

%% 

vec_range = [-12; 30];
rvec = vec_range(1):vec_range(2);

[Mx, My] = meshgrid(rvec);

notchs = [Mx(:) My(:)]';


% figure(1);clf
% mesh(Mx, My, val)


%% 

len = 1e3;
N_scatter = randn(2, len)*10+theta(1:2);

val2 = W_obj.funTheta(W_obj, theta, N_scatter);

inputSet = [N_scatter' val2 + randn(size(val2))*1e-3];

figure(1);clf
hold on;
val = reshape(W_obj.funTheta(W_obj, theta, notchs), length(rvec), []);
mesh(Mx, My, val)
plot3(inputSet(:, 1), inputSet(:, 2), inputSet(:, 3), 'r.');
view([-20 20]);

%% 

alpha = 5e-1;

% theta_ = randn(size(theta))*5;
theta_  = theta + randn(size(theta)).*[10 10 2 2 0.5 0.5]';

figure(2);clf
hold on;
plot3(theta_(1), theta_(2), 2, 'ko')

for itr = 1:len*50
    
    ind = ceil(rand*len);
    
    N = inputSet(ind, 1:2)';
    
    tmp = W_obj.funTheta(W_obj, theta_, N);    
    dTheta = W_obj.cal_gradient_Ntheta(W_obj, N, theta_);
    
    dTheta = dTheta/norm(dTheta);
    
    theta_ = theta_ + alpha*(inputSet(ind, 3) - tmp)*dTheta;
end

disp('theta_ = ')
disp(theta_)

[Mx, My] = meshgrid(rvec);
notchs = [Mx(:) My(:)]';
val = reshape(W_obj.funTheta(W_obj, theta_, notchs), length(rvec), []);

mesh(Mx, My, val)
plot3(inputSet(:, 1), inputSet(:, 2), inputSet(:, 3), 'r.');
view([-20 20]);
















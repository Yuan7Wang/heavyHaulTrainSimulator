

W_obj = W_OBJ{3};

s = linspace(W_obj.rangS(1), W_obj.rangS(2), 400);
v = linspace(W_obj.rangV(1), W_obj.rangV(2), 50);
[Ms, Mv] = meshgrid(s, v);
% stateSet = [Ms(:), Mv(:)]';

current_ind_sv = W_obj.coding_index_transform(W_obj, [Ms(:), Mv(:)]');

Val = zeros(size(Ms));
% Val = W_obj.max_Q_value_of_State(W_obj, [Ms(:), Mv(:)]');
Np = [5; 5];
N_ind = W_obj.coding_action_transform(W_obj, Np);
for itr = 1:numel(Ms)
    Val(itr) = W_obj.action_Q_value_of_State(W_obj, current_ind_sv(:, itr), N_ind); 
end

% Val(Val < -120) = -400; 

% figure(10001);clf
% hmesh = mesh(Ms, Mv, ((Val)));
% view(-23, 52)
% set(gca, 'zdir', 'reverse')

figure(10002);clf
hold on;
contourf(Ms, Mv, Val, 40, 'linecolor', [1 1 1]*1);
colorbar
plot(x_, y_, 'k', 'linewidth', 3);

position_val = [5000 15000 30000 35000];
position_val(end+1, :) = 60/3.6;

current_ind_SomeStateS = zeros(W_obj.nTiling, 4);
for itr = 1:4
    current_ind_SomeStateS(:, itr) = W_obj.coding_index_transform(W_obj, position_val(:, itr));
end

n_contour = 20;
figure(1003);clf
for itr = 1:4
    subplot(2, 2, itr)
    hidden_out_ = W_obj.cal_hidden_output(W_obj, current_ind_SomeStateS(:, itr));
    val = reshape(W_obj.cal_final_output(W_obj, hidden_out_{end}), 25, []);
    contourf(W_obj.Ax, W_obj.Ay, val, n_contour, 'linecolor', [1 1 1]*1);
    title(position_val(1, itr));
    colorbar
end
drawnow;
pause(1e-10);
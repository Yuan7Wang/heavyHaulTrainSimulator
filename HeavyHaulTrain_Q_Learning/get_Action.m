function action = get_Action(Q_, epsilon, MotorNotch)

a_101 = [-1; 0; 1];

a = a_101 + MotorNotch(1) + 13;
b = a_101 + MotorNotch(2) + 13;

a(a > 25) = 25;
a(a < 1) = 1;
b(b > 25) = 25;
b(b < 1) = 1;

if rand > epsilon    % ------ greedy
    q_ = Q_(a, b);
    [~, ind] = max(q_(:));
    ind = ind(1);
else                 % ------ random
    ind = ceil(rand*9);
end

ind1 = rem(ind, 3); ind1(ind1 == 0) = 3;
ind2 = ceil(ind/3);
% action = [a(ind2); b(ind1)] - 13;
action = [a(ind1); b(ind2)] - 13;

% MotorNotch = MotorNotch + a_101([ind2 ind1]); 
% MotorNotch(1) = MotorNotch(1) + a_101(ind2);
% MotorNotch(2) = MotorNotch(2) + a_101(ind1);

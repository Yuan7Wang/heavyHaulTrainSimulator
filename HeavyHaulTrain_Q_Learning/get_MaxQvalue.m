function max_ = get_MaxQvalue(Q_, MotorNotch)

a_101 = [-1; 0; 1];

a = a_101 + MotorNotch(1);
b = a_101 + MotorNotch(2);

a(a > 25) = 25;
a(a < 1) = 1;
b(b > 25) = 25;
b(b < 1) = 1;
q_ = Q_(a, b);

max_ = max(q_(:));
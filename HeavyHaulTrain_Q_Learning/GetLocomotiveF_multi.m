function  Fvec = GetLocomotiveF_multi(V, MotorNotch, TBcl)

% Ne = size(V, 2);
% Fvec = zeros(size(V));

% V = X(carType, :)*3.6;
s = size(V);
V_ = V(:);
MotorNotch_ = MotorNotch(:) + 13;

speed_pos = sum(V_(:) - TBcl(MotorNotch_(:), :, 1) >= 0, 2);
x = TBcl(MotorNotch_(:), speed_pos, 1);
y = TBcl(MotorNotch_(:), speed_pos, 2);
dy = TBcl(MotorNotch_(:), speed_pos, 3);

Fvec = reshape(diag(y)+diag(dy).*(V_ - diag(x)), s(1), s(2));

% TBcl = MatrixTBCL_force;













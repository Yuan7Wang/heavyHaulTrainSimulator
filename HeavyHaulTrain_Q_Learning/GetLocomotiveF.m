function  Fvec = GetLocomotiveF(V, MotorNotch, TBcl)

% Ne = size(V, 2);
Fvec = zeros(size(V));

for itr = 1:size(V, 2)
    for jtr = 1:size(V, 1)
        Fvec(jtr, itr) = TBcl{MotorNotch(jtr, itr)+13}(V(jtr, itr));
    end
end

















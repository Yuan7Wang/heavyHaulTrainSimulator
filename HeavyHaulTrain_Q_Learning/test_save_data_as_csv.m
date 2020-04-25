
% load('TCL_force');   % 12 cells contain the locomotive traction characteristics Force(kN)-Velocity(km/h)
% load('BCL_force');   % 12 cells contain the locomotive regenerative brake characteristics Force(kN)-Velocity(km/h)

load('TCL_forceHXD3.mat');   % 12 cells contain the locomotive traction characteristics Force(kN)-Velocity(km/h)
load('BCL_forceHXD3');   % 12 cells contain the locomotive regenerative brake characteristics Force(kN)-Velocity(km/h)
load('MatrixTBCL_forceHXD3.mat')

Tcl = zeros(12, 12*2);
for itr = 1:12
    Tcl(:, (itr-1)*2+1:itr*2) = TCL_forceHXD3{itr};
end

Bcl = zeros(12, 12*2);
for itr = 1:12
    Bcl(:, (itr-1)*2+1:itr*2) = BCL_forceHXD3{itr}';
end

Tbcl = [Bcl(:, end:-1:1) Tcl];

txtwrite('TBCL_force.csv', Tbcl);



txtwrite('Ad.csv', Ad);
txtwrite('Bd.csv', Bd);

% locmotive_info = zeros(202, 2);
locmotive_info = [mTrainGroup(:, 1) LenTrain'];
txtwrite('locmotive_info.csv', locmotive_info, {'mass (Kg)', 'length (m)'});

basicF_par = [C0(:, 1) Ca(:, 1)];
coupler_par = [KK(:, 1) DD(:, 1)];

txtwrite('basicF_par.csv', basicF_par, {'C_0', 'C_a'});
txtwrite('coupler_par.csv', coupler_par, {'Stiffness (N/m)', 'Damping (N.s/m)'});


load MatrixTBCL_force.mat MatrixTBCL_force 
txtwrite('MatrixTBCL_force_0.csv', MatrixTBCL_force(:, :, 1));
txtwrite('MatrixTBCL_force_1.csv', MatrixTBCL_force(:, :, 2));
txtwrite('MatrixTBCL_force_2.csv', MatrixTBCL_force(:, :, 3));


% MatrixTBCL_force__ = reshape(MatrixTBCL_force_, 25, 12, 3); 








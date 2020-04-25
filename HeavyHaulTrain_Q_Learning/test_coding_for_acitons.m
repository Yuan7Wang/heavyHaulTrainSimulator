% test_coding_for_acitons.m



%% 

rangA = [-12 12];

nNotch = length(rangA(1):rangA(2));

detaA = 5;
nTile = 5;
nAcodeGrid = round(nNotch/detaA)+1;
nAG = nAcodeGrid^2;
code_offset = (0:nTile-1) + 0.5;

% Np = [12; 12];
% Np = -[12; 12];
s = ceil((Np(1, :) + W_obj.code_offset - W_obj.rangA(1))/W_obj.detaA);
v = ceil((Np(2, :) + W_obj.code_offset - W_obj.rangA(1))/W_obj.detaA);

ind_sv = W_obj.nTile*ones(1, size(s, 2)) * W_obj.nAG + s*W_obj.nAcodeGrid + v + 1;











%% 









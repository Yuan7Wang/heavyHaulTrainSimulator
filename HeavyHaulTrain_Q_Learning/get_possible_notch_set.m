function [notch_set, ind_] = get_possible_notch_set(W_obj, notch)
%% 
% notch = [7 12 12]';

%% notch is a colume vector

notch_set = zeros(3^W_obj.nl, W_obj.nl);

%%

% A=[a(:),b(:),c(:),d(:)];

a_101 = [-1; 0; 1];
for itr = 1:size(notch, 1)
    a = a_101 + notch(itr);
    a(a > W_obj.nPosNotch) = W_obj.nPosNotch;
    a(a < -W_obj.nNegNotch) = -W_obj.nNegNotch;
    notch_set(:,itr) = a(W_obj.G(:, itr));
end

% notch_set = unique(notch_set,'rows')';

if nargout == 2
    ind_ = W_obj.ind_t(W_obj, notch_set);
%     ind_ = (notch_set(1, :) + W_obj.nNegNotch)*W_obj.nNotch + notch_set(2, :)+ W_obj.nNegNotch + 1;
end




function action_set = get_possible_action_set(W_obj, action, b2)
%% coding
if nargin == 2
    % action = 3*25+10;
    ind2 = rem(action, W_obj.nNotch); ind2(ind2 == 0) = W_obj.nNotch;
    ind1 = ceil(action/W_obj.nNotch);
elseif nargin == 3
    ind2 = b2 + W_obj.nNegNotch + 1;
    ind1 = action + W_obj.nNegNotch + 1;
end

%%

a_101 = [-1; 0; 1];

a = a_101 + ind1;
b = a_101 + ind2;

a(a > W_obj.nNotch) = W_obj.nNotch;
a(a < 1) = 1;
b(b > W_obj.nNotch) = W_obj.nNotch;
b(b < 1) = 1;
    

action_set = (reshape(b, 1, [])-1)*W_obj.nNotch + a;

action_set = action_set(:);






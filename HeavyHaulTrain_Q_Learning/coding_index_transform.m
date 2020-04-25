function ind_sv = coding_index_transform(W_obj, state)

s = ceil((state(1, :) - W_obj.tile_rand(:, 1))/W_obj.detaS);
v = ceil((state(2, :) - W_obj.tile_rand(:, 2))/W_obj.detaV);

ind_sv = W_obj.tilingIndex*ones(1, size(s, 2)) * W_obj.nSV + s*W_obj.nV + v + 1;







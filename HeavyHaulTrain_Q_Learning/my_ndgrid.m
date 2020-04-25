function output = my_ndgrid(nout, varargin)

% output = my_ndgrid(3, 1:3);

%%

% nout = max(nargout,nargin);
if nargin == 2
    if nout < 2
        output = varargin{1}(:);    
        return
    else
        j = ones(nout,1);
        siz(1:nout) = numel(varargin{1});
    end
else 
    j = 1:nout;
    siz = cellfun(@numel,varargin);
end

tmp = cell(1,max(nargout,1));
if nout == 2 % Optimized Case for 2 dimensions
    x = full(varargin{j(1)}(:));
    y = full(varargin{j(2)}(:)).';
    tmp{1} = repmat(x,size(y));
    tmp{2} = repmat(y,size(x));
else
    for i=1:nout
        x = full(varargin{j(i)});
        s = ones(1,nout); 
        s(i) = numel(x);
        x = reshape(x,s);
        s = siz; 
        s(i) = 1;
        tmp{i} = repmat(x,s);
    end
end

output = zeros(numel(tmp{1}), nout);
for itr = 1:nout
    output(:, itr) = tmp{itr}(:);
end




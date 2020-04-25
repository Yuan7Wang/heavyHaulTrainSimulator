% 2015/6/10 10:24
% 快速滑动滤波方法
function vec = get_mean(vec,N,NUM)

N = 2*floor(N/2);
if NUM == 0 || N == 0
    return;
end
for itr = 1:NUM
    vec = mean_3(vec,N);
end

function vecc = mean_3(vec,N)
vecc = zeros(size(vec));
len = length(vec);
for itr = 1:N/2+1
    vecc(itr) = mean(vec(1:itr+N/2));
    vecc(len-itr+1) = mean(vec(len-itr+1-N/2:len));
end

for itr = N/2+1:len-N/2-1
    vecc(itr+1) = vecc(itr)+(vec(itr+N/2+1)-vec(itr-N/2))/(N+1);
end
















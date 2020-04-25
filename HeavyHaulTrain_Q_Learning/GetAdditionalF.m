function [F, rempTrains] = GetAdditionalF(mTrainGroup, pTrains, rampList)

rempTrains = zeros(size(pTrains));

mm = [min(pTrains(:)), max(pTrains(:))];

[~, indmm(1)] = min(abs(rampList(:, 1) - mm(1)));
[~, indmm(2)] = min(abs(rampList(:, 1) - mm(2)));
indmm = indmm+[-1 1];
indmm(indmm < 1) = 1;
for ptr = indmm(1):indmm(2)
    rempTrains(pTrains >= rampList(ptr, 1) & pTrains < rampList(ptr+1, 1)) = rampList(ptr, 2);
end

F = -mTrainGroup*9.8.*rempTrains;

if nargin == 2 
end

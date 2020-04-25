% 计算空气制动力

function [currentF, F_target] = GetAirBrakeF(currentF, F_target, AirBrkNotch, AirBrkNotch_, air_time_d, air_time_d_, Abr, dt)

Nw = size(currentF, 1);

notchF_ = ones(Nw, 1)*AirBrkNotch_*1e1;  % 原来的目标力
notchF = ones(Nw, 1)*AirBrkNotch*1e1;    % 新来的目标力


tmpBo = air_time_d_ > 0;
F_target(tmpBo) = notchF_(tmpBo);

tmpBo = air_time_d > 0;
F_target(tmpBo) = notchF(tmpBo);

tmpBo = currentF > F_target;
currentF(tmpBo) = currentF(tmpBo) + Abr.Alpha1(tmpBo).*(currentF(tmpBo)+20)*dt;  % break force increase

tmpBo = currentF < F_target;
currentF(tmpBo) = currentF(tmpBo) + Abr.Alpha2(tmpBo).*currentF(tmpBo)*dt;      % break force release









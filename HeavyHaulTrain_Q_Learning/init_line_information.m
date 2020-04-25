% init_line_information.m


%% Load the infrastructure
load('RampPositionList');  % 1.Format:start(m) end(m) gradient, 2.Range: [-10000m,100000m]
load('RampPoint'); % 1.Format:position(m) Height(m),  2.Range:[9000m,37972m]

RampList = RampPositionList(:, [1 3]);











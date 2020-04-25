%% PART II:INITIALIZE Q Learning Parameters.
%% Load the grid like tilings
NumOfTilings = 1;
SupremumOfDistance = 9000;   %m
InfimumOfDistance = 0;
SupremumOfVelocity = 105/3.6; %m/s
InfimumOfVelocity = 0;        %m/s
NumOfGridDistance = 30;
NumOfGridVelocity = 30;
NumOfTiles = NumOfGridVelocity*NumOfGridDistance;
GridWidthDistance = (SupremumOfDistance-InfimumOfDistance)/(NumOfGridDistance-1);
GridWidthVelocity = (SupremumOfVelocity-InfimumOfVelocity)/(NumOfGridVelocity-1);
load('GridLikeTilings');

%% Exploring Start Setting.
% StartDistance = 0:GridWidthDistance:SupremumOfDistance-GridWidthDistance;
% StartVelocity = GridWidthVelocity*7:GridWidthVelocity:SupremumOfVelocity-7*GridWidthVelocity;
% NumofStartDistance = length(StartDistance);
% NumofStartVelocity = length(StartVelocity);
% NumofStartPoint = NumofStartDistance*NumofStartVelocity;

%%
% [X,Y] = meshgrid(3.6*StartVelocity,StartDistance);
% Z = X.*Y*0;
% mesh(X,Y,Z,'linewidth',1);
% alpha(0);
% view([0 90])
% hold on
% pause(0.5);

%% Transform the GridLikeTiling into Matrix for computation.
GridVelocityMatrix = zeros(NumOfTilings,NumOfGridVelocity+1);
GridDistanceMatrix = zeros(NumOfTilings,NumOfGridDistance+1);
for i = 1:NumOfTilings
    GridVelocityMatrix(i,:) = GridLikeTilings{i}(1,:);
    GridDistanceMatrix(i,:) = GridLikeTilings{i}(2,:);
end

%% Initilization for the Q Learning Training.
NumofEpisodes = 100;
FullStepsPerEpisode = 1;
NumofStepsPerEpisode = FullStepsPerEpisode;  % maximum value for j
% Seed = rand(1,NumofEpisodes*NumofStepsPerEpisode);
RewardDiscount = 1;
% NotchSet = 12:-1:-12;
% NumofNotches = length(NotchSet);
% ActionSet = 1:-1:-1;
% NumOfActions = length(ActionSet);
% NumOfFeatures = NumOfGridDistance*NumOfGridVelocity*NumOfTilings;
% CurrentReward = 0;
% alpha = 0.05*0.1/NumOfTilings;
% alpha = 0.8;
% epsilon = 0.1;
Seed = rand(1,NumofEpisodes*NumofStepsPerEpisode);
Seed1 = rand(1,NumofEpisodes);
Seed2 = rand(1,NumofEpisodes);
Seed3 = rand(1,NumofEpisodes);
CurrentTime = 0;
% CurrentStateFeature = zeros(NumOfFeatures,1);
AfterStateFeature = 0;
%% Get Tile coding.
clc;
clear all
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
BaseGrid = [InfimumOfVelocity:GridWidthVelocity:SupremumOfVelocity+GridWidthVelocity
    InfimumOfDistance:GridWidthDistance:SupremumOfDistance+GridWidthDistance];
GridOffset = [GridWidthVelocity*rand(NumOfTilings,1) GridWidthDistance*rand(NumOfTilings,1)];
GridLikeTilings = cell(1,NumOfTilings);

[X,Y] = meshgrid(3.6*BaseGrid(1,1:end-1),BaseGrid(2,1:end-1));
Z = X.*Y*0+1;
surf(X,Y,Z,'linewidth',2);
view([0 90])
alpha(0);

for i = 1:NumOfTilings
    GridLikeTilings{i} = [BaseGrid(1,:)-GridOffset(i,1);BaseGrid(2,:)-GridOffset(i,2)];
    [X,Y] = meshgrid(3.6*GridLikeTilings{i}(1,:),GridLikeTilings{i}(2,:));
    Z = X.*Y*0;
    mesh(X,Y,Z,'linewidth',1);
    alpha(0);
    view([0 90])
    hold on
    pause(0.5);
end

%% Save the GridLikeTilings for multiple tests.
% save('GridLikeTilings','GridLikeTilings');
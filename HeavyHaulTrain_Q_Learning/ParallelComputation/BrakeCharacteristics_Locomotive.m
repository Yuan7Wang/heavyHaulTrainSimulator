clc;
clear all
%% Locomotive Braking.
load('BrakeCharacteristic1');load('BrakeCharacteristic2');load('BrakeCharacteristic3');
load('BrakeCharacteristic4');load('BrakeCharacteristic5');load('BrakeCharacteristic6');
load('BrakeCharacteristic7');load('BrakeCharacteristic8');load('BrakeCharacteristic9');
load('BrakeCharacteristic10');load('BrakeCharacteristic11');load('BrakeCharacteristic12');

BCL_force = cell(1,12);

BCL_force{12} = [BrakeCharacteristic12(:,1) BrakeCharacteristic12(:,2)];
BCL_force{12} = [BCL_force{12};1000 0];
BCL_force{12}(11,1) = 122;
BCL_force{12}(11,2) = 0;
BCL_force{12}(2,1) = 0.2;
BCL_force{12}(2,2) = 40;

BCL_force{11} = [BrakeCharacteristic11(:,1) BrakeCharacteristic11(:,2)];
BCL_force{11} = [BCL_force{11};130 0;1000 0];
BCL_force{11}(10,1) = 122;
BCL_force{11}(10,2) = 0;
BCL_force{11}(2,1) = 0.2;
BCL_force{11}(2,2) = 40;

BCL_force{10} = [BrakeCharacteristic10(:,1) BrakeCharacteristic10(:,2)];
BCL_force{10} = [BCL_force{10};130 0;140 0;1000 0];
BCL_force{10}(9,1) = 122;
BCL_force{10}(9,2) = 0;
BCL_force{10}(2,1) = 0.2;
BCL_force{10}(2,2) = 40;

BCL_force{9} = [BrakeCharacteristic9(:,1) BrakeCharacteristic9(:,2)];
BCL_force{9} = [BCL_force{9};130 0;140 0;150 0;1000 0];
BCL_force{9}(8,1) = 122;
BCL_force{9}(8,2) = 0;
BCL_force{9}(2,1) = 0.2;
BCL_force{9}(2,2) = 40;

BCL_force{8} = [BrakeCharacteristic8(:,1) BrakeCharacteristic8(:,2)];
BCL_force{8} = [BCL_force{8};130 0;140 0;150 0;160 0;1000 0];
BCL_force{8}(7,1) = 122;
BCL_force{8}(7,2) = 0;
BCL_force{8}(2,1) = 0.2;
BCL_force{8}(2,2) = 40;

BCL_force{7} = [BrakeCharacteristic7(:,1) BrakeCharacteristic7(:,2)];
BCL_force{7} = [BCL_force{7};130 0;140 0;150 0;160 0;170 0;180 0;1000 0];
BCL_force{7}(5,1) = 122;
BCL_force{7}(5,2) = 0;
BCL_force{7}(2,1) = 0.2;
BCL_force{7}(2,2) = 40;

BCL_force{6} = [BrakeCharacteristic6(:,1) BrakeCharacteristic6(:,2)];
BCL_force{6} = [BCL_force{6};130 0;140 0;150 0;160 0;170 0;180 0;1000 0];
BCL_force{6}(5,1) = 122;
BCL_force{6}(5,2) = 0;
BCL_force{6}(2,1) = 0.2;
BCL_force{6}(2,2) = 40;

BCL_force{5} = [BrakeCharacteristic5(:,1) BrakeCharacteristic5(:,2)];
BCL_force{5} = [BCL_force{5};130 0;140 0;150 0;160 0;170 0;180 0;1000 0];
BCL_force{5}(5,1) = 122;
BCL_force{5}(5,2) = 0;
BCL_force{5}(2,1) = 0.2;
BCL_force{5}(2,2) = 40;

BCL_force{4} = [BrakeCharacteristic4(:,1) BrakeCharacteristic4(:,2)];
BCL_force{4} = [BCL_force{4};130 0;140 0;150 0;160 0;170 0;180 0;1000 0];
BCL_force{4}(5,1) = 122;
BCL_force{4}(5,2) = 0;
BCL_force{4}(2,1) = 0.2;
BCL_force{4}(2,2) = 40;

BCL_force{3} = [BrakeCharacteristic3(:,1) BrakeCharacteristic3(:,2)];
BCL_force{3} = [BCL_force{3};130 0;140 0;150 0;160 0;170 0;180 0;1000 0];
BCL_force{3}(5,1) = 122;
BCL_force{3}(5,2) = 0;
BCL_force{3}(2,1) = 0.2;
BCL_force{3}(2,2) = 40;

BCL_force{2} = [BrakeCharacteristic2(:,1) BrakeCharacteristic2(:,2)];
BCL_force{2} = [BCL_force{2};130 0;140 0;150 0;160 0;170 0;180 0;1000 0];
BCL_force{2}(5,1) = 122;
BCL_force{2}(5,2) = 0;
BCL_force{2}(2,1) = 0.2;
BCL_force{2}(2,2) = 40;

BCL_force{1} = [BrakeCharacteristic1(:,1) BrakeCharacteristic1(:,2)];
BCL_force{1} = [BCL_force{1};130 0;140 0;150 0;160 0;170 0;180 0;190 0;1000 0];
BCL_force{1}(4,1) = 122;
BCL_force{1}(4,2) = 0;
BCL_force{1}(2,1) = 0.2;
BCL_force{1}(2,2) = 40;


for i = 1:12
   BCL_force{i}(1,1) = 0;
   BCL_force{i}(1,2) = 0;
   BCL_force{i}(:,2) = -BCL_force{i}(:,2);
   BCL_force{i} = BCL_force{i}';
end




%%
figure(1)
plot(BCL_force{1}(1,:),-BCL_force{1}(2,:),BCL_force{2}(1,:),-BCL_force{2}(2,:),BCL_force{3}(1,:),-BCL_force{3}(2,:),...
    BCL_force{4}(1,:),-BCL_force{4}(2,:),BCL_force{5}(1,:),-BCL_force{5}(2,:),BCL_force{6}(1,:),-BCL_force{6}(2,:),...
    BCL_force{7}(1,:),-BCL_force{7}(2,:),BCL_force{8}(1,:),-BCL_force{8}(2,:),BCL_force{9}(1,:),-BCL_force{9}(2,:),...
    BCL_force{10}(1,:),-BCL_force{10}(2,:),BCL_force{11}(1,:),-BCL_force{11}(2,:),BCL_force{12}(1,:),-BCL_force{12}(2,:),'linewidth',2);
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Velocity(km/h)','FontSize',12,'Fontname','Times New Roman')
ylabel('Motor Brake Force(kN)','FontSize',12,'Fontname','Times New Roman')
%legend('LG1 and WG1','WG50 and LG2')
axis([-10 130 0 600]);
grid on
% save('BCL_force','BCL_force');

%% 1-------velocity
%% 2-------force
%% 3-------gradient

MatrixTBCL_force = zeros(25,12,3); 
for i = 1:12
    MatrixTBCL_force(i,:,1) = BCL_force{12-i+1}(1,:); 
    MatrixTBCL_force(i,:,2) = BCL_force{12-i+1}(2,:); 
end
MatrixTBCL_force(1:12,1:end-1,3) = (MatrixTBCL_force(1:12,2:end,2)-MatrixTBCL_force(1:12,1:end-1,2))./...
    (MatrixTBCL_force(1:12,2:end,1)-MatrixTBCL_force(1:12,1:end-1,1));

%% Locomotive Traction.
% load('TractionCharacteristic1');load('TractionCharacteristic2');load('TractionCharacteristic3');
% load('TractionCharacteristic4');load('TractionCharacteristic5');load('TractionCharacteristic6');
% load('TractionCharacteristic7');load('TractionCharacteristic8');load('TractionCharacteristic9');
% load('TractionCharacteristic0');load('TractionCharacteristic11');load('TractionCharacteristic12');
% 
% TCL_force = cell(1,12);
% 
% TCL_force{12} = TractionCharacteristics12;
% TCL_force{12}(1,1) = 0;
% TCL_force{12}(1,2) = 380;
% TCL_force{12} = [TCL_force{12};110 0;1000 0];
% 
% TCL_force{11} = TractionCharacteristics11;
% TCL_force{11}(1,1) = 0;
% TCL_force{11}(1,2) = TCL_force{12}(1,2);
% TCL_force{11}(2,1) = TCL_force{12}(2,1);
% TCL_force{11}(2,2) = TCL_force{12}(2,2);
% TCL_force{11} = [TCL_force{11};110 0;1000 0];
% 
% TCL_force{10} = TractionCharacteristics10;
% TCL_force{10}(1,1) = 0;
% TCL_force{10}(1,2) = TCL_force{12}(1,2);
% TCL_force{10}(2,1) = TCL_force{12}(2,1);
% TCL_force{10}(2,2) = TCL_force{12}(2,2);
% TCL_force{10} = [TCL_force{10};110 0;120 0;1000 0];
% 
% TCL_force{9} = TractionCharacteristics9;
% TCL_force{9}(1,1) = 0;
% TCL_force{9}(1,2) = TCL_force{12}(1,2);
% TCL_force{9}(2,1) = TCL_force{12}(2,1);
% TCL_force{9}(2,2) = TCL_force{12}(2,2);
% TCL_force{9} = [TCL_force{9};110 0;1000 0];
% 
% TCL_force{8} = TractionCharacteristics8;
% TCL_force{8}(1,1) = 0;
% TCL_force{8}(1,2) = TCL_force{12}(1,2);
% TCL_force{8}(2,1) = TCL_force{12}(2,1);
% TCL_force{8}(2,2) = TCL_force{12}(2,2);
% TCL_force{8} = [TCL_force{8};110 0;1000 0];
% 
% TCL_force{7} = TractionCharacteristics7;
% TCL_force{7}(1,1) = 0;
% TCL_force{7}(1,2) = TCL_force{12}(1,2);
% TCL_force{7}(2,1) = TCL_force{12}(2,1);
% TCL_force{7}(2,2) = TCL_force{12}(2,2);
% TCL_force{7} = [TCL_force{7};110 0;120 0;1000 0];
% 
% TCL_force{6} = TractionCharacteristics6;
% TCL_force{6}(1,1) = 0;
% TCL_force{6}(1,2) = TCL_force{12}(1,2);
% TCL_force{6}(2,1) = TCL_force{12}(2,1);
% TCL_force{6}(2,2) = TCL_force{12}(2,2);
% TCL_force{6} = [TCL_force{6};110 0;120 0;130 0;140 0;1000 0];
% 
% TCL_force{5} = TractionCharacteristics5;
% TCL_force{5}(1,1) = 0;
% TCL_force{5} = [TCL_force{5};110 0;120 0;1000 0];
% 
% TCL_force{4} = TractionCharacteristics4;
% TCL_force{4}(1,1) = 0;
% TCL_force{4} = [TCL_force{4};110 0;120 0;1000 0];
% 
% TCL_force{3} = TractionCharacteristics3;
% TCL_force{3}(1,1) = 0;
% TCL_force{3} = [TCL_force{3};110 0;120 0;130 0;140 0;150 0;160 0;1000 0];
% 
% TCL_force{2} = TractionCharacteristics2;
% TCL_force{2}(1,1) = 0;
% TCL_force{2}(end-1,2) = 5;
% TCL_force{2} = [TCL_force{2};110 0;120 0;130 0;140 0;1000 0];
% 
% TCL_force{1} = TractionCharacteristics1;
% TCL_force{1}(1,1) = 0;
% TCL_force{1} = [TCL_force{1};110 0;120 0;130 0;140 0;150 0;1000 0];
% 
% for i = 1:12
%    TCL_force{i} = TCL_force{i}';
% end
% 
% %%
% figure(2)
% plot(TCL_force{1}(1,:),TCL_force{1}(2,:),TCL_force{2}(1,:),TCL_force{2}(2,:),TCL_force{3}(1,:),TCL_force{3}(2,:),...
%     TCL_force{4}(1,:),TCL_force{4}(2,:),TCL_force{5}(1,:),TCL_force{5}(2,:),TCL_force{6}(1,:),TCL_force{6}(2,:),...
%     TCL_force{7}(1,:),TCL_force{7}(2,:),TCL_force{8}(1,:),TCL_force{8}(2,:),TCL_force{9}(1,:),TCL_force{9}(2,:),...
%     TCL_force{10}(1,:),TCL_force{10}(2,:),TCL_force{11}(1,:),TCL_force{11}(2,:),TCL_force{12}(1,:),TCL_force{12}(2,:),'linewidth',2);
% set(gca,'FontSize',12,'Fontname','Times New Roman');
% xlabel('Velocity(km/h)','FontSize',12,'Fontname','Times New Roman')
% ylabel('Motor Traction Force(kN)','FontSize',12,'Fontname','Times New Roman')
% %legend('LG1 and WG1','WG50 and LG2')
% axis([0 1000 0 400]);
% grid on
% % save('TCL_force','TCL_force');

%%
load('TCL_force');
for i = 1:12
   TCL_force{i} = TCL_force{i}';
end
for i = 1:12
    MatrixTBCL_force(i+13,:,1) = TCL_force{i}(1,:); 
    MatrixTBCL_force(i+13,:,2) = TCL_force{i}(2,:); 
end
MatrixTBCL_force(11,:,1) = TCL_force{1}(1,:); 
MatrixTBCL_force(14:25,1:end-1,3) = (MatrixTBCL_force(14:25,2:end,2)-MatrixTBCL_force(14:25,1:end-1,2))./...
    (MatrixTBCL_force(14:25,2:end,1)-MatrixTBCL_force(14:25,1:end-1,1));
save('MatrixTBCL_force','MatrixTBCL_force');










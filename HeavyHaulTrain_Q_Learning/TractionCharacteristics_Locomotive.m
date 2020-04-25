clc;
% clear all
load('TractionCharacteristic1');load('TractionCharacteristic2');load('TractionCharacteristic3');
load('TractionCharacteristic4');load('TractionCharacteristic5');load('TractionCharacteristic6');
load('TractionCharacteristic7');load('TractionCharacteristic8');load('TractionCharacteristic9');
load('TractionCharacteristic10');load('TractionCharacteristic11');load('TractionCharacteristic12');

TCL_force = cell(1,12);

TCL_force{12} = TractionCharacteristic12;
TCL_force{12}(1,1) = 0;
TCL_force{12}(1,2) = 570;
TCL_force{12} = [TCL_force{12};130 0;140 0;1000 0];

TCL_force{11} = TractionCharacteristic11;
TCL_force{11}(1,1) = 0;
TCL_force{11} = [TCL_force{11};130 0;140 0;150 0;1000 0];

TCL_force{10} = TractionCharacteristic10;
TCL_force{10}(1,1) = 0;
TCL_force{10}(end,2) = 0;
TCL_force{10} = [TCL_force{10};130 0;140 0;150 0;160 0;1000 0];

TCL_force{9} = TractionCharacteristic9;
TCL_force{9}(1,1) = 0;
TCL_force{9} = [TCL_force{9};130 0;140 0;150 0;160 0;170 0;1000 0];

TCL_force{8} = TractionCharacteristic8;
TCL_force{8}(1,1) = 0;
TCL_force{8} = [TCL_force{8};130 0;140 0;150 0;160 0;170 0;180 0;1000 0];

TCL_force{7} = TractionCharacteristic7;
TCL_force{7}(1,1) = 0;
TCL_force{7} = [TCL_force{7};130 0;140 0;150 0;160 0;170 0;180 0;190 0;1000 0];

TCL_force{6} = TractionCharacteristic6;
TCL_force{6}(1,1) = 0;
TCL_force{6} = [TCL_force{6};130 0;140 0;150 0;160 0;170 0;180 0;190 0;200 0;1000 0];

TCL_force{5} = TractionCharacteristic5;
TCL_force{5}(1,1) = 0;
TCL_force{5} = [TCL_force{5};130 0;140 0;150 0;160 0;170 0;180 0;190 0;200 0;1000 0];

TCL_force{4} = TractionCharacteristic4;
TCL_force{4}(1,1) = 0;
TCL_force{4} = [TCL_force{4};130 0;140 0;150 0;160 0;170 0;180 0;190 0;200 0;1000 0];

TCL_force{3} = TractionCharacteristic3;
TCL_force{3}(1,1) = 0;
TCL_force{3} = [TCL_force{3};130 0;140 0;150 0;160 0;170 0;180 0;190 0;200 0;1000 0];

TCL_force{2} = TractionCharacteristic2;
TCL_force{2}(1,1) = 0;
TCL_force{2}(2,1) = TCL_force{2}(2,1)-0.5;
TCL_force{2}(3,1) = TCL_force{2}(3,1)-0.3;
TCL_force{2} = [TCL_force{2};130 0;140 0;150 0;160 0;170 0;180 0;190 0;200 0;1000 0];

TCL_force{1} = TractionCharacteristic1;
TCL_force{1}(1,1) = 0;
TCL_force{1}(3,1) = 10;
TCL_force{1}(3,2) = 0;
TCL_force{1} = [TCL_force{1};130 0;140 0;150 0;160 0;170 0;180 0;190 0;200 0;1000 0];

figure(1)
plot(TCL_force{1}(:,1),TCL_force{1}(:,2),TCL_force{2}(:,1),TCL_force{2}(:,2),TCL_force{3}(:,1),TCL_force{3}(:,2),...
    TCL_force{4}(:,1),TCL_force{4}(:,2),TCL_force{5}(:,1),TCL_force{5}(:,2),TCL_force{6}(:,1),TCL_force{6}(:,2),...
    TCL_force{7}(:,1),TCL_force{7}(:,2),TCL_force{8}(:,1),TCL_force{8}(:,2),TCL_force{9}(:,1),TCL_force{9}(:,2),...
    TCL_force{10}(:,1),TCL_force{10}(:,2),TCL_force{11}(:,1),TCL_force{11}(:,2),TCL_force{12}(:,1),TCL_force{12}(:,2),'linewidth',2);
set(gca,'FontSize',12,'Fontname','Times New Roman');
xlabel('Velocity(km/h)','FontSize',12,'Fontname','Times New Roman')
ylabel('Motor Traction Force(kN)','FontSize',12,'Fontname','Times New Roman')
%legend('LG1 and WG1','WG50 and LG2')
axis([0 130 0 600]);
grid on
% save('TCL_force','TCL_force');


















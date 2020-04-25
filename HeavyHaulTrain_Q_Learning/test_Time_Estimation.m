
% test_Time_Estimation.m
% Q-learning

% clear;clc

% episode_Loops = 1e3;
% 
% % Nes = [100 50 20 10 5 2 1];
% % Nes = ones(10, 1)*[1 2 5 10 20 25 50 100];
% Nes = ones(10, 1)*[  40 80 ];
% % Nes = ones(1, 1)*[50 50 30];
% episode_loops_ = episode_Loops./Nes;
% 
% T = 60;
% % 
% R_Recorders = zeros(numel(episode_loops_), 1);
% toocs = zeros(numel(episode_loops_), 1);
% for ltr = 1:numel(episode_loops_)
%     
%     episode_loops = episode_loops_(ltr);
%     Ne = Nes(ltr);
%     
%     test_script_MC_method();
% %     test_script_Q_learning();
%     
%     toocs(ltr) = toc;
%     toc
%     R_Recorders(ltr) = mean(R_Recorder(end, :));
% end
% 
% % save TimeErrorEstimation10_2_50.mat  toocs R_Recorders ltr Nes episode_loops_ T
% save TimeErrorEstimation10_2_30_80.mat  toocs R_Recorders ltr Nes episode_loops_ T
% % load TimeErrorEstimation100_2_2e4.mat  toocs R_Recorders ltr Nes episode_loops_ T

% load TimeErrorEstimation10_2_51_80.mat  toocs R_Recorders ltr Nes episode_loops_ T

% toc

% save AllResults20180717_Q.mat

%% 

load TimeErrorEstimation10_2_51_80.mat  toocs R_Recorders ltr Nes episode_loops_ T

R_Recorders_ = reshape(R_Recorders, 10, []);
toocs_ = reshape(toocs, 10, []);
toocs_m = mean(toocs_);
% 
% figure(101);clf
% hold on;
% plot(R_Recorders_', '.');
% plot(toocs_m, 'r')
% grid on;

figure(101);clf
yyaxis left
plot(toocs_m, 'rs-', 'linewidth', 2)
ylim([10, 110])
ylabel('Average Time (s)');

yyaxis right
boxplot(R_Recorders_);
ylim([-100 -40])
ylabel('Rewards');

xticks([1 2:8])
xticklabels({'N = 1','N = 2','N = 5','N = 10','N = 20','N = 25','N = 50','N = 100'})

%% 

% % figure(1);clf
% % mesh(U_recorder')
% % 
% % figure(2);clf
% % plot(V_recorder')
% 
% figure(601);clf
% % semilogy(Rs_record4.*(ones(size(Rs_record4,1), 1)*reward_weigh))
% plot(Rs_record4.*(ones(size(Rs_record4,1), 1)*reward_weigh))
% legend('车钩力','机车牵引力积分', '拖车力积分' ,'速度偏差');
% 
% col = [1 202];
% figure(501);clf
% axes(1) = subplot(411);
% hold on;
% plot(U_recorder(col(1), :)/1e3, 'linewidth', 2);
% plot(U_recorder(col(2), :)/1e3, 'linewidth', 2);
% ylabel('牵引力');
% axes(2) = subplot(412);
% hold on;
% plot(V_recorder(col(1), :)*3.6, 'linewidth', 2);
% plot(V_recorder(col(2), :)*3.6, 'linewidth', 2);
% ylabel('速度 km/h');
% set(gca,'ylim', [20 90]);
% axes(3) = subplot(413);
% hold on;
% plot(loc_Notchs1, 'linewidth', 2);
% plot(loc_Notchs2, 'linewidth', 2);
% ylabel('级位 km/h');
% set(gca,'ylim', [-12 12]);
% axes(4) = subplot(414);
% hold on;
% plot(RP_recorder(col(1), :), 'linewidth', 2);
% plot(RP_recorder(col(2), :), 'linewidth', 2);
% ylabel('坡度');
% linkaxes(axes, 'x');
% 
% set(gca,'xlim', [0 length(tvec)]);
% 
% % % col = 2;
% % figure(102);clf
% % axes(1) = subplot(411);
% % plot(U_recorder(2, :)/1e3, 'linewidth', 2);
% % ylabel('牵引力');
% % axes(2) = subplot(412);
% % plot(V_recorder(1, :)*3.6, 'linewidth', 2);
% % ylabel('速度 km/h');
% % axes(3) = subplot(413);
% % plot(loc_Notchs1, 'linewidth', 2);
% % ylabel('级位 km/h');
% % axes(4) = subplot(414);
% % plot(RP_recorder(1, :), 'linewidth', 2);
% % ylabel('坡度');
% % linkaxes(axes, 'x');
% 
% 
% figure(303);clf
% hold on;
% plot(R_Recorder, 'c' ,'DisplayName','R_Recorder')
% plot(mean(R_Recorder, 2), 'k', 'DisplayName','R_Recorder', 'linewidth', 2)
% % loglog(-mean(R_Recorder, 2),'DisplayName','R_Recorder')
% 
% 
% % figure(303);clf
% % plot(X(1:Nt, :)-Rspeed)
% 
% 
% % figure(303);clf
% % plot(L_recorder')











% plot_final_figure.m

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
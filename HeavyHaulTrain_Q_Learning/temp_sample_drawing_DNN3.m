
A_epis = episode_history(:, :, 1:hm_counter);
len_epi = reshape((max(A_epis(:, 1, :)))-(min(A_epis(:, 1, :))), [hm_counter, 1]);
[~, ind_] = max(len_epi);

Atmp = episode_history(:, :, hm_counter); 

phase_ = Atmp(:, 1:4);
notch_ = Atmp(:, 5:6);
force_ = Atmp(:, 7:8);
energy_ = Atmp(:, 9);

phase_(:, [1 3]) = (phase_(:, [1 3]) + pos_train')/1e3;
force_ = -force_/1e3;
energy_ = energy_/1e3;
xx_ = mean(phase_(:, [1 3]), 2);

figure(501);clf
axes(1) = subplot(411);
hold on;
plot(x_/1e3, y_, 'k', 'linewidth', 3);
% plot([0  TotalLength], lowSpeedLim([1 1])*3.6, 'k', 'linewidth', 3);
set(gca, 'xlim', [0 TotalLength], 'ylim', maxV)
plot(phase_(:, 1), phase_(:, 2)*3.6, 'b-', 'linewidth', 2)
plot(phase_(:, 3), phase_(:, 4)*3.6, 'r-', 'linewidth', 2)
ylabel('Speed (km/h)');
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
yyaxis right
hfill__ = fill(plot_rampList(:, 1)/1e3, plot_rampList(:, 2), 'g');
set(hfill__,'edgealpha', 0.1,'facealpha',0.3) 
set(gca, 'ylim', [min_ramp max_ramp+(max_ramp - min_ramp)*1]);
ylabel('Altitude (m)')
% set(gca, 'ylim', [min(rampList(:, 3)) max(rampList(:, 3))]*3);
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
box on;

axes(2) = subplot(412);
hold on;
stairs(xx_, notch_(:, 1), 'b-', 'linewidth', 2)
stairs(xx_, notch_(:, 2), 'r-', 'linewidth', 2)
set(gca, 'ylim', [-15 15])
hleg = legend('Locmotive #1','Locmotive #2');
set(hleg, 'Location', 'Southeast')
box on;
ylabel('Notch');
set(gca, 'fontsize', 14, 'fontname', 'Cambria')

axes(3) = subplot(413);
hold on;
stairs(xx_, force_(:, 1), 'b-', 'linewidth', 2)
stairs(xx_, force_(:, 2), 'r-', 'linewidth', 2)
hleg = legend('Maxmum','Minimum');
set(hleg, 'Location', 'Southeast')
box on;
ylabel('Coupler Force (KN)');
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
axes(4) = subplot(414); 
hold on;
plot(x_([1 end])/1e3, [0 0], 'k', 'linewidth', 2);
plot(xx_, energy_, '-', 'linewidth', 2) 
box on;
xlabel('Milage (km)');
ylabel('Energy consumption (KJ)');
set(gca, 'fontsize', 14, 'fontname', 'Cambria')
linkaxes(axes, 'x');
set(gca, 'xlim', [1 40])

drawnow;
pause(1e-10);
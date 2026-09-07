clear all; close all; clc;

defaultDir = fullfile(getenv("HOME"), "Downloads", "paper_plot", "3rd Experiment");
if ~isfolder(defaultDir)
    defaultDir = pwd;
end

set(groot,'defaultFigureRenderer','painters');

[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select logging_specific CSV");
if isequal(file,0)
    disp("Canceled.");
    return;
end

csv_path = fullfile(path, file);
fprintf("[INFO] Reading: %s\n", csv_path);

data = readtable(csv_path, 'VariableNamingRule', 'preserve');

time = data{:,1} * 1e-9;
values = data{:,2:end};
N = size(values,1);

position                 = zeros(N,3);
desired_position         = zeros(N,3);
attitude                 = zeros(N,3);
desired_attitude         = zeros(N,3);
torque_dhat              = zeros(N,3);
desired_torque           = zeros(N,4);
past_com                 = zeros(N,3);

position(:,1)=values(:,2);
position(:,2)=values(:,3);
position(:,3)=values(:,4);

desired_position(:,1)=values(:,5);
desired_position(:,2)=values(:,6);
desired_position(:,3)=values(:,7);

attitude(:,1)=values(:,14);
attitude(:,2)=values(:,15);
attitude(:,3)=values(:,16);

desired_attitude(:,1)=values(:,17);
desired_attitude(:,2)=values(:,18);
desired_attitude(:,3)=values(:,19);

desired_torque(:,1)=values(:,29);
desired_torque(:,2)=values(:,30);
desired_torque(:,3)=values(:,31);
desired_torque(:,4)=values(:,32);   % currently unused

torque_dhat(:,1)=values(:,33);
torque_dhat(:,2)=values(:,34);
torque_dhat(:,3)=values(:,35);

past_com(:,1)=values(:,59);
past_com(:,2)=values(:,60);
past_com(:,3)=values(:,61);


%% =========================
% Metrics
% =========================
% User-configurable evaluation windows [s]
metric_window_overall_s    = [0 80];
metric_window_transient1_s = [5 60];
metric_window_transient2_s = [20 60];
metric_window_complete_s   = [60 80];
% Nominal inertia used for torque -> angular acceleration conversion [kg*m^2]
% Source: J_hat = diag(76.8, 87.1, 113) g*m^2 in the hardware table of the paper.
nominal_inertia_diag = 1e-3 * [76.8, 87.1, 113.0];

controller_label = strrep(file, '.csv', '');
controller_label = strrep(controller_label, '.CSV', '');
controller_label = strrep(controller_label, '3rd_', '');

position_error      = position - desired_position;
position_error_norm = sqrt(sum(position_error.^2, 2));

err_roll  = attitude(:,1) - desired_attitude(:,1);
err_pitch = attitude(:,2) - desired_attitude(:,2);
err_yaw   = unwrap(attitude(:,3)) - unwrap(desired_attitude(:,3));

attitude_error_norm    = sqrt(err_roll.^2 + err_pitch.^2 + err_yaw.^2);
angular_accel_cmd      = desired_torque(:,1:3) ./ nominal_inertia_diag;
dob_estimate_accel     = torque_dhat ./ nominal_inertia_diag;
angular_accel_cmd_norm = sqrt(sum(angular_accel_cmd.^2, 2));
dob_estimate_norm      = sqrt(sum(dob_estimate_accel.^2, 2));

mask_overall    = time >= metric_window_overall_s(1)     & time <= metric_window_overall_s(2);
mask_transient1 = time >= metric_window_transient1_s(1)  & time <= metric_window_transient1_s(2);
mask_transient2 = time >= metric_window_transient2_s(1)  & time <= metric_window_transient2_s(2);
mask_complete   = time >= metric_window_complete_s(1)    & time <= metric_window_complete_s(2);

if any(mask_overall)
    overall_rms_position_norm = sqrt(mean(position_error_norm(mask_overall).^2));
    overall_rms_attitude_norm = sqrt(mean(attitude_error_norm(mask_overall).^2));
    overall_peak_attitude_norm = max(attitude_error_norm(mask_overall));
    overall_rms_angacc_cmd_norm = sqrt(mean(angular_accel_cmd_norm(mask_overall).^2));
else
    overall_rms_position_norm = NaN;
    overall_rms_attitude_norm = NaN;
    overall_peak_attitude_norm = NaN;
    overall_rms_angacc_cmd_norm = NaN;
end

if any(mask_transient1)
    transient1_rms_position_norm = sqrt(mean(position_error_norm(mask_transient1).^2));
    transient1_rms_attitude_norm = sqrt(mean(attitude_error_norm(mask_transient1).^2));
    transient1_peak_attitude_norm = max(attitude_error_norm(mask_transient1));
    transient1_integral_attitude_norm = trapz(time(mask_transient1), attitude_error_norm(mask_transient1));
    transient1_rms_angacc_cmd_norm = sqrt(mean(angular_accel_cmd_norm(mask_transient1).^2));
else
    transient1_rms_position_norm = NaN;
    transient1_rms_attitude_norm = NaN;
    transient1_peak_attitude_norm = NaN;
    transient1_integral_attitude_norm = NaN;
    transient1_rms_angacc_cmd_norm = NaN;
end

if any(mask_transient2)
    transient2_rms_position_norm = sqrt(mean(position_error_norm(mask_transient2).^2));
    transient2_rms_attitude_norm = sqrt(mean(attitude_error_norm(mask_transient2).^2));
    transient2_peak_attitude_norm = max(attitude_error_norm(mask_transient2));
    transient2_integral_attitude_norm = trapz(time(mask_transient2), attitude_error_norm(mask_transient2));
    transient2_rms_angacc_cmd_norm = sqrt(mean(angular_accel_cmd_norm(mask_transient2).^2));
else
    transient2_rms_position_norm = NaN;
    transient2_rms_attitude_norm = NaN;
    transient2_peak_attitude_norm = NaN;
    transient2_integral_attitude_norm = NaN;
    transient2_rms_angacc_cmd_norm = NaN;
end

if any(mask_complete)
    complete_rms_x_error = sqrt(mean(position_error(mask_complete,1).^2));
    complete_rms_y_error = sqrt(mean(position_error(mask_complete,2).^2));
    complete_rms_z_error = sqrt(mean(position_error(mask_complete,3).^2));
    complete_rms_position_norm = sqrt(mean(position_error_norm(mask_complete).^2));

    complete_rms_roll_error = sqrt(mean(err_roll(mask_complete).^2));
    complete_rms_pitch_error = sqrt(mean(err_pitch(mask_complete).^2));
    complete_rms_yaw_error = sqrt(mean(err_yaw(mask_complete).^2));
    complete_rms_attitude_norm = sqrt(mean(attitude_error_norm(mask_complete).^2));
    complete_peak_attitude_norm = max(attitude_error_norm(mask_complete));

    complete_rms_angacc_cmd_norm = sqrt(mean(angular_accel_cmd_norm(mask_complete).^2));
    complete_mean_dob_estimate_norm = mean(dob_estimate_norm(mask_complete));
else
    complete_rms_x_error = NaN;
    complete_rms_y_error = NaN;
    complete_rms_z_error = NaN;
    complete_rms_position_norm = NaN;

    complete_rms_roll_error = NaN;
    complete_rms_pitch_error = NaN;
    complete_rms_yaw_error = NaN;
    complete_rms_attitude_norm = NaN;
    complete_peak_attitude_norm = NaN;

    complete_rms_angacc_cmd_norm = NaN;
    complete_mean_dob_estimate_norm = NaN;
end

fprintf("\n============================================================\n");
fprintf("[METRICS] Controller: %s\n", controller_label);
fprintf("[METRICS] CSV: %s\n", file);
fprintf("============================================================\n");

fprintf("Overall evaluation window (t = %.2f-%.2f s)\n", ...
    metric_window_overall_s(1), metric_window_overall_s(2));
fprintf("  %-44s : %10.4f\n", "RMS position-error norm", overall_rms_position_norm);
fprintf("  %-44s : %10.4f\n", "RMS attitude-error norm", overall_rms_attitude_norm);
fprintf("  %-44s : %10.4f\n", "Peak attitude-error norm", overall_peak_attitude_norm);
fprintf("  %-44s : %10.4f\n", "RMS angular-acceleration-command norm", overall_rms_angacc_cmd_norm);

fprintf("\nTransient 1 window (t = %.2f-%.2f s)\n", ...
    metric_window_transient1_s(1), metric_window_transient1_s(2));
fprintf("  %-44s : %10.4f\n", "RMS position-error norm", transient1_rms_position_norm);
fprintf("  %-44s : %10.4f\n", "RMS attitude-error norm", transient1_rms_attitude_norm);
fprintf("  %-44s : %10.4f\n", "Peak attitude-error norm", transient1_peak_attitude_norm);
fprintf("  %-44s : %10.4f\n", "Integral of attitude-error norm", transient1_integral_attitude_norm);
fprintf("  %-44s : %10.4f\n", "RMS angular-acceleration-command norm", transient1_rms_angacc_cmd_norm);

fprintf("\nTransient 2 window (t = %.2f-%.2f s)\n", ...
    metric_window_transient2_s(1), metric_window_transient2_s(2));
fprintf("  %-44s : %10.4f\n", "RMS position-error norm", transient2_rms_position_norm);
fprintf("  %-44s : %10.4f\n", "RMS attitude-error norm", transient2_rms_attitude_norm);
fprintf("  %-44s : %10.4f\n", "Peak attitude-error norm", transient2_peak_attitude_norm);
fprintf("  %-44s : %10.4f\n", "Integral of attitude-error norm", transient2_integral_attitude_norm);
fprintf("  %-44s : %10.4f\n", "RMS angular-acceleration-command norm", transient2_rms_angacc_cmd_norm);

fprintf("\nComplete window (t = %.2f-%.2f s)\n", ...
    metric_window_complete_s(1), metric_window_complete_s(2));
fprintf("  Position tracking\n");
fprintf("    %-42s : %10.4f\n", "Root-mean-square x-position error", complete_rms_x_error);
fprintf("    %-42s : %10.4f\n", "Root-mean-square y-position error", complete_rms_y_error);
fprintf("    %-42s : %10.4f\n", "Root-mean-square z-position error", complete_rms_z_error);
fprintf("    %-42s : %10.4f\n", "Root-mean-square position-error norm", complete_rms_position_norm);

fprintf("  Attitude regulation\n");
fprintf("    %-42s : %10.4f\n", "Root-mean-square roll error", complete_rms_roll_error);
fprintf("    %-42s : %10.4f\n", "Root-mean-square pitch error", complete_rms_pitch_error);
fprintf("    %-42s : %10.4f\n", "Root-mean-square yaw error", complete_rms_yaw_error);
fprintf("    %-42s : %10.4f\n", "Root-mean-square attitude-error norm", complete_rms_attitude_norm);
fprintf("    %-42s : %10.4f\n", "Peak attitude-error norm", complete_peak_attitude_norm);

fprintf("  %-44s : %10.4f\n", "Root-mean-square angular-acceleration-command norm", complete_rms_angacc_cmd_norm);
fprintf("  %-44s : %10.4f\n", "Mean disturbance-observer-estimation magnitude", complete_mean_dob_estimate_norm);
fprintf("============================================================\n\n");


%% =========================
% Style parameters
% =========================
reference_width = 1.0;
measured_width  = 1.2;
norm_width      = 2.5;
rms_width       = 2.0;
dhat_width      = 1.2;
torque_width    = 1.0;
font_size       = 9;
show_y_labels   = false;   % true = 왼쪽 숫자 표시, false = 숨김

default_red    = [0.90 0.25 0.05];
default_blue   = [0.00 0.40 0.85];
light_red      = [0.95 0.55 0.40];
default_purple = [0.4940 0.1840 0.5560];

% =========================
% Color boost helper
% =========================
boostColor = @(c, s) min(max(0.5 + (c-0.5).*(1+s), 0), 1);

color_boost  = 0.20;
default_red  = boostColor(default_red,  color_boost);
default_blue = boostColor(default_blue, color_boost);
light_red    = boostColor(light_red,    color_boost);
default_purple = boostColor(default_purple, 0.15);

grey_est = [0.35 0.35 0.35];
grey_ref = [0.70 0.70 0.70];
light_r  = [1.00 0.45 0.45];
light_b  = [0.45 0.60 1.00];

twin = [0 80];
window_size = 1500;

set(groot,'DefaultAxesFontName','Cambria Math')
set(groot,'DefaultTextFontName','Cambria Math')

% =========================
% Figure
% =========================
f = figure('Name','DataLogging Overview','NumberTitle','off','Color','w', ...
    'Units','normalized','Position',[0 0 1 1]);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;

% --- grid columns: 3 ---
column_width_scale = 0.6;   % 0.75~0.9 정도 추천
colW = ((1-left-right-hgap*2)/3) * column_width_scale;

% --- tile height 기준을 잡는다 (모든 subplot 높이 동일) ---
availH = 1 - top - bottom;

% row 간 간격은 vgap 1번
tileH = (availH - vgap) / (4 + 4);

height_scale = 0.6;   % <-- 추가 (0.6~0.85 추천)
tileH = tileH * height_scale;

rowH  = 4 * tileH;
% row1 bottom y, row2 bottom y 계산
row1_y = 1 - top - rowH;             % 위 row (row=1)
row2_y = bottom;                      % 아래 row (row=2)

% 패널 높이: tiles 개수 * tileH
H3 = 3*tileH;
H4 = 4*tileH;
H45 = 4.5 * tileH;

% column x 좌표
x1 = left;
x2 = left + colW + hgap;
x3 = left + 2*(colW + hgap);
% =========================
% Derived signals
% =========================
dhat_norm = sqrt(sum(torque_dhat.^2,2));
dhat_rms  = sqrt(movmean(dhat_norm.^2, window_size));

err_roll  = attitude(:,1) - desired_attitude(:,1);
err_pitch = attitude(:,2) - desired_attitude(:,2);
err_yaw   = unwrap(attitude(:,3)) - unwrap(desired_attitude(:,3));

att_err_mat  = [err_roll err_pitch err_yaw];
att_err_norm = sqrt(sum(att_err_mat.^2,2));
att_err_rms  = sqrt(movmean(att_err_norm.^2, window_size));

desired_torque_xyz  = desired_torque(:,1:3);
desired_torque_norm = sqrt(sum(desired_torque_xyz.^2,2));
desired_torque_rms  = sqrt(movmean(desired_torque_norm.^2, window_size));

% =========================
% Position (3 tiles) [row1, col1]
% =========================
p11 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x1 row1_y colW H3],'BackgroundColor','w');
tl11 = tiledlayout(p11,3,1,'TileSpacing','compact','Padding','compact');

ax_posx = nexttile(tl11);

plot(time,position(:,1),'-','Color',default_red,'LineWidth',measured_width); hold on
plot(time,desired_position(:,1),'-.','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.5 0.5])

ax_posy = nexttile(tl11);

plot(time,position(:,2),'-','Color',default_red,'LineWidth',measured_width); hold on
plot(time,desired_position(:,2),'-.','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.5 0.5])

ax_posz = nexttile(tl11);
plot(time,position(:,3),'-','Color',default_red,'LineWidth',measured_width); hold on
plot(time,desired_position(:,3),'--','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-1.2 -0.2])
ax_posz.YTick = [-1.2 -0.7 -0.2];
ax_posz.YTickMode = 'manual';

% =========================
% dhat (4 tiles) [row1, col2]
% =========================
p12 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x2 row1_y colW H4],'BackgroundColor','w');
tl12 = tiledlayout(p12,4,1,'TileSpacing','compact','Padding','compact');

ax_dhatx = nexttile(tl12);
plot(time,torque_dhat(:,1),'Color',default_red,'LineWidth',dhat_width)
grid on; xlim(twin); ylim([-3.5 3.5])

ax_dhaty = nexttile(tl12);
plot(time,torque_dhat(:,2),'Color',default_red,'LineWidth',dhat_width)
grid on; xlim(twin); ylim([-3.5 3.5])

ax_dhatz = nexttile(tl12);
plot(time,torque_dhat(:,3),'Color',default_red,'LineWidth',dhat_width)
grid on; xlim(twin); ylim([-2 2])

ax_dhatn = nexttile(tl12);
plot(time,dhat_norm,'Color',light_red,'LineWidth',norm_width); hold on
plot(time,dhat_rms,'-k','LineWidth',rms_width)
grid on; xlim(twin); ylim([0 4])

% =========================
% Disturbance norm panel (single) [row1, col3]
% =========================
p13 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x3 row1_y colW H4], ...
    'BackgroundColor','w');

ax_dn = axes('Parent',p13);
hold(ax_dn,'on')
plot(time,dhat_norm,'Color',light_red,'LineWidth',norm_width);
plot(time,dhat_rms,'-k','LineWidth',rms_width)
grid on
xlim([20 85])
ylim([0 0.5])

% =========================
% Attitude error (custom: last plot = 1.5x height) [row2, col1]
% =========================
p21 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x1 row2_y colW H45],'BackgroundColor','w');

% ---- manual layout inside p21 ----
pad_top = 0.04;
pad_bot = 0.04;
gap = 0.025;

unit_h = (1 - pad_top - pad_bot - 3*gap) / 4.5;   % 1+1+1+1.5 = 4.5
h_small = unit_h;
h_large = 1.5 * unit_h;

y4 = pad_bot;                          % bottom: norm
y3 = y4 + h_large + gap;               % yaw
y2 = y3 + h_small + gap;               % pitch
y1 = y2 + h_small + gap;               % roll

ax_er = axes('Parent',p21,'Units','normalized', ...
    'Position',[0.10 y1 0.86 h_small]);
plot(time,err_roll,'Color',default_red,'LineWidth',measured_width); hold on
plot(time,zeros(size(time)),'--','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.1 0.1])

ax_ep = axes('Parent',p21,'Units','normalized', ...
    'Position',[0.10 y2 0.86 h_small]);
plot(time,err_pitch,'Color',default_red,'LineWidth',measured_width); hold on
plot(time,zeros(size(time)),'--','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.1 0.1])

ax_ey = axes('Parent',p21,'Units','normalized', ...
    'Position',[0.10 y3 0.86 h_small]);
plot(time,err_yaw,'Color',default_red,'LineWidth',measured_width); hold on
plot(time,zeros(size(time)),'--','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.1 0.1])

ax_en = axes('Parent',p21,'Units','normalized', ...
    'Position',[0.10 y4 0.86 h_large]);
plot(time,att_err_norm,'Color',light_red,'LineWidth',norm_width); hold on
plot(time,att_err_rms,'-k','LineWidth',rms_width)
grid on; xlim(twin); ylim([0 0.1])

% =========================
% CoM (single plot) [row2, col2]
% =========================
p22 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x2 row2_y + (rowH-H3) colW H3],'BackgroundColor','w');

ax_com = axes('Parent',p22);
hold(ax_com,'on')

scale = 1000; % m -> mm

plot(time,past_com(:,1)*scale,'-r','LineWidth',measured_width)
plot(time,past_com(:,2)*scale,'-b','LineWidth',measured_width)
plot(time,past_com(:,3)*scale,'-','Color',grey_est,'LineWidth',measured_width)

yline(0.033*scale,'--','Color',light_r,'LineWidth',reference_width);
yline(-0.033*scale,'--','Color',light_b,'LineWidth',reference_width);
yline(-0.06*scale,'--','Color',grey_ref,'LineWidth',reference_width);

grid on
xlim(twin)
ylim([-70 40])

% =========================
% Desired torque panel (4 tiles) [row2, col3]
% =========================
p23 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x3 row2_y colW H4],'BackgroundColor','w');

tl23 = tiledlayout(p23,3,1,'TileSpacing','compact','Padding','compact');

ax_tdx = nexttile(tl23);
plot(time,desired_torque(:,1),'Color',default_red,'LineWidth',torque_width)
grid on; xlim(twin); ylim([-1 5])

ax_tdy = nexttile(tl23);
plot(time,desired_torque(:,2),'Color',default_red,'LineWidth',torque_width)
grid on; xlim(twin); ylim([-1 5])

ax_tdz = nexttile(tl23);
plot(time,desired_torque(:,3),'Color',default_red,'LineWidth',torque_width)
grid on; xlim(twin); ylim([-3 3])


% =========================
% Global style: y-ticks 3개 고정 + 0 포함 (position z 제외)
% =========================
axs = findall(f,'Type','axes');

for k = 1:length(axs)
    ax = axs(k);

    % CoM single plot
    if isequal(ax, ax_com)
        ax.YTick = -60:20:40;
        ax.YTickMode = 'manual';
        continue
    end

    % position z 제외
    if isequal(ax, ax_posz)
        continue
    end

    yl = ylim(ax);

    if yl(1) < 0 && yl(2) > 0
        ax.YTick = [yl(1), 0, yl(2)];
    elseif yl(1) == 0 && yl(2) > 0
        ax.YTick = [0, 0.5*yl(2), yl(2)];
    elseif yl(2) == 0 && yl(1) < 0
        ax.YTick = [yl(1), 0.5*yl(1), 0];
    else
        if yl(2) < 0
            ylim(ax, [yl(1), 0]);
            ax.YTick = [yl(1), 0.5*yl(1), 0];
        elseif yl(1) > 0
            ylim(ax, [0, yl(2)]);
            ax.YTick = [0, 0.5*yl(2), yl(2)];
        else
            ax.YTick = linspace(yl(1), yl(2), 3);
        end
    end

    ax.YTickMode = 'manual';
end

set(axs,'FontSize',font_size,'Color','w')
set(axs,'TickDir','in','Box','on','LineWidth',0.75)
set(axs,'XTick',twin(1):20:twin(2))

if show_y_labels
    set(axs,'YTickLabelMode','auto')
    set(axs,'XTickLabelMode','auto')
else
    set(axs,'YTickLabel',[])
    set(axs,'XTickLabel',[])
end

% =========================
% Save each panel
% =========================
saveDir = fullfile(path, "panels");
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

exportgraphics(p11, fullfile(saveDir,"panel_position.png"),          'Resolution',300);
exportgraphics(p12, fullfile(saveDir,"panel_dhat.png"),              'Resolution',300);
exportgraphics(p21, fullfile(saveDir,"panel_attitude.png"),          'Resolution',300);
exportgraphics(p22, fullfile(saveDir,"panel_com.png"),               'Resolution',300);
exportgraphics(p13, fullfile(saveDir,"panel_disturbance_norm.png"),  'Resolution',300);
exportgraphics(p23, fullfile(saveDir,"panel_desired_torque.png"),    'Resolution',300);

disp("Panels saved.")

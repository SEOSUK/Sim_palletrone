clear all; close all; clc;

defaultDir = fullfile(getenv("HOME"), "palletrone_simulation", "src", "Sim_palletrone", "src", "palletrone_interfaces", "bag");

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

% ---------------------------------------------------------
% Automatic time alignment using MOCE activation event
% data[64] = MOCE enabled flag
% values(:,1) = layout.data_offset
% values(:,2) = data[0]
% therefore data[64] -> values(:,66)
% ---------------------------------------------------------
moce_enabled = values(:,66);

% Append-only simulator extension: data[93] and data[94:97]. Legacy 95-column
% real-flight logs do not contain these fields and retain all original indices.
if size(values,2) >= 99
    eta_common = values(:,95);
    actual_thrust_scale = values(:,96:99);
    fprintf('[INFO] Common thrust effectiveness range: %.6f to %.6f\n', ...
        min(eta_common), max(eta_common));
else
    eta_common = NaN(N,1);
    actual_thrust_scale = NaN(N,4);
end
if size(values,2) >= 117
    ou_force_body = values(:,112:114);
    total_external_force_body = values(:,115:117);
else
    ou_force_body = NaN(N,3);
    total_external_force_body = NaN(N,3);
end
if size(values,2) >= 125
    physical_motor_thrust = values(:,118:121);
    measured_servo_v5 = values(:,122:125);
else
    physical_motor_thrust = NaN(N,4);
    measured_servo_v5 = NaN(N,4);
end
if size(values,2) >= 111
    ou_torque_body = values(:,106:108);
    total_external_torque_body = values(:,109:111);
else
    ou_torque_body = NaN(N,3);
    total_external_torque_body = NaN(N,3);
end

% First rising edge of MOCE enable flag
idx_moce = find(diff(moce_enabled > 0.5) == 1, 1, 'first') + 1;

if isempty(idx_moce)
    error('MOCE activation event was not found in the CSV.');
end

moce_start_time = time(idx_moce);

% In the real-flight evaluation:
%   t = 5 s corresponds to MOCE activation.
% Therefore set simulation t = 0 to 5 s before MOCE activation.
metric_start_time = moce_start_time - 5.0;

fprintf('[INFO] MOCE activated at log t = %.4f s\n', moce_start_time);
fprintf('[INFO] Metric t = 0 aligned to log t = %.4f s\n', metric_start_time);

% Evaluation windows after alignment
metric_window_overall_s    = [0 80];
metric_window_transient1_s = [5 60];
metric_window_transient2_s = [20 60];
metric_window_complete_s   = [60 80];

analysis_mask = time >= metric_start_time;

if ~any(analysis_mask)
    error('metric_start_time (%.3f s) is outside the logged data range.', ...
        metric_start_time);
end

time             = time(analysis_mask) - metric_start_time;
position         = position(analysis_mask,:);
desired_position = desired_position(analysis_mask,:);
attitude         = attitude(analysis_mask,:);
desired_attitude = desired_attitude(analysis_mask,:);
torque_dhat      = torque_dhat(analysis_mask,:);
desired_torque   = desired_torque(analysis_mask,:);
past_com         = past_com(analysis_mask,:);
eta_common       = eta_common(analysis_mask,:);
actual_thrust_scale = actual_thrust_scale(analysis_mask,:);
ou_torque_body = ou_torque_body(analysis_mask,:);
total_external_torque_body = total_external_torque_body(analysis_mask,:);

fprintf('[INFO] Analysis starts at log t = %.3f s; shifted plot time starts at 0 s.\n', ...
    metric_start_time);

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

set(groot,'DefaultAxesFontName','Times New Roman')
set(groot,'DefaultTextFontName','Times New Roman')

% =========================
% Figure
% =========================
f = figure('Name','DataLogging Overview','NumberTitle','off','Color','w', ...
    'Units','normalized','Position',[0 0 1 1]);
overview = tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
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
tl11 = tiledlayout(overview,3,1,'TileSpacing','compact','Padding','compact');
tl11.Layout.Tile = 1;

ax_posx = nexttile(tl11);

h_pos_measured = plot(time,position(:,1),'-','Color',default_red,'LineWidth',measured_width); hold on
h_pos_desired = plot(time,desired_position(:,1),'-.','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.5 0.5])
title(tl11,'Position Tracking')
legend(ax_posx,[h_pos_measured h_pos_desired],{'Measured','Desired'}, ...
    'Location','best','Box','off')

ax_posy = nexttile(tl11);

plot(time,position(:,2),'-','Color',default_red,'LineWidth',measured_width); hold on
plot(time,desired_position(:,2),'-.','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.5 0.5])

ax_posz = nexttile(tl11);
plot(time,position(:,3),'-','Color',default_red,'LineWidth',measured_width); hold on
plot(time,desired_position(:,3),'--','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([0.2 1.2])
ax_posz.YTick = [0.2 0.7 1.2];
ax_posz.YTickMode = 'manual';

% =========================
% dhat (4 tiles) [row1, col2]
% =========================
tl12 = tiledlayout(overview,4,1,'TileSpacing','compact','Padding','compact');
tl12.Layout.Tile = 2;

ax_dhatx = nexttile(tl12);
h_dhat = plot(time,torque_dhat(:,1),'Color',default_red,'LineWidth',dhat_width);
grid on; xlim(twin); ylim([-3.5 3.5])
title(tl12,'DOB Disturbance-Torque Estimate')
legend(ax_dhatx,h_dhat,{'Estimated disturbance'},'Location','best','Box','off')

ax_dhaty = nexttile(tl12);
plot(time,torque_dhat(:,2),'Color',default_red,'LineWidth',dhat_width)
grid on; xlim(twin); ylim([-3.5 3.5])

ax_dhatz = nexttile(tl12);
plot(time,torque_dhat(:,3),'Color',default_red,'LineWidth',dhat_width)
grid on; xlim(twin); ylim([-2 2])

ax_dhatn = nexttile(tl12);
h_dhat_norm = plot(time,dhat_norm,'Color',light_red,'LineWidth',norm_width); hold on
h_dhat_rms = plot(time,dhat_rms,'-k','LineWidth',rms_width);
grid on; xlim(twin); ylim([0 4])
legend(ax_dhatn,[h_dhat_norm h_dhat_rms],{'Norm','Moving RMS'}, ...
    'Location','best','Box','off')

% =========================
% Disturbance norm panel (single) [row1, col3]
% =========================
tl13 = tiledlayout(overview,1,1,'TileSpacing','compact','Padding','compact');
tl13.Layout.Tile = 3;

ax_dn = nexttile(tl13);
hold(ax_dn,'on')
h_dn_norm = plot(time,dhat_norm,'Color',light_red,'LineWidth',norm_width);
h_dn_rms = plot(time,dhat_rms,'-k','LineWidth',rms_width);
grid on
xlim(twin)
ylim([0 0.5])
title(ax_dn,'Disturbance Estimate Norm')
legend(ax_dn,[h_dn_norm h_dn_rms],{'Norm','Moving RMS'}, ...
    'Location','best','Box','off')

% =========================
% Attitude error (3 tiles) [row2, col1]
% =========================
tl21 = tiledlayout(overview,3,1,'TileSpacing','compact','Padding','compact');
tl21.Layout.Tile = 4;

ax_er = nexttile(tl21);
h_att_error = plot(time,err_roll,'Color',default_red,'LineWidth',measured_width); hold on
h_att_zero = plot(time,zeros(size(time)),'--','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.1 0.1])
title(tl21,'Attitude Tracking Error')
legend(ax_er,[h_att_error h_att_zero],{'Error','Zero reference'}, ...
    'Location','best','Box','off')

ax_ep = nexttile(tl21);
plot(time,err_pitch,'Color',default_red,'LineWidth',measured_width); hold on
plot(time,zeros(size(time)),'--','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.1 0.1])

ax_ey = nexttile(tl21);
plot(time,err_yaw,'Color',default_red,'LineWidth',measured_width); hold on
plot(time,zeros(size(time)),'--','Color',default_blue,'LineWidth',reference_width);
grid on; xlim(twin); ylim([-0.1 0.1])

% =========================
% CoM (single plot) [row2, col2]
% =========================
tl22 = tiledlayout(overview,1,1,'TileSpacing','compact','Padding','compact');
tl22.Layout.Tile = 5;

ax_com = nexttile(tl22);
hold(ax_com,'on')

scale = 1000; % m -> mm

h_com_x = plot(time,past_com(:,1)*scale,'-r','LineWidth',measured_width);
h_com_y = plot(time,past_com(:,2)*scale,'-b','LineWidth',measured_width);
h_com_z = plot(time,past_com(:,3)*scale,'-','Color',grey_est,'LineWidth',measured_width);

h_com_ref_x = yline(0.033*scale,'--','Color',light_r,'LineWidth',reference_width);
h_com_ref_y = yline(-0.033*scale,'--','Color',light_b,'LineWidth',reference_width);
h_com_ref_z = yline(-0.06*scale,'--','Color',grey_ref,'LineWidth',reference_width);

grid on
xlim(twin)
ylim([-70 40])
title(ax_com,'Center-of-Mass Estimate')
legend(ax_com,[h_com_x h_com_y h_com_z h_com_ref_x h_com_ref_y h_com_ref_z], ...
    {'Estimated x','Estimated y','Estimated z','Reference x','Reference y','Reference z'}, ...
    'Location','best','NumColumns',2,'Box','off')

% =========================
% Desired torque panel (4 tiles) [row2, col3]
% =========================
tl23 = tiledlayout(overview,3,1,'TileSpacing','compact','Padding','compact');
tl23.Layout.Tile = 6;

ax_tdx = nexttile(tl23);
h_torque = plot(time,desired_torque(:,1),'Color',default_red,'LineWidth',torque_width);
grid on; xlim(twin); ylim([-1 5])
title(tl23,'Desired Body Torque')
legend(ax_tdx,h_torque,{'Desired torque'},'Location','best','Box','off')

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
set(axs,'YTickLabelMode','auto')
set(axs,'XTickLabel',[])

% Show x-axis ticks and titles only on the bottom axis of each panel.
bottom_axes = [ax_posz ax_dhatn ax_dn ax_ey ax_com ax_tdz];
set(bottom_axes,'XTickLabelMode','auto')

default_font_size = get(groot,'DefaultAxesFontSize');
for k = 1:numel(bottom_axes)
    x_label = xlabel(bottom_axes(k),'Time [s]');
    set(x_label,'FontName','Times New Roman','FontSize',default_font_size)
end

% Keep panel titles at MATLAB's default text size in Times New Roman.
panel_titles = [tl11.Title tl12.Title ax_dn.Title tl21.Title ax_com.Title tl23.Title];
set(panel_titles,'FontName','Times New Roman','FontSize',default_font_size)

% =========================
% Save each panel
% =========================
saveDir = fullfile(path, "panels");
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

exportgraphics(tl11, fullfile(saveDir,"panel_position.png"),          'Resolution',300);
exportgraphics(tl12, fullfile(saveDir,"panel_dhat.png"),              'Resolution',300);
exportgraphics(tl21, fullfile(saveDir,"panel_attitude.png"),          'Resolution',300);
exportgraphics(tl22, fullfile(saveDir,"panel_com.png"),               'Resolution',300);
exportgraphics(tl13, fullfile(saveDir,"panel_disturbance_norm.png"),  'Resolution',300);
exportgraphics(tl23, fullfile(saveDir,"panel_desired_torque.png"),    'Resolution',300);

disp("Panels saved.")

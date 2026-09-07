clear all; close all; clc;

defaultDir = fullfile(getenv("HOME"), "Downloads", "paper_plot", "1st Experiment");
if ~isfolder(defaultDir)
    defaultDir = pwd;
end

set(groot, 'defaultFigureRenderer', 'painters');
set(groot, 'DefaultAxesFontName', 'Cambria Math');
set(groot, 'DefaultTextFontName', 'Cambria Math');

[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select logging_specific CSV");
if isequal(file, 0)
    disp("Canceled.");
    return;
end

csv_path = fullfile(path, file);
fprintf("[INFO] Reading: %s\n", csv_path);

data = readtable(csv_path, 'VariableNamingRule', 'preserve');

time = data{:,1} * 1e-9;
values = data{:,2:end};
N = size(values, 1);

position         = zeros(N,3);
desired_position = zeros(N,3);
attitude         = zeros(N,3);
desired_attitude = zeros(N,3);
desired_force_B  = zeros(N,3);
torque_dhat      = zeros(N,3);
past_com         = zeros(N,3);

position(:,1)         = values(:,2);
position(:,2)         = values(:,3);
position(:,3)         = values(:,4);

desired_position(:,1) = values(:,5);
desired_position(:,2) = values(:,6);
desired_position(:,3) = values(:,7);

attitude(:,1)         = values(:,14);
attitude(:,2)         = values(:,15);
attitude(:,3)         = values(:,16);

desired_attitude(:,1) = values(:,17);
desired_attitude(:,2) = values(:,18);
desired_attitude(:,3) = values(:,19);

% Logger mapping:
% values(:,26:28) -> raw_data entries fx, fy, fz from VehicleThrustSetpoint::xyz
% This is used by the CoM estimator as body_force_desired.
desired_force_B(:,1)  = values(:,26);
desired_force_B(:,2)  = values(:,27);
desired_force_B(:,3)  = values(:,28);

torque_dhat(:,1)      = values(:,33);
torque_dhat(:,2)      = values(:,34);
torque_dhat(:,3)      = values(:,35);

past_com(:,1)         = values(:,59);
past_com(:,2)         = values(:,60);
past_com(:,3)         = values(:,61);

time_rel = time - time(1);

%% Offline nominal-Gramian check
Jhat = diag([0.0768, 0.0871, 0.113]);  % [kg m^2]
omegaQ = 2.0;                          % [rad/s]
Tg_list = [5.0, 10.0, 15.0];          % [s]
t_translation = 28.0;                  % [s]
rho_list = cell(numel(Tg_list), 1);
stats_list = cell(numel(Tg_list), 1);

fprintf("\n[INFO] Offline nominal-Gramian consistency check\n");
fprintf("[INFO] Force source: values(:,26:28) = body-frame commanded force from VehicleThrustSetpoint::xyz\n");
for iT = 1:numel(Tg_list)
    [rho_i, stats_i] = local_compute_nominal_gramian( ...
        time_rel, desired_force_B, Jhat, omegaQ, Tg_list(iT), t_translation);
    rho_list{iT} = rho_i;
    stats_list{iT} = stats_i;
    fprintf("[INFO] Tg = %.1f s\n", Tg_list(iT));
    fprintf("       hover: median = %.6g, max = %.6g\n", ...
        stats_i.hover_median, stats_i.hover_max);
    fprintf("       translation: median = %.6g, min = %.6g, max = %.6g\n", ...
        stats_i.trans_median, stats_i.trans_min, stats_i.trans_max);
end

%% Style parameters
reference_width = 2.0;
measured_width  = 1.8;
norm_width      = 3.5;
rms_width       = 2.2;
font_size       = 9;
column_width_scale = 1.05;

default_red     = [0.90 0.25 0.05];
default_blue    = [0.00 0.40 0.85];
light_red       = [0.95 0.55 0.40];
default_green   = [0.20 0.60 0.20];
grey_est        = [0.35 0.35 0.35];
grey_ref        = [0.70 0.70 0.70];
light_r         = [1.00 0.45 0.45];
light_b         = [0.45 0.60 1.00];

boostColor = @(c, s) min(max(0.5 + (c - 0.5) .* (1 + s), 0), 1);
color_boost = 0.20;

default_red   = boostColor(default_red, color_boost);
default_blue  = boostColor(default_blue, color_boost);
light_red     = boostColor(light_red, color_boost);
default_green = boostColor(default_green, 0.10);

twin = [0 100];
window_size = 1500;

dhat_norm = sqrt(sum(torque_dhat.^2, 2));
dhat_rms  = sqrt(movmean(dhat_norm.^2, window_size));

%% Figure layout
f = figure('Name', 'DataLogging Overview + Nominal Gramian', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0 0 1 1]);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;

colW = (1 - left - right - hgap * 2) / 3 * column_width_scale;
availH = 1 - top - bottom;
tileH = (availH - vgap) / (4 + 4);
height_scale = 0.6;
tileH = tileH * height_scale;

rowH = 4 * tileH;
row1_y = 1 - top - rowH;
row2_y = bottom;

H3 = 3 * tileH;
H4 = 4 * tileH;

x1 = left;
x2 = left + colW + hgap;
x3 = left + 2 * (colW + hgap);

% Position panel
p11 = uipanel('Parent', f, 'Units', 'normalized', ...
    'Position', [x1 row1_y + (rowH - H3) colW H3], 'BackgroundColor', 'w');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_posx = nexttile(tl11);
plot(ax_posx, time_rel, position(:,1), '-', 'Color', default_red, 'LineWidth', measured_width); hold(ax_posx, 'on');
plot(ax_posx, time_rel, desired_position(:,1), '-.', 'Color', default_blue, 'LineWidth', reference_width);
grid(ax_posx, 'on'); xlim(ax_posx, twin); ylim(ax_posx, [-0.5 0.5]);
ylabel(ax_posx, '$x$ [m]', 'Interpreter', 'latex');
title(ax_posx, 'Position');
legend(ax_posx, {'measured', 'desired'}, 'Location', 'eastoutside');

ax_posy = nexttile(tl11);
plot(ax_posy, time_rel, position(:,2) + 0.112226, '-', 'Color', default_red, 'LineWidth', measured_width); hold(ax_posy, 'on');
plot(ax_posy, time_rel, desired_position(:,2) + 0.112226, '-.', 'Color', default_blue, 'LineWidth', reference_width);
grid(ax_posy, 'on'); xlim(ax_posy, twin); ylim(ax_posy, [-0.5 0.5]);
ylabel(ax_posy, '$y$ [m]', 'Interpreter', 'latex');

ax_posz = nexttile(tl11);
plot(ax_posz, time_rel, position(:,3) - 0.051513, '-', 'Color', default_red, 'LineWidth', measured_width); hold(ax_posz, 'on');
plot(ax_posz, time_rel, desired_position(:,3) - 0.051513, '--', 'Color', default_blue, 'LineWidth', reference_width);
grid(ax_posz, 'on'); xlim(ax_posz, twin); ylim(ax_posz, [-1.3 -0.3]);
ax_posz.YTick = [-1.3 -0.8 -0.3];
ylabel(ax_posz, '$z$ [m]', 'Interpreter', 'latex');
xlabel(ax_posz, 'Time [s]');

% Disturbance observer panel
dhat_norm_scale = 3.0;
base_h = tileH;
H_dhat = (3 + dhat_norm_scale) * base_h;
p12_top = row1_y + H4;
p12_y = p12_top - H_dhat;

p12 = uipanel('Parent', f, 'Units', 'normalized', ...
    'Position', [x2 p12_y colW H_dhat], 'BackgroundColor', 'w');

inner_left = 0.10; inner_right = 0.05; inner_top = 0.03; inner_bottom = 0.06; inner_gap = 0.03;
axW = 1 - inner_left - inner_right;
availH_inner = 1 - inner_top - inner_bottom - 3 * inner_gap;
unitH = availH_inner / (3 + dhat_norm_scale);

h1 = unitH; h2 = unitH; h3 = unitH; h4 = dhat_norm_scale * unitH;
y4 = inner_bottom;
y3 = y4 + h4 + inner_gap;
y2 = y3 + h3 + inner_gap;
y1 = y2 + h2 + inner_gap;

ax_dhatx = axes('Parent', p12, 'Units', 'normalized', 'Position', [inner_left y1 axW h1]);
plot(ax_dhatx, time_rel, torque_dhat(:,1), 'Color', default_red, 'LineWidth', reference_width);
grid(ax_dhatx, 'on'); xlim(ax_dhatx, twin); ylim(ax_dhatx, [-2 2]);
ylabel(ax_dhatx, '$\hat{\tau}_x$', 'Interpreter', 'latex');
title(ax_dhatx, 'DOB Torque Estimate');

ax_dhaty = axes('Parent', p12, 'Units', 'normalized', 'Position', [inner_left y2 axW h2]);
plot(ax_dhaty, time_rel, torque_dhat(:,2), 'Color', default_red, 'LineWidth', reference_width);
grid(ax_dhaty, 'on'); xlim(ax_dhaty, twin); ylim(ax_dhaty, [-2 2]);
ylabel(ax_dhaty, '$\hat{\tau}_y$', 'Interpreter', 'latex');

ax_dhatz = axes('Parent', p12, 'Units', 'normalized', 'Position', [inner_left y3 axW h3]);
plot(ax_dhatz, time_rel, torque_dhat(:,3), 'Color', default_red, 'LineWidth', reference_width);
grid(ax_dhatz, 'on'); xlim(ax_dhatz, twin); ylim(ax_dhatz, [-2 2]);
ylabel(ax_dhatz, '$\hat{\tau}_z$', 'Interpreter', 'latex');

ax_dhatn = axes('Parent', p12, 'Units', 'normalized', 'Position', [inner_left y4 axW h4]);
plot(ax_dhatn, time_rel, dhat_norm, 'Color', light_red, 'LineWidth', norm_width); hold(ax_dhatn, 'on');
plot(ax_dhatn, time_rel, dhat_rms, '-k', 'LineWidth', rms_width);
grid(ax_dhatn, 'on'); xlim(ax_dhatn, twin); ylim(ax_dhatn, [0 4.0]);
ylabel(ax_dhatn, '$\|\hat{\tau}\|$', 'Interpreter', 'latex');
xlabel(ax_dhatn, 'Time [s]');
legend(ax_dhatn, {'norm', 'moving RMS'}, 'Location', 'eastoutside');

% Disturbance norm panel
panel_scale = 0.6;
p13 = uipanel('Parent', f, 'Units', 'normalized', ...
    'Position', [x3 row1_y + (rowH - rowH * panel_scale) colW rowH * panel_scale], ...
    'BackgroundColor', 'w');

ax_dn = axes('Parent', p13);
hold(ax_dn, 'on');
plot(ax_dn, time_rel, dhat_norm, 'Color', light_red, 'LineWidth', 4.0);
plot(ax_dn, time_rel, dhat_rms, '-k', 'LineWidth', rms_width);
grid(ax_dn, 'on'); xlim(ax_dn, [20 100]); ylim(ax_dn, [0 0.5]);
title(ax_dn, 'Disturbance Norm Zoom');
xlabel(ax_dn, 'Time [s]');
ylabel(ax_dn, '$\|\hat{\tau}\|$', 'Interpreter', 'latex');
legend(ax_dn, {'norm', 'moving RMS'}, 'Location', 'eastoutside');

% Attitude error panel
p21 = uipanel('Parent', f, 'Units', 'normalized', ...
    'Position', [x1 row2_y colW H4], 'BackgroundColor', 'w');
tl21 = tiledlayout(p21, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

err_roll  = attitude(:,1) - desired_attitude(:,1);
err_pitch = attitude(:,2) - desired_attitude(:,2);
err_yaw   = unwrap(attitude(:,3)) - unwrap(desired_attitude(:,3));

ax_er = nexttile(tl21);
plot(ax_er, time_rel, err_roll, 'Color', default_red, 'LineWidth', measured_width); hold(ax_er, 'on');
plot(ax_er, time_rel, zeros(size(time_rel)), '--', 'Color', default_blue, 'LineWidth', reference_width);
grid(ax_er, 'on'); xlim(ax_er, twin); ylim(ax_er, [-0.1 0.1]);
ylabel(ax_er, '$e_\phi$', 'Interpreter', 'latex');
title(ax_er, 'Attitude Error');
legend(ax_er, {'error', 'zero'}, 'Location', 'eastoutside');

ax_ep = nexttile(tl21);
plot(ax_ep, time_rel, err_pitch, 'Color', default_red, 'LineWidth', measured_width); hold(ax_ep, 'on');
plot(ax_ep, time_rel, zeros(size(time_rel)), '--', 'Color', default_blue, 'LineWidth', reference_width);
grid(ax_ep, 'on'); xlim(ax_ep, twin); ylim(ax_ep, [-0.1 0.1]);
ylabel(ax_ep, '$e_\theta$', 'Interpreter', 'latex');

ax_ey = nexttile(tl21);
plot(ax_ey, time_rel, err_yaw, 'Color', default_red, 'LineWidth', measured_width); hold(ax_ey, 'on');
plot(ax_ey, time_rel, zeros(size(time_rel)), '--', 'Color', default_blue, 'LineWidth', reference_width);
grid(ax_ey, 'on'); xlim(ax_ey, twin); ylim(ax_ey, [-0.1 0.1]);
ylabel(ax_ey, '$e_\psi$', 'Interpreter', 'latex');

ax_en = nexttile(tl21);
att_err_mat  = [err_roll err_pitch err_yaw];
att_err_norm = sqrt(sum(att_err_mat.^2, 2));
att_err_rms  = sqrt(movmean(att_err_norm.^2, window_size));
plot(ax_en, time_rel, att_err_norm, 'Color', light_red, 'LineWidth', norm_width); hold(ax_en, 'on');
plot(ax_en, time_rel, att_err_rms, '-k', 'LineWidth', rms_width);
grid(ax_en, 'on'); xlim(ax_en, twin); ylim(ax_en, [0 0.1]);
ylabel(ax_en, '$\|e_R\|$', 'Interpreter', 'latex');
xlabel(ax_en, 'Time [s]');
legend(ax_en, {'norm', 'moving RMS'}, 'Location', 'eastoutside');

% CoM panel
p22 = uipanel('Parent', f, 'Units', 'normalized', ...
    'Position', [x2 row2_y + (rowH - H3) colW H3], 'BackgroundColor', 'w');

ax_com = axes('Parent', p22);
hold(ax_com, 'on');

scale = 1000;
plot(ax_com, time_rel, past_com(:,1) * scale, '-r', 'LineWidth', measured_width);
plot(ax_com, time_rel, past_com(:,2) * scale, '-b', 'LineWidth', measured_width);
plot(ax_com, time_rel, past_com(:,3) * scale, '-', 'Color', grey_est, 'LineWidth', measured_width);
yline(ax_com, 0.00095 * scale, '--', 'Color', light_r, 'LineWidth', reference_width);
yline(ax_com, 0.0220 * scale, '--', 'Color', light_b, 'LineWidth', reference_width);
yline(ax_com, -0.0458 * scale, '--', 'Color', grey_ref, 'LineWidth', reference_width);
grid(ax_com, 'on'); xlim(ax_com, twin); ylim(ax_com, [-60 40]);
yticks(ax_com, [-60 -40 -20 0 20 40]);
title(ax_com, 'CoM Estimate');
xlabel(ax_com, 'Time [s]');
ylabel(ax_com, 'CoM [mm]');
legend(ax_com, {'$\hat{c}_x$', '$\hat{c}_y$', '$\hat{c}_z$', '$c_x^\star$', '$c_y^\star$', '$c_z^\star$'}, ...
    'Interpreter', 'latex', 'Location', 'eastoutside');

% Gramian panel
p23 = uipanel('Parent', f, 'Units', 'normalized', ...
    'Position', [x3 row2_y colW H4], 'BackgroundColor', 'w');

gramian_pos = [
    0.12 0.71 0.78 0.17
    0.12 0.43 0.78 0.17
    0.12 0.15 0.78 0.17
];

for iT = 1:numel(Tg_list)
    Tg_i = Tg_list(iT);
    rho_i = rho_list{iT};
    t_window_full_i = t_translation + Tg_i;

    ax_g = axes('Parent', p23, 'Units', 'normalized', 'Position', gramian_pos(iT,:));
    plot(ax_g, time_rel, rho_i, '-', 'LineWidth', 2.0, 'Color', [0.4940 0.1840 0.5560]); hold(ax_g, 'on');
    xline(ax_g, t_translation, '--k', 'LineWidth', 1.0);
 %   xline(ax_g, t_window_full_i, ':k', 'LineWidth', 1.0);
    grid(ax_g, 'on');
    xlim(ax_g, twin);

    if iT == numel(Tg_list)
    else
        set(ax_g, 'XTickLabel', []);
    end
end

% Global style
axs = findall(f, 'Type', 'axes');
set(axs, 'FontSize', font_size, 'Color', 'w');
set(axs, 'TickDir', 'in', 'Box', 'on', 'LineWidth', 1.0);
set(axs, 'XTick', twin(1):20:twin(2));

for ax = axs.'
    ax.LooseInset = ax.TightInset;
end

% Save panel images
saveDir = fullfile(path, "panels");
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

exportgraphics(p11, fullfile(saveDir, "panel_position.png"), 'Resolution', 300);
exportgraphics(p12, fullfile(saveDir, "panel_dhat.png"), 'Resolution', 300);
exportgraphics(p13, fullfile(saveDir, "panel_disturbance_norm.png"), 'Resolution', 300);
exportgraphics(p21, fullfile(saveDir, "panel_attitude.png"), 'Resolution', 300);
exportgraphics(p22, fullfile(saveDir, "panel_com.png"), 'Resolution', 300);
exportgraphics(p23, fullfile(saveDir, "panel_gramian.png"), 'Resolution', 300);

fprintf("[INFO] Saved panels to: %s\n", saveDir);

function [rho_g, stats, Ahat_tilde] = local_compute_nominal_gramian(time_rel, force_B, Jhat, omegaQ, Tg, t_translation)
    N = numel(time_rel);

    % Toolbox-free causal Q-filter implementation matching the estimator's
    % continuous-time realization integrated sample by sample.
    force_B_tilde = local_apply_q_filter(time_rel, force_B, omegaQ);

    Ahat_tilde = zeros(N, 9);
    for k = 1:N
        At = Jhat \ local_skew(force_B_tilde(k,:).');
        Ahat_tilde(k,:) = At(:).';
    end

    Sflat = zeros(N, 9);
    for k = 1:N
        At = reshape(Ahat_tilde(k,:), 3, 3);
        Sk = At.' * At;
        Sflat(k,:) = Sk(:).';
    end

    cumS = cumtrapz(time_rel, Sflat);
    rho_g = nan(N,1);

    for k = 1:N
        t0 = max(time_rel(1), time_rel(k) - Tg);
        cumS_t0 = interp1(time_rel, cumS, t0, 'linear');
        What = reshape(cumS(k,:) - cumS_t0, 3, 3);
        What = 0.5 * (What + What.');
        ev = eig(What);
        rho_g(k) = max(min(real(ev)), 0);
    end

    idx_hover = time_rel >= max(Tg, 10.0) & time_rel < t_translation;
    idx_trans = time_rel >= (t_translation + Tg) & time_rel <= time_rel(end);

    stats.hover_median = local_safe_stat(rho_g, idx_hover, @median);
    stats.hover_max    = local_safe_stat(rho_g, idx_hover, @max);
    stats.trans_median = local_safe_stat(rho_g, idx_trans, @median);
    stats.trans_min    = local_safe_stat(rho_g, idx_trans, @min);
    stats.trans_max    = local_safe_stat(rho_g, idx_trans, @max);
end

function y = local_apply_q_filter(time_rel, u, omegaQ)
    root2 = sqrt(2.0);
    fc2 = omegaQ^2;

    A = [-root2 * omegaQ, -fc2;
          1.0,            0.0];
    B = [1.0; 0.0];
    C = [0.0, fc2];

    N = size(u, 1);
    ncol = size(u, 2);
    y = zeros(N, ncol);

    for j = 1:ncol
        x = zeros(2,1);
        y(1,j) = C * x;
        for k = 2:N
            dt = time_rel(k) - time_rel(k - 1);
            if ~isfinite(dt) || dt <= 0
                dt = 0;
            end

            x_dot = A * x + B * u(k - 1, j);
            x = x + x_dot * dt;
            y(k,j) = C * x;
        end
    end
end

function S = local_skew(v)
    S = [   0   -v(3)  v(2);
          v(3)   0    -v(1);
         -v(2)  v(1)   0  ];
end

function out = local_safe_stat(x, mask, funHandle)
    x_use = x(mask & isfinite(x));
    if isempty(x_use)
        out = NaN;
        return;
    end

    switch func2str(funHandle)
        case 'median'
            out = median(x_use);
        case 'max'
            out = max(x_use);
        case 'min'
            out = min(x_use);
        otherwise
            out = funHandle(x_use);
    end
end

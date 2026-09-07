clear all; close all; clc;

defaultDir = fullfile(getenv("HOME"), "Downloads", "paper_plot", "1st Experiment");
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

torque_dhat(:,1)=values(:,33);
torque_dhat(:,2)=values(:,34);
torque_dhat(:,3)=values(:,35);

past_com(:,1)=values(:,59);
past_com(:,2)=values(:,60);
past_com(:,3)=values(:,61);

%% ========================
% Style parameters
% =========================
% com은 4.0, 2.2
% pos att는 2.0 1.8
reference_width = 2.0;  
measured_width  = 1.8;
norm_width      = 4.0;
rms_width       = 2.5;
font_size       = 9;
show_y_labels = false;   % true = 왼쪽 숫자 표시, false = 숨김
column_width_scale = 1.05;   % <---- 너비 조절 (0.9 ~ 1.2 추천)

default_red  = [0.90 0.25 0.05];
default_blue = [0.00 0.40 0.85];
light_red    = [0.95   0.55   0.40];
default_purple = [0.4940 0.1840 0.5560];

% =========================
% Color boost helper (sat/contrast up a bit)
% =========================
boostColor = @(c, s) min(max(0.5 + (c-0.5).*(1+s), 0), 1); % s=0.15~0.30 추천

color_boost = 0.20;   % <-- 여기만 조절 (0.15: 약간 / 0.25: 꽤 쨍)

default_red  = boostColor(default_red,  color_boost);
default_blue = boostColor(default_blue, color_boost);
light_red    = boostColor(light_red,    color_boost);

% 필요하면 보라색도 같이:
default_purple = boostColor(default_purple, 0.15);


twin = [0 100];

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
colW = (1-left-right-hgap*2)/3 * column_width_scale;

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

% column x 좌표
x1 = left;
x2 = left + colW + hgap;
x3 = left + 2*(colW + hgap);

% =========================
% Position (3 tiles)  [row1, col1]
% =========================
p11 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x1 row1_y + (rowH-H3) colW H3],'BackgroundColor','w');
tl11 = tiledlayout(p11,3,1,'TileSpacing','compact','Padding','compact');

ax_posx = nexttile(tl11);
plot(time,position(:,1),'-','Color',default_red,'LineWidth',measured_width); hold on
plot(time,desired_position(:,1),'-.','Color',default_blue,'LineWidth',reference_width); hold on
grid on; xlim(twin); ylim([-0.5 0.5])

ax_posy = nexttile(tl11);
plot(time,position(:,2) + 0.112226,'-','Color',default_red,'LineWidth',measured_width); hold on;
plot(time,desired_position(:,2) + 0.112226,'-.','Color',default_blue,'LineWidth',reference_width); hold on
grid on; xlim(twin); ylim([-0.5 0.5])

ax_posz = nexttile(tl11);  % position z
plot(time,position(:,3) - 0.051513,'-','Color',default_red,'LineWidth',measured_width); hold on
plot(time,desired_position(:,3) - 0.051513,'--','Color',default_blue,'LineWidth',reference_width); hold on
grid on; xlim(twin); ylim([-1.3 -0.3])

% 원하는 tick
ax_posz.YTick = [-1.3 -0.8 -0.3];
ax_posz.YTickMode = 'manual';

% =========================
% dhat (manual layout) [row1, col2]
% x/y/z heights unchanged, disturbance norm = 3x
% =========================

dhat_norm_scale = 3.0;   % disturbance norm height = 3x of x/y/z

% base unit height = same as one regular subplot height
base_h = tileH;

% total panel height:
% x + y + z + norm(3x) = 6 * base_h
H_dhat = (3 + dhat_norm_scale) * base_h;

% keep the TOP aligned with the original H4 panel
p12_top = row1_y + H4;
p12_y   = p12_top - H_dhat;

p12 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x2 p12_y colW H_dhat], ...
    'BackgroundColor','w');

% ---- inner margins inside p12 ----
inner_left   = 0.08;
inner_right  = 0.03;
inner_top    = 0.03;
inner_bottom = 0.05;
inner_gap    = 0.03;

axW = 1 - inner_left - inner_right;
availH_inner = 1 - inner_top - inner_bottom - 3*inner_gap;

unitH = availH_inner / (3 + dhat_norm_scale);

h1 = unitH;
h2 = unitH;
h3 = unitH;
h4 = dhat_norm_scale * unitH;

y4 = inner_bottom;
y3 = y4 + h4 + inner_gap;
y2 = y3 + h3 + inner_gap;
y1 = y2 + h2 + inner_gap;

% ---- dhat x ----
ax_dhatx = axes('Parent',p12,'Units','normalized', ...
    'Position',[inner_left y1 axW h1]);
plot(ax_dhatx, time, torque_dhat(:,1), 'Color',default_red, 'LineWidth',reference_width)
grid(ax_dhatx,'on')
xlim(ax_dhatx,twin)
ylim(ax_dhatx,[-2 2])

% ---- dhat y ----
ax_dhaty = axes('Parent',p12,'Units','normalized', ...
    'Position',[inner_left y2 axW h2]);
plot(ax_dhaty, time, torque_dhat(:,2), 'Color',default_red, 'LineWidth',reference_width)
grid(ax_dhaty,'on')
xlim(ax_dhaty,twin)
ylim(ax_dhaty,[-2 2])

% ---- dhat z ----
ax_dhatz = axes('Parent',p12,'Units','normalized', ...
    'Position',[inner_left y3 axW h3]);
plot(ax_dhatz, time, torque_dhat(:,3), 'Color',default_red, 'LineWidth',reference_width)
grid(ax_dhatz,'on')
xlim(ax_dhatz,twin)
ylim(ax_dhatz,[-2 2])

% ---- disturbance norm (3x height) ----
ax_dhatn = axes('Parent',p12,'Units','normalized', ...
    'Position',[inner_left y4 axW h4]);

window_size = 1500;
dhat_norm = sqrt(sum(torque_dhat.^2,2));
dhat_rms  = sqrt(movmean(dhat_norm.^2,window_size));

plot(ax_dhatn, time, dhat_norm, 'Color',light_red, 'LineWidth',norm_width); hold(ax_dhatn,'on')
plot(ax_dhatn, time, dhat_rms, '-k', 'LineWidth',rms_width)

grid(ax_dhatn,'on')
xlim(ax_dhatn,twin)
ylim(ax_dhatn,[0 4.0])
% =========================
% Attitude error (4 tiles) [row2, col1]
% =========================
p21 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x1 row2_y colW H4],'BackgroundColor','w');
tl21 = tiledlayout(p21,4,1,'TileSpacing','compact','Padding','compact');

err_roll  = attitude(:,1)-desired_attitude(:,1);
err_pitch = attitude(:,2)-desired_attitude(:,2);
err_yaw   = unwrap(attitude(:,3))-unwrap(desired_attitude(:,3));

ax_er = nexttile(tl21);
plot(time,err_roll,'Color',default_red,'LineWidth',measured_width); hold on
plot(time,zeros(size(time)),'--','Color',default_blue,'LineWidth',reference_width); hold on
grid on; xlim(twin); ylim([-0.1 0.1])

ax_ep = nexttile(tl21);
plot(time,err_pitch,'Color',default_red,'LineWidth',measured_width); hold on
plot(time,zeros(size(time)),'--','Color',default_blue,'LineWidth',reference_width); hold on
grid on; xlim(twin); ylim([-0.1 0.1])

ax_ey = nexttile(tl21);
plot(time,err_yaw,'Color',default_red,'LineWidth',measured_width); hold on;
plot(time,zeros(size(time)),'--','Color',default_blue,'LineWidth',reference_width); hold on
grid on; xlim(twin); ylim([-0.1 0.1])

ax_en = nexttile(tl21);
att_err_mat  = [err_roll err_pitch err_yaw];
att_err_norm = sqrt(sum(att_err_mat.^2,2));
att_err_rms  = sqrt(movmean(att_err_norm.^2,window_size));

plot(time,att_err_norm,'Color',light_red,'LineWidth',norm_width); hold on
plot(time,att_err_rms,'-k','LineWidth',rms_width)
grid on; xlim(twin); ylim([0 0.1])

% =========================
% CoM (single plot) [row2, col2]
% =========================
p22 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x2 row2_y + (rowH-H3) colW H3], ...
    'BackgroundColor','w');

ax_com = axes('Parent',p22);
hold(ax_com,'on')

scale = 1000; % m -> mm

% reference lines
% yline(0*scale,'--','Color',[0.8500 0.3250 0.0980],'LineWidth',reference_width);
% yline(0.02436*scale,'--','Color',[0 0.4470 0.7410],'LineWidth',reference_width);
% yline(-0.0442*scale,'--','Color',[0.4660 0.6740 0.1880],'LineWidth',reference_width);

% CoM estimate (old version)
% plot(time,past_com(:,1)*scale,'Color',[0.8500 0.3250 0.0980],'LineWidth',reference_width)
% plot(time,past_com(:,2)*scale,'Color',[0 0.4470 0.7410],'LineWidth',reference_width)
% plot(time,past_com(:,3)*scale,'Color',[0.4660 0.6740 0.1880],'LineWidth',reference_width)

% colors
grey_est = [0.35 0.35 0.35];   % estimate z
grey_ref = [0.70 0.70 0.70];   % reference z (lighter)

light_r = [1.00 0.45 0.45];
light_b = [0.45 0.60 1.00];

% CoM estimate
plot(time, past_com(:,1)*scale, '-r', 'LineWidth',measured_width)
plot(time, past_com(:,2)*scale, '-b', 'LineWidth',measured_width)
plot(time, past_com(:,3)*scale, '-', 'Color',grey_est, 'LineWidth',measured_width)

% reference lines
yline(0.00095*scale,'--','Color',light_r,'LineWidth',reference_width);
yline(0.022*scale,'--','Color',light_b,'LineWidth',reference_width);
yline(-0.0458*scale,'--','Color',grey_ref,'LineWidth',reference_width);

grid on
xlim(twin)
ylim([-60 40])
yticks([-60 -40 -20 0 20 40])
% =========================
% Disturbance norm panel (single) [row1, col3]
% =========================
panel_scale = 0.6;   % 세로 크기 (0.6~0.8 추천)

p13 = uipanel('Parent',f,'Units','normalized', ...
    'Position',[x3 row1_y + (rowH - rowH*panel_scale) colW rowH*panel_scale], ...
    'BackgroundColor','w');

ax_dn = axes('Parent',p13);
hold(ax_dn,'on')

plot(ax_dn, time, dhat_norm, 'Color', light_red, 'LineWidth', 5);
plot(ax_dn, time, dhat_rms, '-k', 'LineWidth', rms_width);

grid(ax_dn,'on')
xlim(ax_dn,[20 100])
ylim(ax_dn,[0 0.5])
ax_dn.YLimMode = 'manual';


% =========================
% Global style: y-ticks 3개 고정 + 0 포함 (position(3) 제외)
% =========================
axs = findall(f,'Type','axes');

for k = 1:length(axs)
    ax = axs(k);

    % ---- CoM subplot들: ytick 고정 ----
    if (exist('ax_cx','var') && isequal(ax, ax_cx)) || ...
       (exist('ax_cy','var') && isequal(ax, ax_cy)) || ...
       (exist('ax_cz','var') && isequal(ax, ax_cz))
        ax.YTick = [-40 -10 20];
        ax.YTickMode = 'manual';
        continue
    end

    yl = ylim(ax);

    if (exist('ax_posz','var') && isequal(ax, ax_posz)) || ...
       isequal(ax, ax_com)
        continue
    end


    % ---- 0을 반드시 포함하도록 3개 tick 구성 ----
    if yl(1) < 0 && yl(2) > 0
        % 이미 0이 범위 안에 있음: [min, 0, max]
        ax.YTick = [yl(1), 0, yl(2)];

    elseif yl(1) == 0 && yl(2) > 0
        % [0, mid, max]
        ax.YTick = [0, 0.5*yl(2), yl(2)];

    elseif yl(2) == 0 && yl(1) < 0
        % [min, mid, 0]
        ax.YTick = [yl(1), 0.5*yl(1), 0];

    else
        % 0이 범위 밖이면(드물지만) 범위를 0 포함하도록 확장
        if yl(2) < 0
            ylim(ax, [yl(1), 0]);
            ax.YTick = [yl(1), 0.5*yl(1), 0];
        elseif yl(1) > 0
            ylim(ax, [0, yl(2)]);
            ax.YTick = [0, 0.5*yl(2), yl(2)];
        else
            % 안전장치
            ax.YTick = linspace(yl(1), yl(2), 3);
        end
    end

    ax.YTickMode = 'manual';
end

set(axs,'FontSize',font_size,'Color','w')
set(axs,'TickDir','in','Box','on','LineWidth',1.0)
for ax = axs.'
    ax.LooseInset = ax.TightInset;
end
% ---- 20초 간격 grid ----
set(axs,'XTick',twin(1):20:twin(2))

% ---- Y label + time label ON/OFF ----
if show_y_labels
    set(axs,'YTickLabelMode','auto')
    set(axs,'XTickLabelMode','auto')   % time 숫자 표시
else
    set(axs,'YTickLabel',[])
    set(axs,'XTickLabel',[])           % time 숫자 숨김
end

% Save each panel as figure
% =========================

saveDir = fullfile(path, "panels");
if ~exist(saveDir, 'dir')
    mkdir(saveDir);
end

exportgraphics(p11, fullfile(saveDir,"panel_position.png"), 'Resolution',300);
exportgraphics(p12, fullfile(saveDir,"panel_dhat.png"),     'Resolution',300);
exportgraphics(p21, fullfile(saveDir,"panel_attitude.png"), 'Resolution',300);
exportgraphics(p22, fullfile(saveDir,"panel_com.png"),      'Resolution',300);
exportgraphics(p13, fullfile(saveDir,"panel_disturbance_norm.png"), 'Resolution',300);
disp("Panels saved.")
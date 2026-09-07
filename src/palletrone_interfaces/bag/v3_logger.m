
clear all; close all; clc;


%% 0) Pick CSV (folder-based UI)
%defaultDir = fullfile(getenv("HOME"), "sitl_ws", "src", "px4_visual", "bag", "from_latte");
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

% 1) Read table
data = readtable(csv_path, 'VariableNamingRule', 'preserve');

% 3. 시간 벡터
time = data{:,1} * 1e-9;  % 첫 번째 열은 상대 시간(sec)

% 4. 데이터 행렬 (나머지 열들)
values = data{:,2:end};  % 각 column은 data[0], data[1], ...
values_size = size(values(:,1));

%
position                     =zeros(values_size(1),3);
desired_position             =zeros(values_size(1),3);

linear_velocity              =zeros(values_size(1),3);
desired_linear_velocity      =zeros(values_size(1),3);

attitude                     =zeros(values_size(1),3);
desired_attitude             =zeros(values_size(1),3);

angular_velocity             =zeros(values_size(1),3);
desired_angular_velocity     =zeros(values_size(1),3);

desired_force                =zeros(values_size(1),3);

desired_torque               =zeros(values_size(1),4);
torque_dhat                  =zeros(values_size(1),3);

individual_motor_thrust      =zeros(values_size(1),4);

servo_angle                  =zeros(values_size(1),5);
desired_servo_angle          =zeros(values_size(1),5);

acceleration                 =zeros(values_size(1),3);
desired_acceleration         =zeros(values_size(1),3);

present_com                  =zeros(values_size(1),3);
past_com                     =zeros(values_size(1),3);
com_tilde                    =zeros(values_size(1),3);
com_update                   =zeros(values_size(1),3);

PWM_cmd                      =zeros(values_size(1),8);
% ===== L1 Adaptive =====
l1_dhat_tau                  = zeros(values_size(1),3);
l1_tau_comp_raw              = zeros(values_size(1),3);
l1_tau_comp_lpf              = zeros(values_size(1),3);

% Main_Drone Data_logging
%------------------------------------------------------------%
position(:,1)                     =values(:,2);
position(:,2)                     =values(:,3);
position(:,3)                     =values(:,4);
desired_position(:,1)             =values(:,5);
desired_position(:,2)             =values(:,6);
desired_position(:,3)             =values(:,7);
%------------------------------------------------------------%
linear_velocity(:,1)              =values(:,8);
linear_velocity(:,2)              =values(:,9);
linear_velocity(:,3)              =values(:,10);
desired_linear_velocity(:,1)      =values(:,11);
desired_linear_velocity(:,2)      =values(:,12);
desired_linear_velocity(:,3)      =values(:,13);
%------------------------------------------------------------%
attitude(:,1)                     =values(:,14);
attitude(:,2)                     =values(:,15);
attitude(:,3)                     =values(:,16);
desired_attitude(:,1)             =values(:,17);
desired_attitude(:,2)             =values(:,18);
desired_attitude(:,3)             =values(:,19);
%------------------------------------------------------------%
angular_velocity(:,1)             =values(:,20);
angular_velocity(:,2)             =values(:,21);
angular_velocity(:,3)             =values(:,22);
desired_angular_velocity(:,1)     =values(:,23);
desired_angular_velocity(:,2)     =values(:,24);
desired_angular_velocity(:,3)     =values(:,25);
%------------------------------------------------------------%
desired_force(:,1)                =values(:,26);
desired_force(:,2)                =values(:,27);
desired_force(:,3)                =values(:,28);
%------------------------------------------------------------%
desired_torque(:,1)               =values(:,29);
desired_torque(:,2)               =values(:,30);
desired_torque(:,3)               =values(:,31);
desired_torque(:,4)               =values(:,32);
%------------------------------------------------------------%
torque_dhat(:,1)                  =values(:,33);
torque_dhat(:,2)                  =values(:,34);
torque_dhat(:,3)                  =values(:,35);
%------------------------------------------------------------%
individual_motor_thrust(:,1)      =values(:,36);
individual_motor_thrust(:,2)      =values(:,37);
individual_motor_thrust(:,3)      =values(:,38);
individual_motor_thrust(:,4)      =values(:,39);
%------------------------------------------------------------%
servo_angle(:,1)                  =values(:,40);
servo_angle(:,2)                  =values(:,41);
servo_angle(:,3)                  =values(:,42);
servo_angle(:,4)                  =values(:,43);
servo_angle(:,5)                  =values(:,44);
%------------------------------------------------------------%
desired_servo_angle(:,1)          =values(:,45);
desired_servo_angle(:,2)          =values(:,46);
desired_servo_angle(:,3)          =values(:,47);
desired_servo_angle(:,4)          =values(:,48);
desired_servo_angle(:,5)          =values(:,49);
%------------------------------------------------------------%
acceleration(:,1)                 =values(:,50);
acceleration(:,2)                 =values(:,51);
acceleration(:,3)                 =values(:,52);
desired_acceleration(:,1)         =values(:,53);
desired_acceleration(:,2)         =values(:,54);
desired_acceleration(:,3)         =values(:,55);
%------------------------------------------------------------%
present_com(:,1)                  =values(:,56);
present_com(:,2)                  =values(:,57);
present_com(:,3)                  =values(:,58);
%------------------------------------------------------------%
past_com(:,1)                     =values(:,59);
past_com(:,2)                     =values(:,60);
past_com(:,3)                     =values(:,61);
%------------------------------------------------------------%
com_tilde(:,1)                    =values(:,62);
com_tilde(:,2)                    =values(:,63);
com_tilde(:,3)                    =values(:,64);
%------------------------------------------------------------%
com_update(:,1)                   =values(:,65);
com_update(:,2)                   =values(:,66);
com_update(:,3)                   =values(:,67);
%------------------------------------------------------------%
% L1 adaptive disturbance estimate
l1_dhat_tau(:,1)                  = values(:,86);
l1_dhat_tau(:,2)                  = values(:,87);
l1_dhat_tau(:,3)                  = values(:,88);

% L1 adaptive raw compensation torque
l1_tau_comp_raw(:,1)              = values(:,89);
l1_tau_comp_raw(:,2)              = values(:,90);
l1_tau_comp_raw(:,3)              = values(:,91);

% L1 adaptive LPF compensation torque
l1_tau_comp_lpf(:,1)              = values(:,92);
l1_tau_comp_lpf(:,2)              = values(:,93);
l1_tau_comp_lpf(:,3)              = values(:,94);
%------------------------------------------------------------%

% ===== Force -> pwm_scaled (0~1), EXACT C behavior =====

a = 2.0962e-05;
b = 0.0085;
c = -36.0347;

pwm_min = 1100;
pwm_max = 1900;

F = individual_motor_thrust;   % (N x 4)

disc = b.^2 - 4*a*(c - F);

pwm_scaled = zeros(size(F));   % output 0~1

valid = disc >= 0;

pwm = zeros(size(F)); 
pwm(valid) = (-b + sqrt(disc(valid))) ./ (2*a);

pwm_scaled(valid) = (pwm(valid) - pwm_min) ./ (pwm_max - pwm_min);
pwm_scaled(~valid) = 0;   % C 코드와 동일

l = 0.028;  % [m] 레버암 길이 (서보축~등가CoM 거리). 네 값으로 바꿔!

theta = servo_angle(:,5);          % [rad] meas
theta_des = desired_servo_angle(:,5);

true_com = zeros(length(time),3);
true_com(:,1) =  l*cos(theta);
true_com(:,2) = -l*sin(theta);
true_com(:,3) =  0;


%% 5. 대시보드 (3x2 blocks)  [panel+manual layout style]
twin = [0 80];

f = figure('Name', 'DataLogging Overview (Pos/Att/dhat/CoM/PWM/Servo)', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0 0 1 1]);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 3; nrow = 2;

w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;

getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Position XYZ (meas vs des)
p11  = uipanel('Parent', f, 'Position', getPos(1,1), 'BackgroundColor', 'w');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl11, 1);
plot(time, desired_position(:,1) , '--', 'LineWidth', 1.2); hold on;
plot(time, position(:,1), '-', 'LineWidth', 1.2);
grid on; xlim(twin); 
ylim([-0.6 0.6])
title('Position')

nexttile(tl11, 2);
plot(time, desired_position(:,2) , '--', 'LineWidth', 1.2); hold on;
plot(time, position(:,2), '-', 'LineWidth', 1.2);
grid on; xlim(twin); 
ylim([-0.6 0.6]);

nexttile(tl11, 3);
plot(time, desired_position(:,3) , '--', 'LineWidth', 1.2); hold on;
plot(time, position(:,3), '-', 'LineWidth', 1.2);
grid on; xlim(twin); 
ylim([-1.1 -0.4]);
xlabel('time [s]');

% (2,1) Attitude Error (RPY + Norm + Moving RMS)  [yaw unwrap]
p21  = uipanel('Parent', f, 'Position', getPos(2,1), 'BackgroundColor', 'w');
tl21 = tiledlayout(p21, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% =========================
% Roll error
% =========================
nexttile(tl21, 1);
err_roll = attitude(:,1) - desired_attitude(:,1);

plot(time, zeros(size(time)), '--', 'LineWidth', 1.2); hold on;
plot(time, err_roll, '-', 'LineWidth', 1.2);
grid on; xlim(twin);
ylim([-0.1 0.1])
title('Attitude Error');

% =========================
% Pitch error
% =========================
nexttile(tl21, 2);
err_pitch = attitude(:,2) - desired_attitude(:,2);

plot(time, zeros(size(time)), '--', 'LineWidth', 1.2); hold on;
plot(time, err_pitch, '-', 'LineWidth', 1.2);
grid on; xlim(twin);
ylim([-0.1 0.1])

% =========================
% Yaw error (unwrap 적용)
% =========================
nexttile(tl21, 3);
yaw_meas_u = unwrap(attitude(:,3));
yaw_des_u  = unwrap(desired_attitude(:,3));
err_yaw = yaw_meas_u - yaw_des_u;

plot(time, zeros(size(time)), '--', 'LineWidth', 1.2); hold on;
plot(time, err_yaw, '-', 'LineWidth', 1.2);
grid on; xlim(twin);
ylim([-0.1 0.1])

% =========================
% Attitude Error Norm + Moving RMS
% =========================
nexttile(tl21, 4);

att_err_mat = [err_roll, err_pitch, err_yaw];
att_err_norm = sqrt(sum(att_err_mat.^2, 2));

% ---- Moving RMS ----
window_size = 500;   % 샘플 개수 (예: 100Hz면 약 2초)
att_err_rms = sqrt(movmean(att_err_norm.^2, window_size));

plot(time, att_err_norm,  'LineWidth', 4.0); hold on;
plot(time, att_err_rms,  'r', 'LineWidth', 2);

grid on; xlim(twin); ylim([0 0.12])
ylabel('||Q||');
xlabel('time [s]');

% (1,2) dhat XYZ
p12  = uipanel('Parent', f, 'Position', getPos(1,2), 'BackgroundColor', 'w');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl12, 1);
plot(time, torque_dhat(:,1), 'LineWidth', 1.2);
grid on; xlim(twin); 
ylim([-3.5 1])
title('Disturbance Observer Torque');

nexttile(tl12, 2);
plot(time, torque_dhat(:,2), 'LineWidth', 1.2);
grid on; xlim(twin); 
ylim([-3.5 1])

nexttile(tl12, 3);
plot(time, torque_dhat(:,3), 'LineWidth', 1.2);
grid on; xlim(twin); 
ylim([-2.25 2.25])
xlabel('time [s]');

% (2,2) CoM XYZ (present / past / update)
p22  = uipanel('Parent', f, 'Position', getPos(2,2), 'BackgroundColor', 'w');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl22, 1);
yline(0.033, '--r', 'LineWidth', 2); hold on;
plot(time, past_com(:,1),    '-', 'LineWidth', 1.8);
grid on; xlim(twin); 
ylim([-0.04 0.04])
title('CoM Estimate');

nexttile(tl22, 2);
yline(-0.033, '--r', 'LineWidth', 2); hold on;
plot(time, past_com(:,2),    '-', 'LineWidth', 1.8);
grid on; xlim(twin); 
ylim([-0.04 0.04])

nexttile(tl22, 3);
yline(-0.06, '--r', 'LineWidth', 2); hold on;
plot(time, past_com(:,3),    '-', 'LineWidth', 1.8);
grid on; xlim(twin); 
ylim([-0.1 0.01])
xlabel('time [s]');

% (1,3) PWM (scaled) 4x1
p13  = uipanel('Parent', f, 'Position', getPos(1,3), 'BackgroundColor', 'w');
tl13 = tiledlayout(p13, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl13, 1);
plot(time, pwm_scaled(:,1), '-', 'LineWidth', 0.8);
grid on; xlim(twin); ylim([0 1]);
title('PWM');

nexttile(tl13, 2);
plot(time, pwm_scaled(:,2), '-', 'LineWidth', 0.8);
grid on; xlim(twin); ylim([0 1]);

nexttile(tl13, 3);
plot(time, pwm_scaled(:,3), '-', 'LineWidth', 0.8);
grid on; xlim(twin); ylim([0 1]);

nexttile(tl13, 4);
plot(time, pwm_scaled(:,4), '-', 'LineWidth', 0.8);
grid on; xlim(twin); ylim([0 1]);
xlabel('time [s]');

% (2,3) Servo angles 5x1 (des vs meas)
p23  = uipanel('Parent', f, 'Position', getPos(2,3), 'BackgroundColor', 'w');
tl23 = tiledlayout(p23, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:5
    nexttile(tl23, i);
    plot(time, desired_servo_angle(:,i), '--', 'LineWidth', 0.8); hold on;
    plot(time, servo_angle(:,i), '-',  'LineWidth', 0.8);
    grid on; xlim(twin); ylim([-0.7 0.7])
    if i == 1
        title('Servo')
    end
    if i == 5
        xlabel('time [s]'); ylim ([0 3.2])
    end
end

% 공통 스타일
set(findall(f, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% ===== 5-in-1 Dashboard (3x2 panels) : Att / Pos / dhat / CoM / Acc =====
twin = [10 60];     % <<<<<< 여기만 바꾸면 xlim 전체 통일
windowSize = 100;    % acceleration smoothing window

f = figure('Name', 'Overview (Att / Pos / dhat / CoM / Acc)', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0.05 0.05 0.9 0.9]);

% manual layout params (same vibe)
left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 3; nrow = 2;   % <<< 3x2로 확장

w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;

getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) Attitude RPY (meas vs des)
p11  = uipanel('Parent', f, 'Position', getPos(1,1), 'BackgroundColor', 'w');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axA1 = nexttile(tl11, 1);
plot(time, desired_attitude(:,1), '-', 'LineWidth', 2.0); hold on;
plot(time, attitude(:,1), '-', 'LineWidth', 1.7);
grid on; xlim(twin);
ylabel('$\phi$ [rad]','Interpreter','latex','FontSize',12);
title('Attitude')

axA2 = nexttile(tl11, 2);
plot(time, desired_attitude(:,2), '-', 'LineWidth', 2.0); hold on;
plot(time, attitude(:,2), '-', 'LineWidth', 1.7);
grid on; xlim(twin);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',12);

axA3 = nexttile(tl11, 3);
plot(time, desired_attitude(:,3), '-', 'LineWidth', 2.0); hold on;
plot(time, attitude(:,3), '-', 'LineWidth', 1.7);
grid on; xlim(twin);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$\psi$ [rad]','Interpreter','latex','FontSize',12);

linkaxes([axA1 axA2],'y');
ylim(axA1,[-0.3 0.3]);
ylim(axA3,[-0.3 0.3]);

% (1,2) XYZ Position (meas vs des)
p12  = uipanel('Parent', f, 'Position', getPos(1,2), 'BackgroundColor', 'w');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl12, 1);
plot(time, desired_position(:,1), '-', 'LineWidth', 2.0); hold on;
plot(time, position(:,1), '-', 'LineWidth', 1.7);
grid on; xlim(twin); ylim([-0.1 0.7]);
ylabel('$x$ [m]','Interpreter','latex','FontSize',12);
title('Position')

nexttile(tl12, 2);
plot(time, desired_position(:,2), '-', 'LineWidth', 2.0); hold on;
plot(time, position(:,2), '-', 'LineWidth', 1.7);
grid on; xlim(twin); ylim([-0.4 0.4]);
ylabel('$y$ [m]','Interpreter','latex','FontSize',12);

nexttile(tl12, 3);
plot(time, desired_position(:,3), '-', 'LineWidth', 2.0); hold on;
plot(time, position(:,3), '-', 'LineWidth', 1.7);
grid on; xlim(twin); ylim([-0.8 0.0]);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$z$ [m]','Interpreter','latex','FontSize',12);

% (2,1) DOB disturbance hat (torque_dhat)
p21  = uipanel('Parent', f, 'Position', getPos(2,1), 'BackgroundColor', 'w');
tl21 = tiledlayout(p21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl21, 1);
plot(time, torque_dhat(:,1), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([-1 1]);
ylabel('$\hat{\tau}_x$ [Nm]','Interpreter','latex','FontSize',12);
title('DOB $\hat{\tau}$','Interpreter','latex')

nexttile(tl21, 2);
plot(time, torque_dhat(:,2), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([-1 1]);
ylabel('$\hat{\tau}_y$ [Nm]','Interpreter','latex','FontSize',12);

nexttile(tl21, 3);
plot(time, torque_dhat(:,3), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([-1 1]);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$\hat{\tau}_z$ [Nm]','Interpreter','latex','FontSize',12);

% (2,2) Center of Mass Hat (com_update)
p22  = uipanel('Parent', f, 'Position', getPos(2,2), 'BackgroundColor', 'w');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl22, 1);
plot(time, com_update(:,1), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([-0.001 0.03]);
ylabel('$c_x$ [m]','Interpreter','latex','FontSize',12);
title('CoM Estimate')

nexttile(tl22, 2);
plot(time, com_update(:,2), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([-0.001 0.03]);
ylabel('$c_y$ [m]','Interpreter','latex','FontSize',12);

nexttile(tl22, 3);
plot(time, com_update(:,3)*0.66, '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([-0.03 0.001]);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$c_z$ [m]','Interpreter','latex','FontSize',12);

% (1,3) Acceleration (filtered X/Y, raw Z)  <<< 추가된 패널
p13  = uipanel('Parent', f, 'Position', getPos(1,3), 'BackgroundColor', 'w');
tl13 = tiledlayout(p13, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl13, 1);
plot(time, movmean(acceleration(:,1), windowSize), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([-1.5 1.5]);
title('Acceleration')
ylabel('$a_x$ [m/s$^2$]','Interpreter','latex','FontSize',12);

nexttile(tl13, 2);
plot(time, movmean(acceleration(:,2), windowSize), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([-1.5 1.5]);
ylabel('$a_y$ [m/s$^2$]','Interpreter','latex','FontSize',12);

nexttile(tl13, 3);
plot(time, acceleration(:,3), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([-12 -8]);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$a_z$ [m/s$^2$]','Interpreter','latex','FontSize',12);

% (2,3) 빈칸 (원하면 여기에 norm plot / PWM / Servo 등 추가 가능)
p23  = uipanel('Parent', f, 'Position', getPos(2,3), 'BackgroundColor', 'w');
uicontrol('Parent', p23, 'Style', 'text', 'String', 'Reserved', ...
    'Units', 'normalized', 'Position', [0 0 1 1], ...
    'BackgroundColor', 'w', 'ForegroundColor', [0.5 0.5 0.5], ...
    'FontSize', 14);

% common style
set(findall(f, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% Attitude
figure('Position',[100 100 600 800], 'Color','w');

% attitude error
att_err = desired_attitude - attitude;

% norm (instantaneous L2 norm)
att_err_norm = sqrt(sum(att_err.^2, 2));

% RMS (running RMS)
att_err_rms = sqrt(movmean(att_err_norm.^2, 200));  
% 200 sample window (원하면 조절)

subplot(4,1,1);
plot(time,desired_attitude(:,1),'-','LineWidth',1); hold on
plot(time,attitude(:,1),'-','LineWidth',1); grid on
ylabel('$\phi$ [rad]','Interpreter','latex','FontSize',14);
xlim([0 50])
ylim([-0.1 0.1])

subplot(4,1,2);
plot(time,desired_attitude(:,2),'-','LineWidth',1); hold on
plot(time,attitude(:,2),'-','LineWidth',1); grid on
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',14);
xlim([0 50])
ylim([-0.1 0.1])

subplot(4,1,3);
plot(time,desired_attitude(:,3),'-','LineWidth',1); hold on
plot(time,attitude(:,3),'-','LineWidth',1); grid on
ylabel('$\psi$ [rad]','Interpreter','latex','FontSize',14);
xlim([0 50])
ylim([-0.1 0.1])

subplot(4,1,4);
plot(time,att_err_norm,'LineWidth',4); hold on
plot(time,att_err_rms,'r','LineWidth',2);
grid on
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$||e_{\eta}||$','Interpreter','latex','FontSize',14);
xlim([0 50])
ylim([0 0.1])


%%  XYZ Position
figure('Position',[100 100 600 600], 'Color','w');

subplot(3,1,1);
plot(time,desired_position(:,1),'-','LineWidth',2.5); hold on
plot(time,position(:,1) ,'LineWidth',2.0); grid on
ylabel('$x$ [m]','Interpreter','latex','FontSize',14);
xlim([0 50])
ylim([-0.4 0.4])

subplot(3,1,2);
plot(time,desired_position(:,2) ,'-','LineWidth',2.5); hold on
plot(time,position(:,2) ,'LineWidth',2.0); grid on
ylabel('$y$ [m]','Interpreter','latex','FontSize',14);
xlim([0 50])
ylim([-0.4 0.4]);

subplot(3,1,3);
plot(time,desired_position(:,3) ,'-','LineWidth',2.5); hold on
plot(time,position(:,3) ,'LineWidth',2.0); grid on
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$z$ [m]','Interpreter','latex','FontSize',14);
xlim([0 50])
ylim([-1. -0.4]);


%% DOB disturbance hat
figure('Position',[100 100 600 600], 'Color','w');

ax1 = subplot(3,1,1);
plot(time,torque_dhat(:,1),'-','LineWidth',2.0); grid on
ylabel('$\hat{\tau}_x$ [Nm]','Interpreter','latex','FontSize',14);
ylim([-1,1])

ax2 = subplot(3,1,2);
plot(time,torque_dhat(:,2),'-','LineWidth',2.0); grid on
ylabel('$\hat{\tau}_y$ [Nm]','Interpreter','latex','FontSize',14);
ylim([-1,1])

ax3 = subplot(3,1,3);
plot(time,torque_dhat(:,3),'-','LineWidth',2.0); grid on
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$\hat{\tau}_z$ [Nm]','Interpreter','latex','FontSize',14);
ylim([-1,1])

% === Y축 한번에 통일 ===

%% Center of Mass Hat
figure('Position',[100 100 600 600], 'Color','w');

ax1 = subplot(4,1,1);
plot(time,com_update(:,1),'-','LineWidth',2.0); grid on
ylabel('$c_x$ [m]','Interpreter','latex','FontSize',14);
xlim([38 153])
ylim([-0.02 0.04])

ax2 = subplot(4,1,2);
plot(time,com_update(:,2),'-','LineWidth',2.0); grid on
ylabel('$c_y$ [m]','Interpreter','latex','FontSize',14);
xlim([38 153])
ylim([-0.02 0.04])

ax3 = subplot(4,1,3);
plot(time,com_update(:,3),'-','LineWidth',2.0); grid on
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$c_z$ [m]','Interpreter','latex','FontSize',14);
xlim([38 153])
ylim([-0.03 0.03])


ax3 = subplot(4,1,4);
plot(time,desired_servo_angle(:,5),'-','LineWidth',2.0);
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$c_z$ [m]','Interpreter','latex','FontSize',14);
xlim([38 153])
ylim([0 6])

%% angular velocity
figure
subplot(3,1,1)
plot(time,desired_angular_velocity(:,1),'-r','LineWidth',1.0); hold on
plot(time,angular_velocity(:,1),'-b','LineWidth',1.0); title('roll\_velocity'); grid on
xlim([30 50])

subplot(3,1,2)
plot(time,desired_angular_velocity(:,2),'-r','LineWidth',1.0); hold on
plot(time,angular_velocity(:,2),'-b','LineWidth',1.0); title('pitch\_velocity'); grid on
xlim([30 50])

subplot(3,1,3)
plot(time,desired_angular_velocity(:,3),'-r','LineWidth',1.0); hold on
plot(time,angular_velocity(:,3),'-b','LineWidth',1.0); title('yaw\_velocity'); grid on
xlim([30 50])

%% Servo_angle
figure
subplot(5,1,1); plot(time,desired_servo_angle(:,1),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,1),'-','LineWidth',1.0); grid on; title('Servo\_1'); 
subplot(5,1,2); plot(time,desired_servo_angle(:,2),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,2),'-','LineWidth',1.0); grid on; title('Servo\_2'); 
subplot(5,1,3); plot(time,desired_servo_angle(:,3),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,3),'-','LineWidth',1.0); grid on; title('Servo\_3'); 
subplot(5,1,4); plot(time,desired_servo_angle(:,4),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,4),'-','LineWidth',1.0); grid on; title('Servo\_4'); 
subplot(5,1,5); plot(time,desired_servo_angle(:,5),'-','LineWidth',2.0); hold on; plot(time,servo_angle(:,5),'-','LineWidth',1.0); grid on; title('Servo\_5'); 

%% Force
figure
subplot(4,1,1); plot(time,individual_motor_thrust(:,1),'-','LineWidth',2.0); grid on; title('F\_1'); 
subplot(4,1,2); plot(time,individual_motor_thrust(:,2),'-','LineWidth',2.0); grid on; title('F\_2');
subplot(4,1,3); plot(time,individual_motor_thrust(:,3),'-','LineWidth',2.0); grid on; title('F\_3'); 
subplot(4,1,4); plot(time,individual_motor_thrust(:,4),'-','LineWidth',2.0); grid on; title('F\_4'); 

%% PWM
figure
subplot(4,1,1); plot(time,pwm_scaled(:,1),'-','LineWidth',2.0); grid on; title('pwm1'); ylim([0 1]); 
subplot(4,1,2); plot(time,pwm_scaled(:,2),'-','LineWidth',2.0); grid on; title('pwm2'); ylim([0 1]); 
subplot(4,1,3); plot(time,pwm_scaled(:,3),'-','LineWidth',2.0); grid on; title('pwm3'); ylim([0 1]); 
subplot(4,1,4); plot(time,pwm_scaled(:,4),'-','LineWidth',2.0); grid on; title('pwm4'); ylim([0 1]); 

%% Linear Velocity plot
figure
subplot(3,1,1)
plot(time,desired_linear_velocity(:,1),'-r','LineWidth',2.0); hold on
plot(time,linear_velocity(:,1),'-b','LineWidth',2.0);
ylabel('$\bf{\dot{x}}$ \rm\bf{(m/s)}','Interpreter','latex')
legend('$^{G}\dot{x_d}$','$^{G}\dot{x}$','Interpreter','latex','Orientation','horizontal')
title('Velocity'); grid on

subplot(3,1,2)
plot(time,desired_linear_velocity(:,2),'-r','LineWidth',2.0); hold on
plot(time,linear_velocity(:,2),'-b','LineWidth',2.0);
ylabel('$\bf{\dot{y}}$ \rm\bf{(m/s)}','Interpreter','latex')
legend('$^{G}\dot{y_d}$','$^{G}\dot{y}$','Interpreter','latex','Orientation','horizontal')
grid on

subplot(3,1,3)
plot(time,desired_linear_velocity(:,3),'-r','LineWidth',2.0); hold on
plot(time,linear_velocity(:,3),'-b','LineWidth',2.0);
legend('$^{G}\dot{z}$','Interpreter','latex','Orientation','horizontal')
grid on

%% desired_torque
figure
subplot(4,1,1); plot(time,desired_torque(:,1),'-','LineWidth',2.0); grid on; title('roll\_torque'); 
subplot(4,1,2); plot(time,desired_torque(:,2),'-','LineWidth',2.0); grid on; title('pitch\_torque'); 
subplot(4,1,3); plot(time,desired_torque(:,3),'-','LineWidth',2.0); grid on; title('yaw\_torque');  hold on;
plot(time,desired_torque(:,4),'-','LineWidth',2.0); grid on; title('yaw\_torque trim');
subplot(4,1,4); plot(time,desired_torque(:,4),'-','LineWidth',2.0); grid on; title('yaw\_torque\_trim');

%% Desired force (raw vs Butterworth 2nd-order LPF)
% cutoff: wc = 1.0 rad/s  =>  fc = wc/(2*pi) Hz

wc = 2.0;                 % [rad/s]
fc = wc/(2*pi);           % [Hz]

% sampling frequency from time vector
dt = mean(diff(time));    % [s] (time이 거의 균일 샘플링이라는 가정)
fs = 1/dt;                % [Hz]

% 2nd-order Butterworth LPF
Wn = fc/(fs/2);           % normalized cutoff (0~1)
Wn = min(max(Wn, 1e-6), 0.999999);  % 안전 클램핑

[b,a] = butter(2, Wn, 'low');

% zero-phase filtering (원본과 위상 맞춰 비교하기 좋음)
desired_force_filt = filtfilt(b, a, desired_force);

figure
subplot(3,1,1);
plot(time, desired_force(:,1),      '-b','LineWidth',2.0); hold on;
plot(time, desired_force_filt(:,1), '-r','LineWidth',2.0); grid on;
title('Fx\_desired (raw vs filt)'); legend('raw','butter2 wc=1rad/s');

subplot(3,1,2);
plot(time, desired_force(:,2),      '-b','LineWidth',2.0); hold on;
plot(time, desired_force_filt(:,2), '-r','LineWidth',2.0); grid on;
title('Fy\_desired (raw vs filt)'); legend('raw','butter2 wc=1rad/s');

subplot(3,1,3);
plot(time, desired_force(:,3),      '-b','LineWidth',2.0); hold on;
plot(time, desired_force_filt(:,3), '-r','LineWidth',2.0); grid on;
title('Fz\_desired (raw vs filt)'); legend('raw','butter2 wc=1rad/s');
%% acceleration
windowSize = 10;
figure
subplot(3,1,1)
plot(time,desired_acceleration(:,1),'-r','LineWidth',1.0); hold on
plot(time,movmean(acceleration(:,1), windowSize),'-b','LineWidth',1.0); grid on
ylim([-2 2])
title('acceleration\_X')
xlim([130 150])


subplot(3,1,2)
plot(time,desired_acceleration(:,2),'-r','LineWidth',1.0); hold on
plot(time,movmean(acceleration(:,2), windowSize),'-b','LineWidth',1.0); grid on
ylim([-2 2])
title('acceleration\_Y')
xlim([130 150])

subplot(3,1,3)
plot(time,desired_acceleration(:,3),'-r','LineWidth',2.0); hold on
plot(time,acceleration(:,3),'-b','LineWidth',2.0); grid on
ylim([-11 -0])
title('acceleration\_Z')
xlim([130 150])

%% ===== true_com from servo_angle (assumption: 0->+x, pi/2->-y) =====
l = 0.03;  % [m] 레버암 길이 (서보축~등가CoM 거리). 네 값으로 바꿔!

theta = servo_angle(:,5);          % [rad] meas
theta_des = desired_servo_angle(:,5);

% (옵션) 너가 예전에 쓰던 기준 오프셋이 있으면 활성화
% theta_offset = pi/4;
% theta = theta - theta_offset;
% theta_des = theta_des - theta_offset;

true_com = zeros(length(time),3);
true_com(:,1) =  l*cos(theta);
true_com(:,2) = -l*sin(theta);
true_com(:,3) =  0;

true_com_des = zeros(length(time),3);
true_com_des(:,1) =  l*cos(theta_des);
true_com_des(:,2) = -l*sin(theta_des);
true_com_des(:,3) =  0;

% ===== Plot: com_update vs true_com (from servo) =====
figure('Position',[100 100 700 650], 'Color','w');

subplot(4,1,1);
plot(time, com_update(:,1) , '-', 'LineWidth',1.2); hold on;
plot(time, true_com(:,1), '--', 'LineWidth',2.0);
grid on; ylabel('$c_x$ [m]','Interpreter','latex','FontSize',14);
legend('com\_update','true\_com(meas)','Location','best');

subplot(4,1,2);
plot(time, com_update(:,2), '-', 'LineWidth',1.2); hold on;
plot(time, true_com(:,2), '--', 'LineWidth',2.0);
grid on; ylabel('$c_y$ [m]','Interpreter','latex','FontSize',14);

subplot(4,1,3);
plot(time, com_update(:,3), '-', 'LineWidth',1.2); hold on;
plot(time, true_com(:,3), '--', 'LineWidth',2.0);
grid on; ylabel('$c_z$ [m]','Interpreter','latex','FontSize',14);
xlabel('Time [s]','Interpreter','latex','FontSize',14);

subplot(4,1,4);
plot(time, theta_des, '-', 'LineWidth',2.0); hold on;
plot(time, theta,     '-', 'LineWidth',1.0);
grid on; title('Servo\_5'); ylabel('[rad]'); xlabel('Time [s]');
legend('des','meas','Location','best');


%% ===================== 3D Trajectory Plot (meas vs des) =====================
% 원하는 구간 설정 (sec)
t_start = 50;      % <-- 여기 바꿔
t_end   = 130;      % <-- 여기 바꿔

% 인덱스 마스크
idx = (time >= t_start) & (time <= t_end);

% 데이터 슬라이스
p_meas = position(idx, :);
p_des  = desired_position(idx, :);

% --- 3D Plot ---
figure('Position',[120 80 900 700], 'Color','w');
plot3(p_des(:,1),  p_des(:,2),  p_des(:,3),  'r-', 'LineWidth', 2.5); hold on;
plot3(p_meas(:,1), p_meas(:,2), p_meas(:,3), 'b-', 'LineWidth', 2.0);

% 시작/끝 점 표시
plot3(p_des(1,1),  p_des(1,2),  p_des(1,3),  'ro', 'MarkerFaceColor','r', 'MarkerSize',7);
plot3(p_meas(1,1), p_meas(1,2), p_meas(1,3), 'bo', 'MarkerFaceColor','b', 'MarkerSize',7);
plot3(p_des(end,1),  p_des(end,2),  p_des(end,3),  'rs', 'MarkerFaceColor','r', 'MarkerSize',7);
plot3(p_meas(end,1), p_meas(end,2), p_meas(end,3), 'bs', 'MarkerFaceColor','b', 'MarkerSize',7);

grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title(sprintf('3D Trajectory (t = %.2f ~ %.2f s)', t_start, t_end));
legend('Desired (red)', 'Measured (blue)', 'Des start', 'Meas start', 'Des end', 'Meas end', ...
       'Location','best');
view(45, 25);

% (옵션) 보기 편하게 범위 약간 여유
pad = 0.02;
xmin = min([p_des(:,1); p_meas(:,1)]) - pad; xmax = max([p_des(:,1); p_meas(:,1)]) + pad;
ymin = min([p_des(:,2); p_meas(:,2)]) - pad; ymax = max([p_des(:,2); p_meas(:,2)]) + pad;
zmin = min([p_des(:,3); p_meas(:,3)]) - pad; zmax = max([p_des(:,3); p_meas(:,3)]) + pad;
xlim([xmin xmax]); ylim([ymin ymax]); zlim([zmin zmax]);

% (옵션) 2D 투영 (XY / XZ / YZ)
figure('Position',[150 120 900 650], 'Color','w');

subplot(3,1,1);
plot(p_des(:,1),  p_des(:,2),  'r-', 'LineWidth',2.5); hold on;
plot(p_meas(:,1), p_meas(:,2), 'b-', 'LineWidth',2.0);
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); title('XY Projection');

subplot(3,1,2);
plot(p_des(:,1),  p_des(:,3),  'r-', 'LineWidth',2.5); hold on;
plot(p_meas(:,1), p_meas(:,3), 'b-', 'LineWidth',2.0);
grid on; axis equal; xlabel('x [m]'); ylabel('z [m]'); title('XZ Projection');

subplot(3,1,3);
plot(p_des(:,2),  p_des(:,3),  'r-', 'LineWidth',2.5); hold on;
plot(p_meas(:,2), p_meas(:,3), 'b-', 'LineWidth',2.0);
grid on; axis equal; xlabel('y [m]'); ylabel('z [m]'); title('YZ Projection');


%% Center of Mass Hat
figure('Position',[100 100 600 600], 'Color','w');

ax1 = subplot(4,1,1);
plot(time, com_update(:,1), '-', 'LineWidth',2.0); hold on; grid on
plot(time, com_servo(:,2),  '--','LineWidth',2.0);
ylabel('$c_x$ [m]','Interpreter','latex','FontSize',14);
xlim([40 155]); ylim([-0.005 0.03]);
legend('com\_update','com\_servo(q)','Interpreter','latex','Location','best');

ax2 = subplot(4,1,2);
plot(time, com_update(:,2), '-', 'LineWidth',2.0); hold on; grid on
plot(time, com_servo(:,1),  '--','LineWidth',2.0);
ylabel('$c_y$ [m]','Interpreter','latex','FontSize',14);
xlim([40 155]); ylim([-0.01 0.03]);
legend('com\_update','com\_servo(q)','Interpreter','latex','Location','best');

ax3 = subplot(4,1,3);
plot(time, com_update(:,3), '-', 'LineWidth',2.0); hold on; grid on
plot(time, com_servo(:,3),  '--','LineWidth',2.0);
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$c_z$ [m]','Interpreter','latex','FontSize',14);
xlim([40 155]); ylim([-0.03 0.001]);
legend('com\_update','com\_servo(q)','Interpreter','latex','Location','best');

ax4 = subplot(4,1,4);
plot(time, desired_servo_angle(:,5), '-', 'LineWidth',2.0); hold on; grid on
plot(time, servo_angle(:,5),         '-', 'LineWidth',1.2);
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('$q_5$ [rad]','Interpreter','latex','FontSize',14);
xlim([15 63]); ylim([0 6]);
legend('q\_des','q\_meas','Interpreter','latex','Location','best');









%% ============================================================
%% True CoM (model) vs past_com + LS fit uncertainty (delta)
%% + regression time-window (t_fit_start ~ t_fit_end)
%% ============================================================
% ----- Given (from your slide) -----
twin=[20 180]
l = 0.6;                 % [m]
m_payload = 0.2;           % [kg]
m_body    = 4.2;           % [kg]
beta = m_payload/(m_body+m_payload);

p0_bar = [0.01418; 0; -0.16731];   % [m] nominal p0

% ----- Regression window (USER SET) -----
t_fit_start = 50;          % [s] 회귀 시작(비행 시작 이후로)
t_fit_end   = 150;         % [s] 회귀 끝(착륙 이전으로)

% (optional) 비행구간 자동 필터(PWM 기반) 쓰고 싶으면 true
use_pwm_flight_mask = false;
pwm_thresh = 0.08;         % pwm_scaled 평균이 이 값 이상이면 비행중으로 간주

% ----- Signals -----
q = servo_angle(:,5);      % [rad]
Y = past_com;              % Nx3 (estimated CoM)

% ----- Build nominal model: pc_nom(q) = beta * (p0_bar + l*f(q)) -----
f = [ (sqrt(3)/2)*sin(q), ...
      (1/4 + 3/4*cos(q)), ...
      -(sqrt(3)/4)*(1 - cos(q)) ];     % Nx3

p_nom  = (p0_bar.' + l*f);            % Nx3
pc_nom = beta * p_nom;                % Nx3 (nominal true_com)

% ----- Valid mask + time window (+ optional flight mask) -----
mask_valid = all(isfinite(Y),2) & isfinite(q) & (vecnorm(Y,2,2) > 1e-6);
mask_time  = (time >= t_fit_start) & (time <= t_fit_end);

mask = mask_valid & mask_time;

if use_pwm_flight_mask
    pwm_mean = mean(pwm_scaled, 2);           % requires pwm_scaled computed earlier
    mask_flight = pwm_mean > pwm_thresh;
    mask = mask & mask_flight;
end

fprintf('[FIT window] %.2f ~ %.2f sec, samples = %d\n', ...
    t_fit_start, t_fit_end, nnz(mask));

% ----- Linear regression for constant uncertainty term:
%       Y ≈ pc_nom + delta   (delta is 1x3 constant)
delta = mean(Y(mask,:) - pc_nom(mask,:), 1);     % 1x3

delta = [0 0 -0.015]
pc_fit = pc_nom + delta;                         % Nx3

% ----- Print results -----
fprintf('[beta] %.6f\n', beta);
fprintf('[FIT] delta = [%.6f %.6f %.6f] m\n', delta(1), delta(2), delta(3));

E_nom = Y(mask,:) - pc_nom(mask,:);
E_fit = Y(mask,:) - pc_fit(mask,:);
rmse_nom = sqrt(mean(E_nom.^2,1));
rmse_fit = sqrt(mean(E_fit.^2,1));
rmse_tot_nom = sqrt(mean(sum(E_nom.^2,2)));
rmse_tot_fit = sqrt(mean(sum(E_fit.^2,2)));

fprintf('[RMSE] nominal xyz = [%.4f %.4f %.4f] m, total = %.4f m\n', ...
    rmse_nom(1), rmse_nom(2), rmse_nom(3), rmse_tot_nom);
fprintf('[RMSE] fitted  xyz = [%.4f %.4f %.4f] m, total = %.4f m\n', ...
    rmse_fit(1), rmse_fit(2), rmse_fit(3), rmse_tot_fit);

% ----- Plot window (can be different from fit window if you want) -----
twin = [t_fit_start t_fit_end];

figure('Color','w','Position',[120 80 850 700]);

subplot(4,1,1);
plot(time, Y(:,1), '-', 'LineWidth',1.5); hold on; grid on;
plot(time, pc_nom(:,1), '--', 'LineWidth',2.0);
plot(time, pc_fit(:,1), ':',  'LineWidth',2.5);
ylabel('$c_x$ [m]','Interpreter','latex'); xlim(twin);
title('past\_com vs true\_com(model) + fitted uncertainty');
legend('past\_com','true\_com (nominal)','true\_com + \delta','Location','best');

subplot(4,1,2);
plot(time, Y(:,2), '-', 'LineWidth',1.5); hold on; grid on;
plot(time, pc_nom(:,2), '--', 'LineWidth',2.0);
plot(time, pc_fit(:,2), ':',  'LineWidth',2.5);
ylabel('$c_y$ [m]','Interpreter','latex'); xlim(twin);

subplot(4,1,3);
plot(time, Y(:,3), '-', 'LineWidth',1.5); hold on; grid on;
plot(time, pc_nom(:,3), '--', 'LineWidth',2.0);
plot(time, pc_fit(:,3), ':',  'LineWidth',2.5);
ylabel('$c_z$ [m]','Interpreter','latex'); xlim(twin);

subplot(4,1,4);
plot(time, q, '-', 'LineWidth',1.5); grid on;
ylabel('$q_5$ [rad]','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
xlim(twin);










%% ===================== True CoM (model) for overlay on CoM panel =====================
% ----- Given -----
l = 0.6;                 % [m]
m_payload = 0.2;         % [kg]
m_body    = 4.2;         % [kg]
beta = m_payload/(m_body+m_payload);

p0_bar = [0.01418; 0; -0.16731];   % [m]

% ----- Regression window -----
t_fit_start = 50;     % [s]
t_fit_end   = 150;    % [s]

use_pwm_flight_mask = false;
pwm_thresh = 0.08;

% ----- Signals -----
q = servo_angle(:,5);      % [rad]
Y = past_com;              % Nx3

% ----- Nominal model -----
f_nom = [ (sqrt(3)/2)*sin(q), ...
          (1/4 + 3/4*cos(q)), ...
          -(sqrt(3)/4)*(1 - cos(q)) ];      % Nx3

p_nom  = (p0_bar.' + l*f_nom);    % Nx3
pc_nom = beta * p_nom;            % Nx3

% ----- Mask -----
mask_valid = all(isfinite(Y),2) & isfinite(q) & (vecnorm(Y,2,2) > 1e-6);
mask_time  = (time >= t_fit_start) & (time <= t_fit_end);
mask = mask_valid & mask_time;

if use_pwm_flight_mask
    pwm_mean = mean(pwm_scaled, 2);
    mask = mask & (pwm_mean > pwm_thresh);
end

fprintf('[FIT window] %.2f ~ %.2f sec, samples = %d\n', t_fit_start, t_fit_end, nnz(mask));

% ----- Fit delta -----
delta = mean(Y(mask,:) - pc_nom(mask,:), 1);   % 1x3
delta = [0 0 -0.015];                          % (네가 준 overwrite)
pc_fit = pc_nom + delta;                        % Nx3

true_com = pc_fit;  % <<<<<< CoM panel overlay로 쓸 true com



















%% ============================================================
% 5. 대시보드 (3x2 blocks)  [panel+manual layout style]
%  - true_com overlay in CoM panel
%  - make ALL subplot plot-box widths identical (tiledlayout-safe)
% ============================================================

twin = [35 220];







% ============================================================
% past_com 미분 (d/dt)  -> CoM 패널에 오버레이용
% ============================================================
dt = [diff(time); median(diff(time))];   % Nx1
dt(dt <= 0) = median(dt(dt > 0));        % 안전장치

d_past_com = zeros(size(past_com));      % Nx3
d_past_com(2:end,:) = diff(past_com) ./ dt(1:end-1);

% (선택) 보기 좋게 smoothing
use_dcom_lpf = true;
if use_dcom_lpf
    win = 50; % samples (예: 0.5s @100Hz)
    d_past_com = movmean(d_past_com, win, 1);
end




% ============================================================
% past_com 미분 (d/dt)  -> (2,2) 패널에 우측축으로 오버레이
% ============================================================
dt_vec = [diff(time); median(diff(time))];   % Nx1 (마지막은 대충 유지)
dt_vec(dt_vec <= 0) = median(dt_vec(dt_vec > 0)); % 안전장치

d_past_com = [diff(past_com)./dt_vec(1:end-1), zeros(size(past_com,1)-1,2)]; % <-- (임시) 잘못된 형태 방지용
% 위 줄 대신 아래 "정상" 코드 사용:
d_past_com = zeros(size(past_com));
d_past_com(1,:) = 0;
d_past_com(2:end,:) = diff(past_com) ./ dt_vec(1:end-1);

% (선택) 보기 좋게 살짝 smoothing
use_dcom_lpf = false;
if use_dcom_lpf
    win = 50; % samples
    d_past_com = movmean(d_past_com, win, 1);
end




f = figure('Name', 'DataLogging Overview (Pos/Att/dhat/CoM/PWM/Servo)', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0 0 1 1]);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 3; nrow = 2;

w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;

getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% ============================================================
% TRUE CoM model (pc_fit) to overlay on CoM panel
% ============================================================
l = 0.6;                   % [m]
m_payload = 0.3;           % [kg]
m_body    = 4.2;           % [kg]
beta = m_payload/(m_body+m_payload);

p0_bar = [0.01418; 0; -0.16731];   % [m]

t_fit_start = 50;          % [s]
t_fit_end   = 150;         % [s]

use_pwm_flight_mask = false;
pwm_thresh = 0.08;

q = servo_angle(:,5);      % [rad]
Y = past_com;              % Nx3

f_nom = [ (sqrt(3)/2)*sin(q), ...
          (1/4 + 3/4*cos(q)), ...
          -(sqrt(3)/4)*(1 - cos(q)) ];     % Nx3

p_nom  = (p0_bar.' + l*f_nom);             % Nx3
pc_nom = beta * p_nom;                     % Nx3

mask_valid = all(isfinite(Y),2) & isfinite(q) & (vecnorm(Y,2,2) > 1e-6);
mask_time  = (time >= t_fit_start) & (time <= t_fit_end);
mask = mask_valid & mask_time;

if use_pwm_flight_mask
    pwm_mean = mean(pwm_scaled, 2);
    mask = mask & (pwm_mean > pwm_thresh);
end

fprintf('[FIT window] %.2f ~ %.2f sec, samples = %d\n', ...
    t_fit_start, t_fit_end, nnz(mask));

delta = mean(Y(mask,:) - pc_nom(mask,:), 1);     % computed
delta = [0 0 -0.];                            % << overwrite as requested

true_com = pc_nom + delta;                       % Nx3

% ============================================================
% (1,1) Position XYZ (meas vs des)
% ============================================================
p11  = uipanel('Parent', f, 'Position', getPos(1,1), 'BackgroundColor', 'w');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl11, 1);
plot(time, desired_position(:,1) , '--', 'LineWidth', 1.8); hold on;
plot(time, position(:,1), '-', 'LineWidth', 1.8);
grid on; xlim(twin);
ylim([-0.6 0.6])
title('Position')

nexttile(tl11, 2);
plot(time, desired_position(:,2) , '--', 'LineWidth', 1.8); hold on;
plot(time, position(:,2), '-', 'LineWidth', 1.8);
grid on; xlim(twin);
ylim([-0.6 0.6]);

nexttile(tl11, 3);
plot(time, desired_position(:,3) , '--', 'LineWidth', 1.8); hold on;
plot(time, position(:,3), '-', 'LineWidth', 1.8);
grid on; xlim(twin);
ylim([-1.1 -0.4]);
xlabel('time [s]');


% ============================================================
% (2,1) Attitude Error (RPY + Norm + Moving RMS)  [yaw unwrap]
% ============================================================
p21  = uipanel('Parent', f, 'Position', getPos(2,1), 'BackgroundColor', 'w');
tl21 = tiledlayout(p21, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl21, 1);
err_roll = attitude(:,1) - desired_attitude(:,1);
plot(time, zeros(size(time)), '--', 'LineWidth', 1.8); hold on;
plot(time, err_roll, '-', 'LineWidth', 1.8);
grid on; xlim(twin);
ylim([-0.1 0.1])
title('Attitude Error');

nexttile(tl21, 2);
err_pitch = attitude(:,2) - desired_attitude(:,2);
plot(time, zeros(size(time)), '--', 'LineWidth', 1.8); hold on;
plot(time, err_pitch, '-', 'LineWidth', 1.8);
grid on; xlim(twin);
ylim([-0.1 0.1])

nexttile(tl21, 3);
yaw_meas_u = unwrap(attitude(:,3));
yaw_des_u  = unwrap(desired_attitude(:,3));
err_yaw = yaw_meas_u - yaw_des_u;
plot(time, zeros(size(time)), '--', 'LineWidth', 1.8); hold on;
plot(time, err_yaw, '-', 'LineWidth', 1.8);
grid on; xlim(twin);
ylim([-0.1 0.1])

nexttile(tl21, 4);
att_err_mat  = [err_roll, err_pitch, err_yaw];
att_err_norm = sqrt(sum(att_err_mat.^2, 2));
window_size  = 200;
att_err_rms  = sqrt(movmean(att_err_norm.^2, window_size));

plot(time, att_err_norm, 'k', 'LineWidth', 1.2); hold on;
plot(time, att_err_rms,  'r', 'LineWidth', 2);

grid on; xlim(twin); ylim([0 0.07])
ylabel('||Q||');
xlabel('time [s]');

% ============================================================
% (1,2) dhat XYZ
% ============================================================
p12  = uipanel('Parent', f, 'Position', getPos(1,2), 'BackgroundColor', 'w');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl12, 1);
plot(time, torque_dhat(:,1), 'LineWidth', 1.8);
grid on; xlim(twin);
ylim([-2. 2.])
title('Disturbance Observer Torque');

nexttile(tl12, 2);
plot(time, torque_dhat(:,2), 'LineWidth', 1.8);
grid on; xlim(twin);
ylim([-2. 2.])

nexttile(tl12, 3);
plot(time, torque_dhat(:,3), 'LineWidth', 1.8);
grid on; xlim(twin);
ylim([-2. 2.])
xlabel('time [s]');

% ============================================================
% (2,2) CoM XYZ (past_com + true_com overlay)
% ============================================================
p22  = uipanel('Parent', f, 'Position', getPos(2,2), 'BackgroundColor', 'w');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl22, 1);
plot(time, past_com(:,1) - 0.003, '-',  'LineWidth', 1.8); hold on;
plot(time, -true_com(:,1), '--', 'LineWidth', 2.2);
grid on; xlim(twin);
ylim([-0.05 0.01])
title('CoM Estimate');

% legend는 axes 폭을 갉아먹을 수 있으니 밖으로 빼서 고정

nexttile(tl22, 2);
plot(time, past_com(:,2), '-',  'LineWidth', 1.8); hold on;
plot(time, -true_com(:,2), '--', 'LineWidth', 2.2);
grid on; xlim(twin);
ylim([-0.05 0.03])

nexttile(tl22, 3);
plot(time, past_com(:,3), '-',  'LineWidth', 1.8); hold on;
plot(time, true_com(:,3), '--', 'LineWidth', 2.2);
grid on; xlim(twin);
ylim([-0.06 0.02])
xlabel('time [s]');

% ============================================================
% (1,3) PWM (scaled) 4x1
% ============================================================
p13  = uipanel('Parent', f, 'Position', getPos(1,3), 'BackgroundColor', 'w');
tl13 = tiledlayout(p13, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl13, 1);
plot(time, pwm_scaled(:,1), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([0 1]);
title('PWM');

nexttile(tl13, 2);
plot(time, pwm_scaled(:,2), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([0 1]);

nexttile(tl13, 3);
plot(time, pwm_scaled(:,3), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([0 1]);

nexttile(tl13, 4);
plot(time, pwm_scaled(:,4), '-', 'LineWidth', 1.8);
grid on; xlim(twin); ylim([0 1]);
xlabel('time [s]');

% ============================================================
% (2,3) Servo angles 5x1 (des vs meas)
% ============================================================
p23  = uipanel('Parent', f, 'Position', getPos(2,3), 'BackgroundColor', 'w');
tl23 = tiledlayout(p23, 5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:5
    nexttile(tl23, i);
    plot(time, desired_servo_angle(:,i), '--', 'LineWidth', 1.8); hold on;
    plot(time, servo_angle(:,i), '-',  'LineWidth', 1.8);
    grid on; xlim(twin); ylim([-0.7 0.7])
    if i == 1
        title('Servo')
    end
    if i == 5
        xlabel('time [s]'); ylim([0 3.2])
    end
end

% ============================================================
% 공통 스타일
% ============================================================
ax = findall(f, 'Type', 'axes');
set(ax, 'FontSize', 9, 'Color', 'w');

% ============================================================
% === 핵심: tiledlayout-safe 가로폭 동일화 (plot-box width equal) ===
% - Position을 건드리지 않음 (경고 안 뜸)
% - ytick label / ylabel 때문에 달라지는 좌측 여백을 "예약"해서 동일화
% ============================================================
drawnow;  % 레이아웃 확정 후 적용

ax = findall(f, 'Type', 'axes');
ax = ax(isvalid(ax));

% 1) y축 exponent(×10^n)로 폭 튀는 것 방지
for k = 1:numel(ax)
    ax(k).YAxis.Exponent = 0;
end

% 2) ytick 라벨 자릿수 통일 (여기 포맷은 필요에 따라 바꿔도 됨)
%    핵심은 "모든 axes가 동일 포맷"을 쓰게 하는 것
for k = 1:numel(ax)
    try
        ytickformat(ax(k), '%.2f');  % <- 소수점 2자리 고정 (가로폭 안정)
    catch
        % 구버전 MATLAB 호환
    end
end

% 3) ylabel 폭을 전체 통일: 모든 axes에 동일한 ylabel을 '예약'
%    (필요한 축만 보이게 하고, 나머지는 투명 처리)
reserveLabel = '||Q||';   % 가장 넓게 잡고 싶은 문자열로 통일 (원하면 더 길게 가능)

for k = 1:numel(ax)
    % 이미 ylabel이 있든 없든, 동일 문자열로 폭 예약
    yl = ylabel(ax(k), reserveLabel);
    yl.Units = 'normalized';
    yl.Visible = 'on';
    yl.Color = [0 0 0 0];   % 투명(보이지 않게) -> 공간은 유지됨
end

% 4) 실제로 보이고 싶은 ylabel만 다시 보이게 (Attitude norm subplot 하나)
%    (네 코드에서 Attitude norm subplot은 p21의 4번째 tile)
%    타겟 찾기: yLabel이 이미 '||Q||'로 설정된 축 중, ylim이 [0 0.06]인 축을 우선으로 잡음
targetAx = [];
for k = 1:numel(ax)
    ylstr = string(ax(k).YLabel.String);
    if ylstr == reserveLabel
        ylm = ax(k).YLim;
        if numel(ylm)==2 && abs(ylm(1)-0)<1e-9 && abs(ylm(2)-0.06)<1e-3
            targetAx = ax(k);
            break;
        end
    end
end
if ~isempty(targetAx) && isvalid(targetAx)
    targetAx.YLabel.Color = [0 0 0 1];  % 이 축만 ylabel 보이게
end

drawnow;
















%% ===== 4-in-1 Dashboard (2x2 panels) : Att / Pos / tau_comp LPF / tau_comp RAW =====
twin = [38 128];   % <<<<<< 여기만 바꾸면 xlim 전체 통일

f = figure('Name', 'Overview (Att / Pos / \tau_{comp,LPF} / \tau_{comp,raw})', ...
    'NumberTitle', 'off', 'Color', 'w', 'Units', 'normalized', ...
    'Position', [0.05 0.05 0.9 0.85]);

% manual layout params (same vibe)
left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.04; vgap = 0.06;
ncol = 2; nrow = 2;

w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;

getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% =========================================================
% (1,1) Attitude RPY (meas vs des)
% =========================================================
p11  = uipanel('Parent', f, 'Position', getPos(1,1), 'BackgroundColor', 'w');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

axA1 = nexttile(tl11, 1);
plot(time, desired_attitude(:,1), '-', 'LineWidth', 2.0); hold on;
plot(time, attitude(:,1), '-', 'LineWidth', 1.7);
grid on; xlim(twin);
ylabel('$\phi$ [rad]','Interpreter','latex','FontSize',12);
title('Attitude (des vs meas)','Interpreter','latex')
legend({'des','meas'},'Interpreter','latex','Orientation','horizontal','Location','best')

axA2 = nexttile(tl11, 2);
plot(time, desired_attitude(:,2), '-', 'LineWidth', 2.0); hold on;
plot(time, attitude(:,2), '-', 'LineWidth', 1.7);
grid on; xlim(twin);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',12);
legend({'des','meas'},'Interpreter','latex','Orientation','horizontal','Location','best')

axA3 = nexttile(tl11, 3);
plot(time, desired_attitude(:,3), '-', 'LineWidth', 2.0); hold on;   % yaw des
plot(time, attitude(:,3), '-', 'LineWidth', 1.7);
grid on; xlim(twin);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$\psi$ [rad]','Interpreter','latex','FontSize',12);
legend({'des','meas'},'Interpreter','latex','Orientation','horizontal','Location','best')

linkaxes([axA1 axA2],'y');
%ylim(axA1,[-0.3 0.3]);
%ylim(axA3,[-0.3 0.3]);

% =========================================================
% (1,2) XYZ Position (meas vs des)
% =========================================================
p12  = uipanel('Parent', f, 'Position', getPos(1,2), 'BackgroundColor', 'w');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl12, 1);
plot(time, desired_position(:,1), '-', 'LineWidth', 2.0); hold on;
plot(time, position(:,1), '-', 'LineWidth', 1.7);
grid on; xlim(twin); ylim([-0.3 0.3]);
ylabel('$x$ [m]','Interpreter','latex','FontSize',12);
title('Position (des vs meas)','Interpreter','latex')
legend({'des','meas'},'Interpreter','latex','Orientation','horizontal','Location','best')

nexttile(tl12, 2);
plot(time, desired_position(:,2), '-', 'LineWidth', 2.0); hold on;
plot(time, position(:,2), '-', 'LineWidth', 1.7);
grid on; xlim(twin); ylim([-0.3 0.3]);
ylabel('$y$ [m]','Interpreter','latex','FontSize',12);
legend({'des','meas'},'Interpreter','latex','Orientation','horizontal','Location','best')

nexttile(tl12, 3);
plot(time, desired_position(:,3), '-', 'LineWidth', 2.0); hold on;
plot(time, position(:,3), '-', 'LineWidth', 1.7);
grid on; xlim(twin); ylim([-0.6 -0.2]);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$z$ [m]','Interpreter','latex','FontSize',12);
legend({'des','meas'},'Interpreter','latex','Orientation','horizontal','Location','best')

% =========================================================
% (2,1) tau_comp LPF (3x1) + Servo #5  → (4x1)
% =========================================================
p21  = uipanel('Parent', f, 'Position', getPos(2,1), 'BackgroundColor', 'w');
tl21 = tiledlayout(p21, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- tau_x
nexttile(tl21, 1);
plot(time, l1_tau_comp_lpf(:,1), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-0.5 1.7])
ylabel('$\tau_x$ [Nm]','Interpreter','latex','FontSize',12);
title('$\tau_{comp}$ (LPF output)','Interpreter','latex')
legend({'LPF'},'Interpreter','latex','Orientation','horizontal','Location','best')

% --- tau_y
nexttile(tl21, 2);
plot(time, l1_tau_comp_lpf(:,2), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-0.5 1.7])
ylabel('$\tau_y$ [Nm]','Interpreter','latex','FontSize',12);
legend({'LPF'},'Interpreter','latex','Orientation','horizontal','Location','best')

% --- tau_z
nexttile(tl21, 3);
plot(time, l1_tau_comp_lpf(:,3), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-0.05 0.05])
ylabel('$\tau_z$ [Nm]','Interpreter','latex','FontSize',12);
legend({'LPF'},'Interpreter','latex','Orientation','horizontal','Location','best')

% --- Servo angle #5 (measured only)
nexttile(tl21, 4);
plot(time, servo_angle(:,5), '-', 'LineWidth', 1.8);
grid on; xlim(twin);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$\theta_5$ [rad]','Interpreter','latex','FontSize',12);
legend({'servo 5'},'Interpreter','latex','Orientation','horizontal','Location','best')

% =========================================================
% (2,2) tau_comp RAW (3x1)
% =========================================================
p22  = uipanel('Parent', f, 'Position', getPos(2,2), 'BackgroundColor', 'w');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl22, 1);
plot(time, l1_tau_comp_raw(:,1), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-300 300])
ylabel('$\tau_x$ [Nm]','Interpreter','latex','FontSize',12);
title('$\tau_{comp}$ (raw)','Interpreter','latex')
legend({'raw'},'Interpreter','latex','Orientation','horizontal','Location','best')

nexttile(tl22, 2);
plot(time, l1_tau_comp_raw(:,2), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-300 300])
ylabel('$\tau_y$ [Nm]','Interpreter','latex','FontSize',12);
legend({'raw'},'Interpreter','latex','Orientation','horizontal','Location','best')

nexttile(tl22, 3);
plot(time, l1_tau_comp_raw(:,3), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-300 300])
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$\tau_z$ [Nm]','Interpreter','latex','FontSize',12);
legend({'raw'},'Interpreter','latex','Orientation','horizontal','Location','best')

% common style
set(findall(f, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');















%% 5. 대시보드 (3x2 blocks)  [panel+manual layout style]
twin = [45 127];

f = figure('Name', 'DataLogging Overview (Pos/Att/tau_raw/tau_lpf/PWM/Servo)', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0 0 1 1]);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 3; nrow = 2;

w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;

getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% =========================================================
% (1,1) Position XYZ (meas vs des)
% =========================================================
p11  = uipanel('Parent', f, 'Position', getPos(1,1), 'BackgroundColor', 'w');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl11, 1);
plot(time, desired_position(:,1) , '--', 'LineWidth', 1.2); hold on;
plot(time, position(:,1), '-', 'LineWidth', 1.2);
grid on; xlim(twin);
ylim([-0.6 0.6])
title('Position')

nexttile(tl11, 2);
plot(time, desired_position(:,2) , '--', 'LineWidth', 1.2); hold on;
plot(time, position(:,2), '-', 'LineWidth', 1.2);
grid on; xlim(twin);
ylim([-0.6 0.6]);

nexttile(tl11, 3);
plot(time, desired_position(:,3) , '--', 'LineWidth', 1.2); hold on;
plot(time, position(:,3), '-', 'LineWidth', 1.2);
grid on; xlim(twin);
ylim([-1.1 -0.4]);
xlabel('time [s]');

% =========================================================
% (2,1) Attitude Error (RPY + Norm + Moving RMS)  [yaw unwrap]
% =========================================================
p21  = uipanel('Parent', f, 'Position', getPos(2,1), 'BackgroundColor', 'w');
tl21 = tiledlayout(p21, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% Roll error
nexttile(tl21, 1);
err_roll = attitude(:,1) - desired_attitude(:,1);
plot(time, zeros(size(time)), '--', 'LineWidth', 1.2); hold on;
plot(time, err_roll, '-', 'LineWidth', 1.2);
grid on; xlim(twin);
ylim([-0.1 0.1])
title('Attitude Error');

% Pitch error
nexttile(tl21, 2);
err_pitch = attitude(:,2) - desired_attitude(:,2);
plot(time, zeros(size(time)), '--', 'LineWidth', 1.2); hold on;
plot(time, err_pitch, '-', 'LineWidth', 1.2);
grid on; xlim(twin);
ylim([-0.1 0.1])

% Yaw error (unwrap)
nexttile(tl21, 3);
yaw_meas_u = unwrap(attitude(:,3));
yaw_des_u  = unwrap(desired_attitude(:,3));
err_yaw = yaw_meas_u - yaw_des_u;
plot(time, zeros(size(time)), '--', 'LineWidth', 1.2); hold on;
plot(time, err_yaw, '-', 'LineWidth', 1.2);
grid on; xlim(twin);
ylim([-0.1 0.1])

% Attitude Error Norm + Moving RMS
nexttile(tl21, 4);
att_err_mat  = [err_roll, err_pitch, err_yaw];
att_err_norm = sqrt(sum(att_err_mat.^2, 2));

window_size = 200;   % samples (예: 100Hz면 약 2초)
att_err_rms = sqrt(movmean(att_err_norm.^2, window_size));

plot(time, att_err_norm, 'LineWidth', 4.0); hold on;
plot(time, att_err_rms,  'r', 'LineWidth', 2);

grid on; xlim(twin); ylim([0 0.1])
ylabel('||Q||');
xlabel('time [s]');

% =========================================================
% (1,2) tau_comp RAW (3x1)
% =========================================================
p12  = uipanel('Parent', f, 'Position', getPos(1,2), 'BackgroundColor', 'w');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl12, 1);
plot(time, l1_tau_comp_raw(:,1), '-', 'LineWidth', 1.8); hold on;
plot(time, l1_tau_comp_lpf(:,1), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-300 300])
ylabel('$\tau_x$ [Nm]','Interpreter','latex','FontSize',12);
title('$\tau_{comp}$ (raw)','Interpreter','latex')
legend({'raw'},'Interpreter','latex','Orientation','horizontal','Location','best')
ylim([-6.0 2.0])

nexttile(tl12, 2);
plot(time, l1_tau_comp_raw(:,2), '-', 'LineWidth', 1.8); hold on;
plot(time, l1_tau_comp_lpf(:,2), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-300 300])
ylabel('$\tau_y$ [Nm]','Interpreter','latex','FontSize',12);
legend({'raw'},'Interpreter','latex','Orientation','horizontal','Location','best')
ylim([-6.0 2.0])

nexttile(tl12, 3);
plot(time, l1_tau_comp_raw(:,3), '-', 'LineWidth', 1.8); hold on;
plot(time, l1_tau_comp_lpf(:,3), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-300 300])
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$\tau_z$ [Nm]','Interpreter','latex','FontSize',12);
legend({'raw'},'Interpreter','latex','Orientation','horizontal','Location','best')
ylim([-2.5 2.5])

% =========================================================
% (2,2) tau_comp LPF (3x1) + Servo #5  → (4x1)
% =========================================================
p22  = uipanel('Parent', f, 'Position', getPos(2,2), 'BackgroundColor', 'w');
tl22 = tiledlayout(p22, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% tau_x
nexttile(tl22, 1);
plot(time, l1_tau_comp_lpf(:,1), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-0.5 1.7])
ylabel('$\tau_x$ [Nm]','Interpreter','latex','FontSize',12);
title('$\tau_{comp}$ (LPF output)','Interpreter','latex')
legend({'LPF'},'Interpreter','latex','Orientation','horizontal','Location','best')

% tau_y
nexttile(tl22, 2);
plot(time, l1_tau_comp_lpf(:,2), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-0.5 1.7])
ylabel('$\tau_y$ [Nm]','Interpreter','latex','FontSize',12);
legend({'LPF'},'Interpreter','latex','Orientation','horizontal','Location','best')

% tau_z
nexttile(tl22, 3);
plot(time, l1_tau_comp_lpf(:,3), '-', 'LineWidth', 1.8);
grid on; xlim(twin); %ylim([-0.05 0.05])
ylabel('$\tau_z$ [Nm]','Interpreter','latex','FontSize',12);
legend({'LPF'},'Interpreter','latex','Orientation','horizontal','Location','best')

% Servo angle #5 (measured only)
nexttile(tl22, 4);
plot(time, servo_angle(:,5), '-', 'LineWidth', 1.8);
grid on; xlim(twin);
xlabel('Time [s]','Interpreter','latex','FontSize',12);
ylabel('$\theta_5$ [rad]','Interpreter','latex','FontSize',12);
legend({'servo 5'},'Interpreter','latex','Orientation','horizontal','Location','best')

% =========================================================
% (1,3) PWM (scaled) 4x1
% =========================================================
p13  = uipanel('Parent', f, 'Position', getPos(1,3), 'BackgroundColor', 'w');
tl13 = tiledlayout(p13, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl13, 1);
plot(time, pwm_scaled(:,1), '-', 'LineWidth', 0.8);
grid on; xlim(twin); ylim([0 1]);
title('PWM');

nexttile(tl13, 2);
plot(time, pwm_scaled(:,2), '-', 'LineWidth', 0.8);
grid on; xlim(twin); ylim([0 1]);

nexttile(tl13, 3);
plot(time, pwm_scaled(:,3), '-', 'LineWidth', 0.8);
grid on; xlim(twin); ylim([0 1]);

nexttile(tl13, 4);
plot(time, pwm_scaled(:,4), '-', 'LineWidth', 0.8);
grid on; xlim(twin); ylim([0 1]);
xlabel('time [s]');

% =========================================================
% (2,3) Servo angles 5x1 (des vs meas)
% =========================================================
p23  = uipanel('Parent', f, 'Position', getPos(2,3), 'BackgroundColor', 'w');
tl23 = tiledlayout(p23, 5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:5
    nexttile(tl23, i);
    plot(time, desired_servo_angle(:,i), '--', 'LineWidth', 0.8); hold on;
    plot(time, servo_angle(:,i), '-',  'LineWidth', 0.8);
    grid on; xlim(twin);

    if i == 1
        title('Servo')
    end

    if i < 5
        ylim([-0.7 0.7])
    else
        ylim([0 3.2])
        xlabel('time [s]');
    end
end

% =========================================================
% 공통 스타일
% =========================================================
set(findall(f, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');



%% Servo (desired vs measured) + Thruster (auto-scaled) : 4x2
figure('Color','w', 'Position', [100 100 1350 900]);   % [left bottom width height]

mass = 4.02;         % [kg]
g = 9.81;            % [m/s^2]
hover_window = [3 8];

idx_hover = (time >= hover_window(1)) & (time <= hover_window(2));

if ~any(idx_hover)
    error('No samples found in the hover window %.1f to %.1f s.', ...
        hover_window(1), hover_window(2));
end

hover_thrust_target = mass * g;   % total hover thrust [N]
hover_thrust_mean_raw = mean(sum(individual_motor_thrust(idx_hover,1:4), 2));
thrust_scale = hover_thrust_target / hover_thrust_mean_raw;

thruster_scaled = individual_motor_thrust(:,1:4) * thrust_scale;

fprintf('[INFO] Hover window        : %.1f ~ %.1f s\n', hover_window(1), hover_window(2));
fprintf('[INFO] Target hover thrust : %.4f N\n', hover_thrust_target);
fprintf('[INFO] Raw mean total thrust in window : %.4f N\n', hover_thrust_mean_raw);
fprintf('[INFO] Applied thrust scale factor     : %.6f\n', thrust_scale);

for i = 1:4
    % Left: Servo (desired vs measured)
    subplot(4,2,2*i-1);
    plot(time, servo_angle(:,i), '-', 'LineWidth', 1.5); hold on;
    plot(time, desired_servo_angle(:,i), '--', 'LineWidth', 1.5);
    grid on;
    title(sprintf('Servo %d', i));
    ylabel('[rad]');
    ylim([-0.6 0.6]);
    xlim([0 80]);

    if i == 1
        legend('Measured', 'Desired', 'Location', 'best');
    end

    % Right: Thruster (auto-scaled)
    subplot(4,2,2*i);
    plot(time, thruster_scaled(:,i), '-', 'LineWidth', 1.5);
    grid on;
    title(sprintf('Thruster %d', i));
    ylabel('[N]');
    ylim([0 22]);
    xlim([0 80]);
end


%% ============================================================
% Limit utilization summary for paper sentence
% ============================================================

% ----- Table 1 limits (edit these to your exact values) -----
thruster_limit_N   = 22;    % [N] per individual thruster
servo_angle_limit  = 0.6;   % [rad]
servo_rate_limit   = 6.0;   % [rad/s]  <-- change to your Table 1 value

% ----- Use hover-matched thrust scaling from earlier block -----
% thruster_scaled = individual_motor_thrust(:,1:4) * thrust_scale;

% ----- Servo #5 angular rate from measured angle -----
servo5 = servo_angle(:,5);

dt = [diff(time); median(diff(time))];
dt(dt <= 0) = median(dt(dt > 0));

servo5_rate = zeros(size(servo5));
servo5_rate(2:end) = diff(servo5) ./ dt(1:end-1);

% optional smoothing for cleaner reported peak
use_rate_smoothing = true;
if use_rate_smoothing
    servo5_rate = movmean(servo5_rate, 5);
end

% ----- Utilizations -----
max_individual_thrust_N = max(thruster_scaled(:,1:4), [], 'all');
max_thrust_util_pct = 100 * max_individual_thrust_N / thruster_limit_N;

max_servo_angle_rad = max(abs(servo5));
max_servo_angle_util_pct = 100 * max_servo_angle_rad / servo_angle_limit;

max_servo_rate_rads = max(abs(servo5_rate));
max_servo_rate_util_pct = 100 * max_servo_rate_rads / servo_rate_limit;

% ----- Print to MATLAB terminal -----
fprintf('\n========== LIMIT UTILIZATION SUMMARY ==========\n');
fprintf('Max individual thrust command      : %.4f N\n', max_individual_thrust_N);
fprintf('Max thrust utilization             : %.2f %%  (limit = %.2f N)\n', ...
    max_thrust_util_pct, thruster_limit_N);

fprintf('Peak tilting-servo angle           : %.4f rad\n', max_servo_angle_rad);
fprintf('Peak servo-angle utilization       : %.2f %%  (limit = %.4f rad)\n', ...
    max_servo_angle_util_pct, servo_angle_limit);

fprintf('Peak tilting-servo angular rate    : %.4f rad/s\n', max_servo_rate_rads);
fprintf('Peak servo-rate utilization        : %.2f %%  (limit = %.4f rad/s)\n', ...
    max_servo_rate_util_pct, servo_rate_limit);
fprintf('===============================================\n\n');

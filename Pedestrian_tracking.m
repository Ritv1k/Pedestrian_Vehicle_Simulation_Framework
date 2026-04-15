clc; clear;

%% -----------------------------
% Load X/Y trajectory from Excel
% -----------------------------
data = readtable("pedestrian_xy.xlsx");

t = data.Time_s;    % time [s]
x = data.X_m;       % X position [m]
y = data.Y_m;       % Y position [m]

%% -----------------------------
% Force start at origin (safety)
% -----------------------------
x = x - x(1);
y = y - y(1);

%% -----------------------------
% Compute yaw from motion direction
% -----------------------------
dx = [diff(x); x(end) - x(end-1)];
dy = [diff(y); y(end) - y(end-1)];

yawRad = atan2(dy, dx);
yawDeg = rad2deg(yawRad);

%% -----------------------------
% Create timeseries for Simulink
% -----------------------------
pedX   = timeseries(x, t);       % meters
pedY   = timeseries(y, t);       % meters
pedYaw = timeseries(yawDeg, t);  % degrees

disp("✅ Pedestrian X/Y/Yaw time series ready");

%% -----------------------------
% Plot sanity check
% -----------------------------
figure;
plot(x, y, '-o');
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
title('Pedestrian Path Loaded from Excel');

clc; clear;

%% -----------------------------
% Trajectory parameters
% -----------------------------
fs = 10;          % Hz
dt = 1/fs;
v  = 1.2;         % walking speed (m/s)

t1 = 5;           % straight segment (s)
t2 = 5;           % after turn (s)

%% -----------------------------
% Time vectors
% -----------------------------
t_straight = 0:dt:t1;
t_turn     = dt:dt:t2;

t = [t_straight, t_straight(end) + t_turn];

%% -----------------------------
% Straight motion (along Y)
% -----------------------------
y1 = v * t_straight;
x1 = zeros(size(y1));

%% -----------------------------
% 90-degree turn (walk along X)
% -----------------------------
y2 = y1(end) * ones(size(t_turn));
x2 = v * t_turn;

%% -----------------------------
% Combine trajectory
% -----------------------------
x = [x1, x2];
y = [y1, y2];

%% -----------------------------
% Create table and save to Excel
% -----------------------------
T = table(t(:), x(:), y(:), ...
    'VariableNames', {'Time_s','X_m','Y_m'});

writetable(T, 'pedestrian_xy.xlsx');

disp("✅ pedestrian_xy.xlsx generated");

%% -----------------------------
% Plot sanity check
% -----------------------------
figure;
plot(x, y, '-o');
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
title('Generated Pedestrian X/Y Path');

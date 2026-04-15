% --- MASTER SCRIPT: Read CSV, Convert Frames & Package for Simulink ---
clear; clc; close all;

% --- 1. SETTINGS ---
filename = 'test_record_data.csv';
needs_decoding = false; 

% The new starting coordinates for Unreal Engine
start_X_offset = 50.0;
start_Y_offset = 50.0;

% --- 2. READ THE DATA ---
disp('Loading CSV data...');
data = readtable(filename);

raw_x = data.Xr;
raw_y = data.Yr;
raw_yaw = data.Psi_r;
time = data.time;

% --- 3. APPLY PhD FIXED-POINT DECODING (If Needed) ---
if needs_decoding
    disp('Applying fixed-point decoding...');
    X_Fr = (raw_x / 100.0) - 10000.0;
    Y_Fr = (raw_y / 100.0) - 10000.0;
    Psi_Fr = (raw_yaw / 100.0) - 180.0;
else
    disp('Skipping decoding. Using CSV values directly...');
    X_Fr = raw_x;
    Y_Fr = raw_y;
    Psi_Fr = raw_yaw;
end

% --- 4. UNREAL ENGINE FRAME CONVERSION ---
disp('Translating origin and rotating to +X Forward...');

% Set the very first point as the reference origin
origin_X = X_Fr(1);
origin_Y = Y_Fr(1);
origin_Yaw = deg2rad(Psi_Fr(1));

N = length(X_Fr);
carx = zeros(N, 1);
cary = zeros(N, 1);
caryaw  = zeros(N, 1);

for i = 1:N
    % Step A: Find distance from the very first point
    dx = X_Fr(i) - origin_X;
    dy = Y_Fr(i) - origin_Y;
    
    % Step B: Rotate to +X Forward, AND add the 50m offset
    carx(i) =  (dx * cos(origin_Yaw) + dy * sin(origin_Yaw)) + start_X_offset;
    cary(i) = (-dx * sin(origin_Yaw) + dy * cos(origin_Yaw)) + start_Y_offset;
    
    % Step C: Calculate relative Yaw (Counter-clockwise)
    caryaw(i) = Psi_Fr(i) - rad2deg(origin_Yaw);
end

% --- 5. PACKAGE FOR SIMULINK (STRICT MATRIX FORMAT) ---
% 1. Force everything to be a strict column vector
time_col = time(:);
carx_col = carx(:);
cary_col = cary(:);
caryaw_col = caryaw(:);

% 2. Shift time so it starts perfectly at 0
time_shifted = time_col - time_col(1); 

% 3. THE FIX: Find and delete any NaNs or Infs caused by blank CSV rows
valid_idx = isfinite(time_shifted) & isfinite(carx_col) & isfinite(cary_col) & isfinite(caryaw_col);

% 4. Build the final 2D double-precision matrices for Simulink
Simulink_Car_X   = double([time_shifted(valid_idx), carx_col(valid_idx)]);
Simulink_Car_Y   = double([time_shifted(valid_idx), cary_col(valid_idx)]);
Simulink_Car_Yaw = double([time_shifted(valid_idx), caryaw_col(valid_idx)]);

disp(['✅ Packaged ', num2str(sum(valid_idx)), ' clean data points for Simulink!']);
% --- 6. PLOT VERIFICATION ---
figure('Name', 'Unreal Engine Trajectory', 'Color', 'w');
plot(carx, cary, 'b-', 'LineWidth', 2);
hold on; grid on;
plot(carx(1), cary(1), 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); 
xlabel('Local X (Forward) [meters]', 'FontWeight', 'bold');
ylabel('Local Y (Left) [meters]', 'FontWeight', 'bold');
title('Converted Trajectory for Unreal Engine Vehicle');
legend('Vehicle Path', 'Start (50, 50)', 'Location', 'best');
axis equal;
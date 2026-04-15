% --- LIVE SMARTPHONE PEDESTRIAN TRACKER (WITH PLOTTING) ---
clear; clc; close all;

% --- SETTINGS ---
start_X_offset = 50.0;
start_Y_offset = 50.0;

% 1. INITIALIZE WORKSPACE VARIABLES (Crucial for Simulink at t=0)
assignin('base', 'Ped_X', start_X_offset);
assignin('base', 'Ped_Y', start_Y_offset);
assignin('base', 'Ped_Yaw', 0.0);

% 2. CONNECT TO PHONE
disp('📱 Connecting to mobile device...');
m = mobiledev;
m.PositionSensorEnabled = true;
m.AccelerationSensorEnabled = true;
m.AngularVelocitySensorEnabled = true; 
m.SampleRate = 25; % 25 Hz update rate
pause(2);
m.Logging = true;

% 3. WAIT FOR GPS LOCK (Sets the Origin)
disp('🛰️ Waiting for GPS lock to set Origin... Walk outside!');
origin_lat = NaN;
origin_lon = NaN;
while isnan(origin_lat)
    [lat, lon, ~, ~, ~, ~, acc] = poslog(m); 
    if ~isempty(lat) && acc(end) < 20 
        origin_lat = lat(end);
        origin_lon = lon(end);
        fprintf('✅ Origin Locked! Accuracy: %.2f meters\n', acc(end));
    end
    pause(0.5);
end

% 4. TRACKING SETUP
dt = 0.04; % 25Hz
x_est = [0; 0; 0; 0]; % Kalman State: [Local X, Local Y, Vx, Vy]
P = eye(4); 
A = [1 0 dt 0; 0 1 0 dt; 0 0 0.85 0; 0 0 0 0.85]; % Friction
B = [0.5*dt^2 0; 0 0.5*dt^2; dt 0; 0 dt];
C = [1 0 0 0; 0 1 0 0]; 
Q = eye(4) * 0.05;  
R = eye(2) * 2.0;   
last_gps_time = -1;
ped_yaw_rad = 0.0; 

% --- DATA ARRAYS FOR PLOTTING ---
hist_X = []; 
hist_Y = [];
hist_gps_X = []; 
hist_gps_Y = [];

% --- THE STOP BUTTON UI ---
stop_fig = figure('Name', 'Tracker Control', 'Position', [100, 100, 300, 100], 'MenuBar', 'none', 'ToolBar', 'none');
uicontrol('Style', 'pushbutton', 'String', '🛑 STOP TRACKING & PLOT', ...
          'FontSize', 12, 'FontWeight', 'bold', 'Position', [25, 20, 250, 60], ...
          'Callback', 'delete(gcf)');

disp('======================================================');
disp('🚶 LIVE TRACKING ACTIVE! Start walking.');
disp('Hit RUN in Simulink now.');
disp('Click the STOP button on the pop-up window to finish and plot.');
disp('======================================================');

% 5. MAIN TRACKING LOOP (Runs until you close the Stop window)
while ishandle(stop_fig) 
    [lat, lon, t_gps, ~, ~, ~, gps_acc] = poslog(m);
    [accel, ~] = accellog(m);
    [angvel, ~] = angvellog(m); 
    
    if ~isempty(accel) && ~isempty(angvel)
        
        % --- PROCESS YAW (Gyro Integration) ---
        wz = angvel(end, 3); 
        ped_yaw_rad = ped_yaw_rad + (wz * dt);
        
        % --- PROCESS ACCELERATION ---
        raw_a = accel(end, 2); 
        if abs(raw_a) < 1.0, raw_a = 0; end 
        a_forward = raw_a * 0.1; 
        
        u = [a_forward * cos(ped_yaw_rad); a_forward * sin(ped_yaw_rad)];
        
        % --- KALMAN PREDICT ---
        x_est = A * x_est + B * u;
        P = A * P * A' + Q;
        
        % --- KALMAN UPDATE (GPS) ---
        if ~isempty(t_gps) && t_gps(end) ~= last_gps_time
            last_gps_time = t_gps(end);
            
            x_gps = (lon(end) - origin_lon) * cos(origin_lat * pi/180) * 111320;
            y_gps = (lat(end) - origin_lat) * 110540;
            
            R_dynamic = R * (gps_acc(end) / 5); 
            K = P * C' / (C * P * C' + R_dynamic); 
            x_est = x_est + K * ([x_gps; y_gps] - C * x_est); 
            P = (eye(4) - K * C) * P; 
            
            % Save Raw GPS point with the Unreal Engine offset
            hist_gps_X(end+1) = x_gps + start_X_offset;
            hist_gps_Y(end+1) = y_gps + start_Y_offset;
        end
        
        % --- SAVE SMOOTHED PATH FOR PLOTTING ---
        final_X = x_est(1) + start_X_offset;
        final_Y = x_est(2) + start_Y_offset;
        hist_X(end+1) = final_X;
        hist_Y(end+1) = final_Y;
        
        % --- BROADCAST TO SIMULINK ---
        assignin('base', 'Ped_X', final_X);
        assignin('base', 'Ped_Y', final_Y);
        assignin('base', 'Ped_Yaw', rad2deg(ped_yaw_rad)); 
    end
    
    drawnow limitrate;
    pause(dt); 
end

% --- 6. CLEANUP & PLOTTING ---
m.Logging = false; % Safely turn off the phone connection
disp('Tracking stopped. Generating final trajectory plot...');

figure('Name', 'Final Trajectory', 'Color', 'w', 'Position', [150, 150, 800, 600]);
plot(hist_X, hist_Y, 'b-', 'LineWidth', 2.5);
hold on; grid on;

if ~isempty(hist_gps_X)
    plot(hist_gps_X, hist_gps_Y, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
end

% Plot the (50, 50) starting location
plot(start_X_offset, start_Y_offset, 'g^', 'MarkerSize', 12, 'MarkerFaceColor', 'g');

xlabel('Unreal Engine X Position (meters)', 'FontWeight', 'bold');
ylabel('Unreal Engine Y Position (meters)', 'FontWeight', 'bold');
title('Pedestrian HIL: Smoothed Trajectory vs Raw GPS');
legend('Smoothed Kalman/Gyro Path', 'Raw GPS Updates', 'Start (50,50)', 'Location', 'best');
axis equal;
% === THESIS BASELINE: REAL-TIME 100Hz SENSOR DASHBOARD ===
clear; close all; clc;

% --- 1. SETTINGS ---
test_duration_sec = 180; % 3 Minutes
sample_rate_hz = 100;    % Hardware polling rate
ui_update_rate = 0.1;    % Update graphs every 0.1s (10 Hz) for UI performance
step_threshold = 11.5;   % Accelerometer step detection threshold

% --- 2. CONNECT TO PHONE ---
disp('📱 Connecting to Mobile Device...');
m = mobiledev;
m.PositionSensorEnabled = true;    
m.AccelerationSensorEnabled = true;
m.OrientationSensorEnabled = true; 
m.SampleRate = sample_rate_hz;     
pause(2); % Give sensors a moment to warm up

disp('🛰️ Waiting for Phone GPS lock to set Origin... Walk outside!');
origin_lat = NaN;
origin_lon = NaN;
start_time = NaN;

while isnan(origin_lat)
    [lat, lon, t_gps, ~, ~, ~, acc] = poslog(m); 
    if ~isempty(lat) && acc(end) < 20 
        origin_lat = lat(end);
        origin_lon = lon(end);
        start_time = t_gps(1); % Lock the T=0 starting time
        fprintf('✅ Origin Locked! Initial Accuracy: %.2f meters\n', acc(end));
    end
    pause(0.5); 
end

% --- 3. SETUP REAL-TIME DASHBOARD (3 SUBPLOTS) ---
fig_main = figure('Name', 'Live Hardware Baseline', 'Color', 'w', 'Position', [100, 100, 1000, 800], 'MenuBar', 'none');

% Plot 1: GPS Trajectory
ax1 = subplot(2, 2, [1, 3]); % Takes up the left half
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
title(ax1, 'Raw GPS Trajectory (Local Meters)', 'FontWeight', 'bold');
xlabel(ax1, 'X [m]'); ylabel(ax1, 'Y [m]');
line_gps = animatedline(ax1, 'Marker', 'o', 'Color', [0.8500 0.3250 0.0980], 'LineStyle', '-', 'LineWidth', 1.5, 'MarkerSize', 3);
plot(ax1, 0, 0, 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start Marker

% Plot 2: Accelerometer
ax2 = subplot(2, 2, 2); % Top right
hold(ax2, 'on'); grid(ax2, 'on');
title(ax2, 'Accelerometer Magnitude (100 Hz)', 'FontWeight', 'bold');
ylabel(ax2, 'Accel (m/s²)');
yline(ax2, 9.81, 'k-', 'Gravity', 'LabelHorizontalAlignment', 'left');
yline(ax2, step_threshold, 'r--', 'Step Threshold', 'LabelHorizontalAlignment', 'left');
line_acc = animatedline(ax2, 'Color', [0.0 0.4470 0.7410], 'LineWidth', 1);
xlim(ax2, [0, 10]); % Initial limit

% Plot 3: Yaw (Compass)
ax3 = subplot(2, 2, 4); % Bottom right
hold(ax3, 'on'); grid(ax3, 'on');
title(ax3, 'Yaw / Compass Heading', 'FontWeight', 'bold');
xlabel(ax3, 'Time [s]'); ylabel(ax3, 'Degrees');
line_yaw = animatedline(ax3, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.5);
xlim(ax3, [0, 10]); % Initial limit

% --- Index Tracking (To only plot new data each loop) ---
last_idx_gps = 0;
last_idx_acc = 0;
last_idx_yaw = 0;

% --- 4. START RECORDING ---
disp('======================================================');
fprintf('🔥 RECORDING LIVE DATA FOR %d SECONDS...\n', test_duration_sec);
disp('Close the window at any time to stop early.');
disp('======================================================');

m.Logging = true;
tic; % Start the stopwatch

try
    while ishandle(fig_main) && toc < test_duration_sec
        
        % --- Extract New Data ---
        [lat, lon, t_gps] = poslog(m);
        [accel, t_accel] = accellog(m);
        [orient, t_orient] = orientlog(m);
        
        % --- Update GPS (Plot 1) ---
        if length(t_gps) > last_idx_gps
            new_lat = lat(last_idx_gps+1 : end);
            new_lon = lon(last_idx_gps+1 : end);
            
            new_x = (new_lon - origin_lon) .* cos(origin_lat * pi/180) * 111320;
            new_y = (new_lat - origin_lat) * 110540;
            
            addpoints(line_gps, new_x, new_y);
            last_idx_gps = length(t_gps);
        end
        
        % --- Update Accelerometer (Plot 2) ---
        if length(t_accel) > last_idx_acc
            new_accel = accel(last_idx_acc+1 : end, :);
            new_t_acc = t_accel(last_idx_acc+1 : end) - start_time;
            
            new_mag = sqrt(new_accel(:,1).^2 + new_accel(:,2).^2 + new_accel(:,3).^2);
            
            addpoints(line_acc, new_t_acc, new_mag);
            
            % Anchor X-axis at 0 and let it grow indefinitely
            xlim(ax2, [0, max(new_t_acc(end), 10)]); 
            
            last_idx_acc = length(t_accel);
        end
        
        % --- Update Yaw (Plot 3) ---
        if length(t_orient) > last_idx_yaw
            new_yaw = -orient(last_idx_yaw+1 : end, 1); % Counter-clockwise positive
            new_t_yaw = t_orient(last_idx_yaw+1 : end) - start_time;
            
            addpoints(line_yaw, new_t_yaw, new_yaw);
            
            % Anchor X-axis at 0 and let it grow indefinitely
            xlim(ax3, [0, max(new_t_yaw(end), 10)]); 
            
            last_idx_yaw = length(t_orient);
        end
        
        % Refresh the UI and wait for the next cycle
        drawnow limitrate;
        pause(ui_update_rate); 
    end
    
    m.Logging = false;
    disp('✅ Test Complete! Save the figure for your thesis.');

catch ME
    m.Logging = false;
    disp('🛑 Dashboard closed early.');
end
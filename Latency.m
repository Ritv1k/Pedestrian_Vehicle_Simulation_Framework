clc;
clear m;
% --- 0. TEST SETTINGS ---
test_duration_sec = 240; % 4 Minutes (Change this to whatever length you want)

% --- 1. INITIALIZATION ---
disp('📱 Connecting to Phone & Initializing Sensors...');
m = mobiledev;
m.PositionSensorEnabled = true;        % Position
m.OrientationSensorEnabled = true;     % Orientation
m.AccelerationSensorEnabled = true;    % Accelerometer
m.AngularVelocitySensorEnabled = true; % Angular Velocity
m.SampleRate = 100; 
pause(2);

disp('======================================================');
fprintf('📡 MEASURING END-TO-END LATENCY (%d Seconds)\n', test_duration_sec);
disp('Recording silently in background to prevent terminal lag...');
disp('======================================================');

m.Logging = true;
pc_start_time = tic;

% --- 2. STORAGE VARIABLES ---
pc_time_gps = []; latency_gps = []; last_t_gps = -1;
pc_time_yaw = []; latency_yaw = []; last_t_yaw = -1;
pc_time_acc = []; latency_acc = []; last_t_acc = -1;
pc_time_gyr = []; latency_gyr = []; last_t_gyr = -1;

% ONE Master Sync to align PC clock with Phone clock
master_sync_offset = -1; 

% --- 3. MAIN RECORDING LOOP ---
while toc(pc_start_time) < test_duration_sec
    current_pc_time = toc(pc_start_time);
    
    % Fetch live sensor logs
    [~, ~, t_gps] = poslog(m);
    [~, t_yaw] = orientlog(m);
    [~, t_acc] = accellog(m);
    [~, t_gyr] = angvellog(m);
    
    % Establish Master Sync using the first valid high-speed packet
    if master_sync_offset == -1 && ~isempty(t_yaw)
        master_sync_offset = current_pc_time - t_yaw(end);
        disp('✅ Master Clocks Synchronized! Recording...');
    end
    
    % Only record if clocks are synced
    if master_sync_offset ~= -1
        
        % --- POSITION LATENCY ---
        if ~isempty(t_gps)
            latest_t = t_gps(end);
            if latest_t ~= last_t_gps 
                lat_ms = abs(current_pc_time - (latest_t + master_sync_offset)) * 1000 + 40;
                latency_gps(end+1) = lat_ms;
                pc_time_gps(end+1) = current_pc_time;
                last_t_gps = latest_t;
            end
        end
        
        % --- ORIENTATION LATENCY ---
        if ~isempty(t_yaw)
            latest_t = t_yaw(end);
            if latest_t ~= last_t_yaw
                lat_ms = abs(current_pc_time - (latest_t + master_sync_offset)) * 1000 + 40;
                latency_yaw(end+1) = lat_ms;
                pc_time_yaw(end+1) = current_pc_time;
                last_t_yaw = latest_t;
            end
        end
        
        % --- ACCELEROMETER LATENCY ---
        if ~isempty(t_acc)
            latest_t = t_acc(end);
            if latest_t ~= last_t_acc
                lat_ms = abs(current_pc_time - (latest_t + master_sync_offset)) * 1000 + 40;
                latency_acc(end+1) = lat_ms;
                pc_time_acc(end+1) = current_pc_time;
                last_t_acc = latest_t;
            end
        end
        
        % --- ANGULAR VELOCITY LATENCY ---
        if ~isempty(t_gyr)
            latest_t = t_gyr(end);
            if latest_t ~= last_t_gyr
                lat_ms = abs(current_pc_time - (latest_t + master_sync_offset)) * 1000 + 40;
                latency_gyr(end+1) = lat_ms;
                pc_time_gyr(end+1) = current_pc_time;
                last_t_gyr = latest_t;
            end
        end
    end
    
    pause(0.01); 
end

% Stop the clock to get the EXACT duration for perfect Hz calculation
actual_duration = toc(pc_start_time); 
m.Logging = false;
disp('✅ Data collection complete. Generating graphs and statistics...');

% --- 4. STATISTICAL CALCULATIONS ---
% Pull the FULL arrays from the buffer to bypass Wi-Fi batching and get the true sensor count
[~, ~, t_gps_full] = poslog(m);
[~, t_yaw_full] = orientlog(m);
[~, t_acc_full] = accellog(m);
[~, t_gyr_full] = angvellog(m);

% Updated helper function to accept the true total count
print_stats = @(name, lat_data, full_count, duration) fprintf(...
    '[%s] Average: %6.1f ms | Median: %6.1f ms | StdDev: %6.1f ms | Min: %6.1f ms | Max: %6.1f ms | True Hz: %4.1f\n', ...
    name, mean(lat_data), median(lat_data), std(lat_data), min(lat_data), max(lat_data), full_count/duration);

fprintf('\n======================== FINAL LATENCY STATISTICS ========================\n');
all_averages = [];

if ~isempty(latency_gps)
    print_stats('Position        ', latency_gps, length(t_gps_full), actual_duration);
    all_averages(end+1) = mean(latency_gps);
end
if ~isempty(latency_yaw)
    print_stats('Orientation     ', latency_yaw, length(t_yaw_full), actual_duration);
    all_averages(end+1) = mean(latency_yaw);
end
if ~isempty(latency_acc)
    print_stats('Accelerometer   ', latency_acc, length(t_acc_full), actual_duration);
    all_averages(end+1) = mean(latency_acc);
end
if ~isempty(latency_gyr)
    print_stats('Angular Velocity', latency_gyr, length(t_gyr_full), actual_duration);
    all_averages(end+1) = mean(latency_gyr);
end

% Print the overall system average
fprintf('--------------------------------------------------------------------------\n');
if ~isempty(all_averages)
    fprintf('[SYSTEM OVERALL  ] Average Latency: %6.1f ms\n', mean(all_averages));
end
fprintf('==========================================================================\n');

% --- 5. PLOTTING (4 SEPARATE GRAPHS) ---
fig = figure('Name', 'Multi-Sensor Latency Analysis', 'Color', 'w', 'Position', [100, 100, 1200, 800]);

% Graph 1: Position
ax1 = subplot(2, 2, 1);
if ~isempty(latency_gps)
    plot(ax1, pc_time_gps, latency_gps, '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5);
    title(ax1, sprintf('Position Latency (Average: %.1f ms)', mean(latency_gps)), 'FontWeight', 'bold');
    xlabel(ax1, 'Time (s)'); ylabel(ax1, 'Latency (ms)'); grid(ax1, 'on'); xlim(ax1, [0 test_duration_sec]);
end

% Graph 2: Orientation
ax2 = subplot(2, 2, 2);
if ~isempty(latency_yaw)
    plot(ax2, pc_time_yaw, latency_yaw, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.0);
    title(ax2, sprintf('Orientation Latency (Average: %.1f ms)', mean(latency_yaw)), 'FontWeight', 'bold');
    xlabel(ax2, 'Time (s)'); ylabel(ax2, 'Latency (ms)'); grid(ax2, 'on'); xlim(ax2, [0 test_duration_sec]);
end

% Graph 3: Accelerometer
ax3 = subplot(2, 2, 3);
if ~isempty(latency_acc)
    plot(ax3, pc_time_acc, latency_acc, '-', 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 1.0);
    title(ax3, sprintf('Accelerometer Latency (Average: %.1f ms)', mean(latency_acc)), 'FontWeight', 'bold');
    xlabel(ax3, 'Time (s)'); ylabel(ax3, 'Latency (ms)'); grid(ax3, 'on'); xlim(ax3, [0 test_duration_sec]);
end

% Graph 4: Angular Velocity
ax4 = subplot(2, 2, 4);
if ~isempty(latency_gyr)
    plot(ax4, pc_time_gyr, latency_gyr, '-', 'Color', [0.4940 0.1840 0.5560], 'LineWidth', 1.0);
    title(ax4, sprintf('Angular Velocity Latency (Average: %.1f ms)', mean(latency_gyr)), 'FontWeight', 'bold');
    xlabel(ax4, 'Time (s)'); ylabel(ax4, 'Latency (ms)'); grid(ax4, 'on'); xlim(ax4, [0 test_duration_sec]);
end
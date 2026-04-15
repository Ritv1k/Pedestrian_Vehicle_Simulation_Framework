% === 2-MINUTE LIVE GPS INACCURACY & DRIFT VISUALIZER ===
if exist('m','var'); clear m; end
clc; close all;

% --- 0. TEST SETTINGS ---
test_duration_sec = 120; % Strictly run for 2 minutes

% --- 1. INITIALIZATION ---
disp('📱 Connecting to Phone & Initializing GPS...');
m = mobiledev;
m.PositionSensorEnabled = true; % Enable GPS
m.Logging = true;
pause(2);

disp('🧭 Waiting for Initial GPS Lock...');
lat_origin = NaN;
lon_origin = NaN;

while isnan(lat_origin)
    [lat, lon, ~, ~, ~, ~, acc] = poslog(m);
    if ~isempty(lat)
        lat_origin = lat(end);
        lon_origin = lon(end);
        disp('✅ GPS Locked! Setting Local Origin (0,0).');
    end
    pause(0.5);
end

% --- 2. UI DASHBOARD (DUAL PLOT) ---
fig_main = figure('Name', 'Live GPS Inaccuracy Tracker', 'Position', [100, 100, 1200, 600], 'Color', 'w');

% Plot 1: The 2D Map with Uncertainty Bubbles
ax1 = subplot(1, 2, 1, 'Parent', fig_main);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
title(ax1, 'Live GPS Drift & Inaccuracy', 'FontWeight', 'bold', 'FontSize', 14);
xlabel(ax1, 'Local X [meters]', 'FontWeight', 'bold'); 
ylabel(ax1, 'Local Y [meters]', 'FontWeight', 'bold');
line_traj = animatedline(ax1, 'Color', 'k', 'LineWidth', 2, 'DisplayName', 'Reported GPS Path');
plot(ax1, 0, 0, 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Starting Origin');
fill(ax1, NaN, NaN, 'r', 'FaceAlpha', 0.15, 'EdgeColor', 'r', 'DisplayName', 'Error Radius');

% Plot 2: Accuracy Over Time
ax2 = subplot(1, 2, 2, 'Parent', fig_main);
hold(ax2, 'on'); grid(ax2, 'on');
title(ax2, 'Horizontal Accuracy Over Time', 'FontWeight', 'bold', 'FontSize', 14);
xlabel(ax2, 'Time Elapsed [s]', 'FontWeight', 'bold'); 
ylabel(ax2, 'Inaccuracy Radius [meters]', 'FontWeight', 'bold');
line_acc = animatedline(ax2, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
xlim(ax2, [0, test_duration_sec]); % Lock X-axis to the full 2 minutes

% Constants & Tracking Variables
R_earth = 6378137; % Earth radius in meters
lat_rad = deg2rad(lat_origin);
last_t_gps = -1;
theta = linspace(0, 2*pi, 40); % For drawing the circles
acc_history = []; % To calculate the average

disp('======================================================');
fprintf('🚶 SYSTEM LIVE! Recording for %d seconds...\n', test_duration_sec);
disp('Walk around to see how your GPS accuracy fluctuates.');
disp('======================================================');

% --- 3. MAIN LOOP (Strict 120 Second Timer) ---
test_start_time = tic;

try
    while toc(test_start_time) < test_duration_sec && ishandle(fig_main)
        
        elapsed_time = toc(test_start_time);
        
        % Fetch the latest GPS log
        [lat_log, lon_log, t_log, ~, ~, ~, acc_log] = poslog(m);
        
        if ~isempty(t_log)
            latest_t = t_log(end);
            
            % Only plot if we received a NEW GPS coordinate (GPS is 1Hz)
            if latest_t > last_t_gps
                current_lat = lat_log(end);
                current_lon = lon_log(end);
                current_acc = acc_log(end); % Error Radius in METERS
                
                % Track accuracy for the average calculation
                acc_history(end+1) = current_acc;
                current_avg = mean(acc_history);
                
                % 1. Convert Lat/Lon to Local X/Y in METERS
                dLat = deg2rad(current_lat - lat_origin);
                dLon = deg2rad(current_lon - lon_origin);
                local_x = dLon * R_earth * cos(lat_rad);
                local_y = dLat * R_earth;
                
                % 2. Update Map Trajectory (This stays on top!)
                addpoints(line_traj, local_x, local_y);
                
                % 3. Draw the translucent red "Uncertainty Bubble"
                circle_x = local_x + (current_acc * cos(theta));
                circle_y = local_y + (current_acc * sin(theta));
                
                % Save the patch to a variable (h_bubble) and push it to the bottom
                h_bubble = fill(ax1, circle_x, circle_y, 'r', 'FaceAlpha', 0.10, 'EdgeColor', 'none');
                uistack(h_bubble, 'bottom'); 
                
                % Dynamically scale the Map view so bubbles don't get cut off
                axis_padding = max(10, current_acc * 1.5);
                xlim(ax1, [local_x - axis_padding, local_x + axis_padding]);
                ylim(ax1, [local_y - axis_padding, local_y + axis_padding]);
                
                % 4. Update the Accuracy Over Time Graph
                addpoints(line_acc, elapsed_time, current_acc);
                
                % Update title with live running average
                title(ax2, sprintf('Accuracy Over Time (Live Avg: %.2f m)', current_avg), 'FontWeight', 'bold', 'FontSize', 14);
                
                % Minimal print to terminal
                fprintf('📍 Time: %4.0fs | Error Radius: %4.1fm | Running Avg: %4.1fm\n', ...
                    elapsed_time, current_acc, current_avg);
                
                last_t_gps = latest_t;
                drawnow limitrate;
            end
        end
        
        pause(0.1); % Keep MATLAB responsive
    end
    
    % --- 4. FINAL CLEANUP & STATS ---
    m.Logging = false;
    disp('✅ 2-Minute Test Complete.');
    
    fprintf('\n================== FINAL GPS ACCURACY STATISTICS ==================\n');
    if ~isempty(acc_history)
        fprintf('Total Data Points Logged : %d\n', length(acc_history));
        fprintf('Average (Mean) Inaccuracy: %.2f meters\n', mean(acc_history));
        fprintf('Worst (Max) Inaccuracy   : %.2f meters\n', max(acc_history));
        fprintf('Best (Min) Inaccuracy    : %.2f meters\n', min(acc_history));
    else
        fprintf('No GPS data was logged. Ensure your phone has a clear view of the sky.\n');
    end
    fprintf('===================================================================\n');

catch ME
    m.Logging = false;
    disp('🛑 Tracking interrupted.');
    rethrow(ME);
end
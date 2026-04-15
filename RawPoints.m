% === THESIS DATA COLLECTION: 1-MINUTE GPS TRAJECTORY ===
clear; close all; clc;

% --- 1. SETTINGS ---
test_duration_sec = 60;  % 1 minute test
sample_rate_hz = 100;     % 10 Hz is plenty for raw GPS polling

% --- 2. CONNECT TO PHONE ---
disp('📱 Connecting to Mobile Device...');
m = mobiledev;
m.PositionSensorEnabled = true;     % We only need GPS for this script
m.SampleRate = sample_rate_hz;     
pause(2); % Give the GPS chip a moment to wake up and lock

% --- 3. RECORD DATA ---
disp('======================================================');
fprintf('🔥 RECORDING GPS DATA FOR %d SECONDS...\n', test_duration_sec);
disp('Walk outside in a large loop, square, or specific shape!');
disp('======================================================');

m.Logging = true;
pause(test_duration_sec); % Wait while data is collected
m.Logging = false;

disp('✅ Recording complete. Processing spatial data...');

% --- 4. EXTRACT LOGS ---
[lat, lon, t_gps, speed, course, alt, horiz_acc] = poslog(m);

% --- 5. PROCESS & PLOT GPS (X/Y TRAJECTORY) ---
if length(lat) >= 2
    % Set the very first GPS point as our (0,0) Origin
    origin_lat = lat(1);
    origin_lon = lon(1);
    
    % Convert Lat/Lon differences to physical meters
    % 1 degree of latitude is ~110,540 meters
    % 1 degree of longitude is ~111,320 * cos(latitude) meters
    gps_x_meters = (lon - origin_lon) * cos(origin_lat * pi/180) * 111320;
    gps_y_meters = (lat - origin_lat) * 110540;
    
    % --- GENERATE THESIS FIGURE ---
    figure('Name', '1-Minute GPS Walking Trajectory', 'Color', 'w', 'Position', [150, 150, 700, 600]);
    
    % Plot the continuous path
    plot(gps_x_meters, gps_y_meters, '-o', 'Color', [0.8500 0.3250 0.0980], ...
         'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', [0.8500 0.3250 0.0980]);
    hold on; grid on; 
    
    % Force the axes to be equal so physical distances are proportionate
    % (A 10m x 10m square will actually look like a square)
    axis equal; 
    
    % Highlight the Start and End points
    plot(gps_x_meters(1), gps_y_meters(1), 'g^', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot(gps_x_meters(end), gps_y_meters(end), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    % Formatting
    title('Raw GPS Walking Trajectory (1-Minute Trial)', 'FontWeight', 'bold', 'FontSize', 14);
    xlabel('Local X (meters)', 'FontWeight', 'bold');
    ylabel('Local Y (meters)', 'FontWeight', 'bold');
    legend('GPS Path', 'Start (Green)', 'End (Red)', 'Location', 'best');
    
    % Print some quick stats to the command window
    % Calculate the total distance walked by summing the distance between each point
    total_distance = sum(sqrt(diff(gps_x_meters).^2 + diff(gps_y_meters).^2));
    fprintf('➡️ Total Distance Tracked: %.2f meters\n', total_distance);
    fprintf('➡️ Average Horizontal Accuracy: %.2f meters\n', mean(horiz_acc));
else
    disp('⚠️ WARNING: Not enough GPS data to plot a spatial trajectory.');
    disp('Make sure you have a clear view of the sky!');
end

% --- 6. CLEANUP ---
clear m;
disp('Data collection and plotting finished.');
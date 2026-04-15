% === THESIS DATA COLLECTION: REAL ACCELEROMETER STEP DETECTION ===
clear; close all; clc;

% --- 1. SETTINGS ---
test_duration_sec = 60;  % 15 seconds is perfect for a quick walking test
sample_rate_hz = 100;     % 50 Hz is standard for step detection
step_threshold = 11.5;   % m/s^2 (Gravity is 9.8, so 11.5 detects a firm step)

% --- 2. CONNECT TO PHONE ---
disp('📱 Connecting to Mobile Device...');
m = mobiledev;
m.AccelerationSensorEnabled = true; % We only need the Accelerometer
m.SampleRate = sample_rate_hz;     
pause(2); % Give sensor a moment to warm up

% --- 3. RECORD DATA ---
disp('======================================================');
fprintf('🔥 RECORDING FOR %d SECONDS...\n', test_duration_sec);
disp('Put the phone in your hand or pocket and walk normally!');
disp('======================================================');

m.Logging = true;
pause(test_duration_sec); % Wait while data is collected
m.Logging = false;

disp('✅ Recording complete. Processing motion data...');

% --- 4. EXTRACT & PROCESS DATA ---
[accel, t_accel] = accellog(m);

if length(t_accel) > 10
    % Time vector starting from 0
    time_sec = t_accel - t_accel(1);
    
    % Calculate the 3D Magnitude using the Pythagorean theorem
    % Formula: sqrt(X^2 + Y^2 + Z^2)
    accel_mag = sqrt(accel(:,1).^2 + accel(:,2).^2 + accel(:,3).^2);
    
    % --- 5. STEP COUNTING ALGORITHM (PEAK DETECTION) ---
    % Find peaks that cross our threshold. 
    % 'MinPeakDistance' prevents double-counting a single messy footstep.
    % Assuming max 2.5 steps per second, minimum distance is 50Hz / 2.5 = 20 samples
    [peak_values, peak_indices] = findpeaks(accel_mag, ...
        'MinPeakHeight', step_threshold, ...
        'MinPeakDistance', round(sample_rate_hz / 2.5));
    
    time_of_steps = time_sec(peak_indices);
    total_steps = length(peak_values);
    
    fprintf('➡️ Total Steps Detected: %d\n', total_steps);
    
    % --- 6. GENERATE THESIS FIGURE ---
    figure('Name', 'Accelerometer Step Detection', 'Color', 'w', 'Position', [100, 100, 900, 400]);
    
    % Plot the raw magnitude data
    plot(time_sec, accel_mag, 'Color', [0.0 0.4470 0.7410], 'LineWidth', 1.2);
    hold on; grid on;
    
    % Plot the Gravity Baseline (~9.8 m/s^2)
    yline(9.81, '-', 'Gravity Baseline (9.81 m/s²)', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    
    % Plot the Detection Threshold Line
    yline(step_threshold, 'r--', sprintf('Detection Threshold (%.1f m/s²)', step_threshold), ...
        'LineWidth', 2, 'LabelHorizontalAlignment', 'left', 'LabelVerticalAlignment', 'bottom');
    
    % Highlight the exact moment a step was counted (The Peaks)
    plot(time_of_steps, peak_values, 'rv', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    
    % Formatting
    title(sprintf('Pedestrian Motion Analysis: %d Steps Detected', total_steps), 'FontWeight', 'bold', 'FontSize', 14);
    ylabel('Acceleration Magnitude (m/s²)', 'FontWeight', 'bold');
    xlabel('Time (s)', 'FontWeight', 'bold');
    legend('Raw Acceleration', 'Gravity', 'Threshold', 'Detected Steps', 'Location', 'northeast');
    
    % Scale Y-axis to frame the data perfectly
    ylim([min(accel_mag) - 2, max(max(accel_mag) + 2, step_threshold + 3)]);
    xlim([0, max(time_sec)]);

else
    disp('⚠️ WARNING: Not enough accelerometer data recorded.');
end

% --- 7. CLEANUP ---
clear m;
disp('Data collection and plotting finished.');
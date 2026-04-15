% === MASTER HIL CONTROLLER: DEAD-RECKONING + CONTINUOUS WALK TIMER ===
clear; close all; clc;

% --- 1. GLOBAL SETTINGS & OFFSETS ---
car_X_offset = 15.0;
car_Y_offset = 15.0;
ped_X_offset = 25.0;   
ped_Y_offset = 25.0;   

% --- 2. Port Routing ---
matlabListenPort = 5000;      
simulinkIP = "192.168.0.127"; 
simulinkPort = 8002;          

% --- 3. Create UDP Objects ---
disp(['📡 MATLAB listening for Car data on port ', num2str(matlabListenPort), '...']);
disp(['📡 MATLAB sending Pedestrian data to ', char(simulinkIP), ':', num2str(simulinkPort)]);

u_recv = udpport("datagram", "IPv4", "LocalPort", matlabListenPort);
u_recv.ByteOrder = "big-endian"; 
u_send = udpport("datagram", "IPv4");
u_send.ByteOrder = "big-endian";
flush(u_recv);

% --- 4. SETUP PEDESTRIAN CONNECTION (PHONE) ---
disp('📱 Connecting to Mobile Device for Pedestrian...');
m = mobiledev;
m.PositionSensorEnabled = true;    
m.OrientationSensorEnabled = true; 
m.AccelerationSensorEnabled = true;
m.SampleRate = 100;                
pause(2);
m.Logging = true;

disp('🛰️ Waiting for Phone GPS lock to set Origin... Walk outside!');
ped_origin_lat = NaN;
ped_origin_yaw_rad = NaN; 

while isnan(ped_origin_lat)
    [lat, ~, ~, ~, ~, ~, acc] = poslog(m); 
    [orient, ~] = orientlog(m);
    if ~isempty(lat) && ~isempty(orient) && acc(end) < 20 
        ped_origin_lat = lat(end);
        ped_origin_yaw_rad = deg2rad(-orient(end, 1));
        fprintf('✅ Pedestrian Origin & Yaw Locked! Switching to smooth dead-reckoning.\n');
    end
    pause(0.5); 
end

% --- 5. DEAD RECKONING SETUP ---
dt = 0.04;             
ped_local_x = 0.0;     
ped_local_y = 0.0;     
walk_speed = 1.4;      
accel_threshold = 11.5; 

% --- State Variables & Step Timer ---
prev_car_reset = 0; 
ped_reset_out = 0;  
last_idx_acc = 0;             
time_since_last_step = 999;   % The walking timer
start_time_acc = NaN;         

% --- 6. INITIALIZE WORKSPACE FOR SIMULINK ---
assignin('base', 'Ped_X', ped_X_offset);
assignin('base', 'Ped_Y', ped_Y_offset);
assignin('base', 'Ped_Yaw', 0.0);
assignin('base', 'Car_X', car_X_offset);
assignin('base', 'Car_Y', car_Y_offset);
assignin('base', 'Car_Yaw', 0.0);
assignin('base', 'Car_Status', 0.0);

% --- 7. UI DASHBOARD ---
fig_main = figure('Name', 'Dead-Reckoning HIL Dashboard', 'Position', [100, 100, 1000, 500], 'MenuBar', 'none');
setappdata(fig_main, 'TriggerReset', false);

uicontrol('Style', 'pushbutton', 'String', '🔄 RESET ORIGINS & YAW', ...
          'FontSize', 12, 'FontWeight', 'bold', 'Position', [20, 20, 250, 50], ...
          'Callback', @(src, event) setappdata(fig_main, 'TriggerReset', true));

uicontrol('Style', 'text', 'String', '(Close this window to Stop Tracking)', ...
          'FontSize', 10, 'Position', [280, 20, 250, 40], ...
          'HorizontalAlignment', 'left', 'BackgroundColor', get(fig_main, 'Color'));

% Plot 1: Trajectory
ax1 = subplot(1, 2, 1, 'Parent', fig_main);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
title(ax1, 'Live Dead-Reckoning Trajectory', 'FontWeight', 'bold', 'FontSize', 12);
xlabel(ax1, 'Local X [meters]', 'FontWeight', 'bold'); 
ylabel(ax1, 'Local Y [meters]', 'FontWeight', 'bold');
line_traj = animatedline(ax1, 'Color', [0.0 0.4470 0.7410], 'LineWidth', 2.5);
plot(ax1, ped_X_offset, ped_Y_offset, 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); 

% Plot 2: Accelerometer
ax2 = subplot(1, 2, 2, 'Parent', fig_main);
hold(ax2, 'on'); grid(ax2, 'on');
title(ax2, 'Live 3D Accelerometer Magnitude', 'FontWeight', 'bold', 'FontSize', 12);
xlabel(ax2, 'Time [s]', 'FontWeight', 'bold'); 
ylabel(ax2, 'Magnitude (m/s²)', 'FontWeight', 'bold');
yline(ax2, accel_threshold, 'r--', 'Step Threshold (11.5)', 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
yline(ax2, 9.81, 'k-', 'Gravity', 'LabelHorizontalAlignment', 'left');
line_acc = animatedline(ax2, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5);
xlim(ax2, [0, 10]);

disp('======================================================');
disp('🔥 SYSTEM LIVE! Tracking Both Agents...');
disp('======================================================');

try
    while ishandle(fig_main) 
        
        % ---------------------------------------------------------
        % SYSTEM RESET LOGIC 
        % ---------------------------------------------------------
        if getappdata(fig_main, 'TriggerReset') == true
            disp('🔄 Executing System Reset: Zeroing Dead-Reckoning & Re-locking Yaw...');
            
            ped_reset_out = 1; 
            
            ped_local_x = 0.0;
            ped_local_y = 0.0;
            ped_origin_yaw_rad = NaN; 
            
            % Reset Timers & Indexes
            [~, current_t_accel] = accellog(m);
            last_idx_acc = length(current_t_accel); 
            time_since_last_step = 999;
            start_time_acc = NaN; 
            
            clearpoints(line_traj);
            clearpoints(line_acc);
            
            setappdata(fig_main, 'TriggerReset', false);
        else
            ped_reset_out = 0; 
        end
        
        % ---------------------------------------------------------
        % AGENT 1: RECEIVE DATA (Car)
        % ---------------------------------------------------------
        if u_recv.NumDatagramsAvailable > 0
            dgrams = read(u_recv, u_recv.NumDatagramsAvailable); 
            raw_bytes = dgrams(end).Data; 
            
            if length(raw_bytes) == 32
                parsed_doubles = swapbytes(typecast(uint8(raw_bytes), 'double'));
                
                carx      = parsed_doubles(1) + car_X_offset;
                cary      = parsed_doubles(2) + car_Y_offset;
                caryaw    = parsed_doubles(3); 
                car_reset = parsed_doubles(4); 
                
                if car_reset == 1 && prev_car_reset == 0
                    setappdata(fig_main, 'TriggerReset', true);
                    disp('🔄 RESET TRIGGER RECEIVED FROM SIMULINK!');
                end
                prev_car_reset = car_reset;
                
                assignin('base', 'Car_X', carx);
                assignin('base', 'Car_Y', cary);
                assignin('base', 'Car_Yaw', caryaw);
                assignin('base', 'Car_Status', car_reset);
            end
        end
        
        % ---------------------------------------------------------
        % AGENT 2: PEDESTRIAN DEAD-RECKONING & SEND
        % ---------------------------------------------------------
        [accel, t_accel] = accellog(m);
        [orient, ~] = orientlog(m); 
        
        if ~isempty(accel) && ~isempty(orient)
            
            % 1. Get Yaw and calculate Relative Heading
            raw_yaw_rad = deg2rad(-orient(end, 1));
            if isnan(ped_origin_yaw_rad)
                ped_origin_yaw_rad = raw_yaw_rad;
                disp('✅ Pedestrian Yaw Re-Locked!');
            end
            
            ped_yaw_rad = raw_yaw_rad - ped_origin_yaw_rad;
            ped_yaw_deg = rad2deg(ped_yaw_rad);
            
            % 2. Detect Movement via Batching & The Step Timer
            if length(t_accel) > last_idx_acc
                
                if isnan(start_time_acc)
                    start_time_acc = t_accel(end); % Lock current time
                end
                
                % Extract the new batch of data
                new_accel = accel(last_idx_acc+1 : end, :);
                new_t_acc = t_accel(last_idx_acc+1 : end) - start_time_acc;
                
                % Calculate 3D Magnitude
                new_mag = sqrt(new_accel(:,1).^2 + new_accel(:,2).^2 + new_accel(:,3).^2);
                
                % Update Plot with exact hardware timestamps
                addpoints(line_acc, new_t_acc, new_mag);
                xlim(ax2, [0, max(new_t_acc(end), 10)]); 
                
                % Did ANY microsecond in this batch cross the threshold?
                if max(new_mag) > accel_threshold
                    time_since_last_step = 0; % RESET THE WALKING TIMER
                end
                
                last_idx_acc = length(t_accel);
            end
            
            % 3. Apply Continuous Speed
            % If you took a step in the last 1.5 seconds, you are walking continuously.
            if time_since_last_step < 1.50
                current_speed = walk_speed;
            else
                current_speed = 0.0; 
            end
            
            % Increment the timer
            time_since_last_step = time_since_last_step + dt;
            
            % 4. Dead Reckoning Integration 
            ped_local_x = ped_local_x + (current_speed * cos(ped_yaw_rad) * dt);
            ped_local_y = ped_local_y + (current_speed * sin(ped_yaw_rad) * dt);
            
            % 5. Apply the Offsets for Simulink
            final_ped_X = ped_local_x + ped_X_offset;
            final_ped_Y = ped_local_y + ped_Y_offset;
            
            % Update Trajectory Plot
            addpoints(line_traj, final_ped_X, final_ped_Y);
            
            % Push to Simulink base workspace
            assignin('base', 'Ped_X', final_ped_X);
            assignin('base', 'Ped_Y', final_ped_Y);
            assignin('base', 'Ped_Yaw', ped_yaw_deg); 
            
            % 6. Send 4 doubles out over UDP (X, Y, Yaw, ResetBit)
            reply = double([final_ped_X, final_ped_Y, ped_yaw_deg, ped_reset_out]);
            write(u_send, reply, "double", simulinkIP, simulinkPort);
            
            % Minimal print display
            fprintf('🔼 [PED] X: %8.2f | Y: %8.2f | Yaw: %8.2f° | Spd: %3.1f | Res: %d\n', ...
                    final_ped_X, final_ped_Y, ped_yaw_deg, current_speed, ped_reset_out);
        end
        
        drawnow limitrate;
        pause(dt); 
    end

    % --- Normal Cleanup ---
    m.Logging = false; 
    clear u_recv u_send;
    disp('Tracking stopped normally.');

catch ME
    m.Logging = false;
    clear u_recv u_send;
    fprintf('\n🛑 Communication stopped and ports cleared.\n');
    rethrow(ME);
end
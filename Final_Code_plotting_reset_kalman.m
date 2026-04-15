% === MASTER HIL CONTROLLER: ORIGINAL KALMAN + RESET + YAW SYNC ===
clear; close all; clc;

% --- 1. GLOBAL SETTINGS & OFFSETS ---
car_X_offset = 15.0;
car_Y_offset = 15.0;
ped_X_offset = 20.0;   
ped_Y_offset = 20.0;   

% --- 2. Port Routing ---
matlabListenPort = 5000;      % MATLAB listens here (receiving Car data)
simulinkIP = "192.168.1.77";  % MATLAB sends to this IP (sending Pedestrian data)
simulinkPort = 20009;         % MATLAB sends to this Port

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
m.PositionSensorEnabled = true;    % For Lat, Lon, and Horizontal Accuracy
m.OrientationSensorEnabled = true; % For Absolute Yaw (Azimuth)
m.AccelerationSensorEnabled = true;% For Kalman Filter prediction
m.SampleRate = 100;                 % 25 Hz for smooth acceleration
pause(2);  
m.Logging = true;

disp('🛰️ Waiting for Phone GPS lock to set Origins... Walk outside!');
ped_origin_lat = NaN;
ped_origin_lon = NaN;
ped_origin_yaw_rad = NaN; % Locks the starting physical direction

% Origin Lock Variables for the Car
car_origin_x = NaN;
car_origin_y = NaN;
car_origin_yaw_rad = NaN;

% Initial Origin Lock Loop
while isnan(ped_origin_lat)
    [lat, lon, ~, ~, ~, ~, acc] = poslog(m); 
    [orient, ~] = orientlog(m);
    if ~isempty(lat) && ~isempty(orient) && acc(end) < 20 
        ped_origin_lat = lat(end);
        ped_origin_lon = lon(end);
        ped_origin_yaw_rad = deg2rad(-orient(end, 1)); 
        fprintf('✅ Pedestrian Origin & Yaw Locked! Initial Accuracy: %.2f meters\n', acc(end));
    end
    pause(0.5); 
end

% --- 5. ORIGINAL KALMAN FILTER SETUP ---
dt = 0.04; % 25 Hz loop pacing
x_est = [0; 0; 0; 0]; % State: [x, y, vx, vy]
P = eye(4); 
A = [1 0 dt 0; 0 1 0 dt; 0 0 0.85 0; 0 0 0 0.85]; 
B = [0.5*dt^2 0; 0 0.5*dt^2; dt 0; 0 dt];
C = [1 0 0 0; 0 1 0 0]; 
Q = eye(4) * 0.05;  % Original process noise
R = eye(2) * 2.0;   % Original measurement noise base
last_gps_time = -1;

% --- Reset State Variables ---
prev_car_reset = 0; % Tracks previous state to detect 0->1 edge
ped_reset_out = 0;  % The 4th bit we send to Simulink

% --- 6. INITIALIZE WORKSPACE FOR SIMULINK ---
assignin('base', 'Ped_X', ped_X_offset);
assignin('base', 'Ped_Y', ped_Y_offset);
assignin('base', 'Ped_Yaw', 0.0);
assignin('base', 'Car_X', car_X_offset);
assignin('base', 'Car_Y', car_Y_offset);
assignin('base', 'Car_Yaw', 0.0);
assignin('base', 'Car_Status', 0.0);

% --- 7. UI DASHBOARD (RESET BUTTON & LIVE PLOT) ---
fig_main = figure('Name', 'HIL Control Dashboard', 'Position', [100, 100, 800, 600], 'MenuBar', 'none');

setappdata(fig_main, 'TriggerReset', false);

uicontrol('Style', 'pushbutton', 'String', '🔄 RESET ORIGINS & YAW', ...
          'FontSize', 14, 'FontWeight', 'bold', 'Position', [20, 20, 250, 50], ...
          'Callback', @(src, event) setappdata(fig_main, 'TriggerReset', true));

uicontrol('Style', 'text', 'String', '(Close this window to Stop Tracking)', ...
          'FontSize', 10, 'Position', [280, 20, 250, 40], ...
          'HorizontalAlignment', 'left', 'BackgroundColor', get(fig_main, 'Color'));

ax = axes('Parent', fig_main, 'Position', [0.1, 0.2, 0.85, 0.75]);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
title(ax, 'Live Pedestrian Trajectory: Raw GPS vs Kalman Filter', 'FontWeight', 'bold', 'FontSize', 14);
xlabel(ax, 'Local X [meters]', 'FontWeight', 'bold'); 
ylabel(ax, 'Local Y [meters]', 'FontWeight', 'bold');

line_gps = animatedline(ax, 'Marker', 'o', 'Color', [0.8500 0.3250 0.0980], 'LineStyle', 'none', 'DisplayName', 'Raw GPS Points');
line_kf  = animatedline(ax, 'Color', [0.0 0.4470 0.7410], 'LineWidth', 2.5, 'DisplayName', 'Kalman Filter Path');
legend(ax, 'Location', 'best');

disp('======================================================');
disp('🔥 SYSTEM LIVE! Tracking Both Agents...');
disp('======================================================');

try
    while ishandle(fig_main) 
        
        % ---------------------------------------------------------
        % SYSTEM RESET LOGIC (Triggered by UI or Simulink bit)
        % ---------------------------------------------------------
        if getappdata(fig_main, 'TriggerReset') == true
            disp('🔄 Executing System Reset: Zeroing Kalman & Re-locking Origins...');
            
            ped_reset_out = 1; 
            
            x_est = [0; 0; 0; 0];
            P = eye(4);
            
            ped_origin_lat = NaN;
            ped_origin_yaw_rad = NaN;
            car_origin_x = NaN;
            
            clearpoints(line_gps);
            clearpoints(line_kf);
            
            setappdata(fig_main, 'TriggerReset', false);
        else
            ped_reset_out = 0; 
        end
        
        % ---------------------------------------------------------
        % AGENT 1: RECEIVE DATA & TRANSFORM CAR FRAME
        % ---------------------------------------------------------
        if u_recv.NumDatagramsAvailable > 0
            dgrams = read(u_recv, u_recv.NumDatagramsAvailable); 
            raw_bytes = dgrams(end).Data; 
            
            if length(raw_bytes) == 32
                parsed_doubles = swapbytes(typecast(uint8(raw_bytes), 'double'));
                
                raw_x     = parsed_doubles(1);
                raw_y     = parsed_doubles(2);
                raw_yaw   = parsed_doubles(3);
                car_reset = parsed_doubles(4);
                
                if car_reset == 1 && prev_car_reset == 0
                    setappdata(fig_main, 'TriggerReset', true);
                    disp('🔄 RESET TRIGGER RECEIVED FROM SIMULINK!');
                end
                prev_car_reset = car_reset;
                
                X_Fr = (raw_x / 100.0) - 10000.0;
                Y_Fr = (raw_y / 100.0) - 10000.0;
                Psi_Fr = (raw_yaw / 100.0)-180;
                
                if isnan(car_origin_x)
                    car_origin_x = X_Fr;
                    car_origin_y = Y_Fr;
                    car_origin_yaw_rad = deg2rad(Psi_Fr);
                    disp('✅ Car Origin Re-Locked!');
                end
                
                dx = X_Fr - car_origin_x;
                dy = Y_Fr - car_origin_y;
                
                carx =  (dx * cos(car_origin_yaw_rad) + dy * sin(car_origin_yaw_rad)) + car_X_offset;
                cary = (-dx * sin(car_origin_yaw_rad) + dy * cos(car_origin_yaw_rad)) + car_Y_offset;
                caryaw = -(Psi_Fr - rad2deg(car_origin_yaw_rad));
                
                assignin('base', 'Car_X', carx);
                assignin('base', 'Car_Y', cary);
                assignin('base', 'Car_Yaw', caryaw);
                assignin('base', 'Car_Status', car_reset);

                                % Display the received values to the command window
                fprintf('🔽 [CAR IN]  X: %8.2f | Y: %8.2f | Yaw: %8.2f | Reset: %1.0f\n', ...
                        carx, cary, caryaw, car_reset);
            end
        end
        
        % ---------------------------------------------------------
        % AGENT 2: ORIGINAL KALMAN FILTER & SEND
        % ---------------------------------------------------------
        [lat, lon, t_gps, ~, ~, ~, gps_acc] = poslog(m);
        [accel, ~] = accellog(m);
        [orient, ~] = orientlog(m); 
        
        if ~isempty(accel) && ~isempty(orient) && ~isempty(lat)
            
            raw_yaw_rad = deg2rad(-orient(end, 1));
            
            if isnan(ped_origin_lat)
                ped_origin_lat = lat(end);
                ped_origin_lon = lon(end);
                ped_origin_yaw_rad = raw_yaw_rad;
                disp('✅ Pedestrian Origin & Yaw Re-Locked!');
            end
            
            ped_yaw_rad = raw_yaw_rad - ped_origin_yaw_rad;
            ped_yaw_deg = rad2deg(ped_yaw_rad);
            
            raw_a = accel(end, 2); 
            if abs(raw_a) < 1.0, raw_a = 0; end 
            a_forward = raw_a * 0.1; 
            
            u_kalman = [a_forward * cos(ped_yaw_rad); a_forward * sin(ped_yaw_rad)];
            x_est = A * x_est + B * u_kalman;
            P = A * P * A' + Q;
            
            if t_gps(end) ~= last_gps_time
                last_gps_time = t_gps(end);
                
                x_gps = (lon(end) - ped_origin_lon) * cos(ped_origin_lat * pi/180) * 111320;
                y_gps = (lat(end) - ped_origin_lat) * 110540;
                
                rotated_x_gps = x_gps * cos(-ped_origin_yaw_rad) - y_gps * sin(-ped_origin_yaw_rad);
                rotated_y_gps = x_gps * sin(-ped_origin_yaw_rad) + y_gps * cos(-ped_origin_yaw_rad);
                
                addpoints(line_gps, rotated_x_gps + ped_X_offset, rotated_y_gps + ped_Y_offset);
                
                % --- ORIGINAL UPDATE LOGIC RESTORED HERE ---
                R_dynamic = R * (gps_acc(end) / 5); 
                K = P * C' / (C * P * C' + R_dynamic); 
                x_est = x_est + K * ([rotated_x_gps; rotated_y_gps] - C * x_est); 
                P = (eye(4) - K * C) * P; 
            end
            
            final_ped_X = x_est(1) + ped_X_offset;
            final_ped_Y = x_est(2) + ped_Y_offset;
            
            addpoints(line_kf, final_ped_X, final_ped_Y);
            
            assignin('base', 'Ped_X', final_ped_X);
            assignin('base', 'Ped_Y', final_ped_Y);
            assignin('base', 'Ped_Yaw', ped_yaw_deg); 
            
            reply = double([final_ped_X, final_ped_Y, ped_yaw_deg, ped_reset_out]);
            write(u_send, reply, "double", simulinkIP, simulinkPort);
            
            fprintf('🔼 [PED OUT] X: %8.2f | Y: %8.2f | Rel_Yaw: %8.2f° | Res: %d\n', final_ped_X, final_ped_Y, ped_yaw_deg, ped_reset_out);
        end
        
        drawnow limitrate;
        pause(dt); 
    end

    % --- Normal Cleanup ---
    m.Logging = false; 
    clear u_recv u_send;
    disp('Tracking stopped normally.');

catch ME
    % --- Error Handling ---
    m.Logging = false;
    clear u_recv u_send;
    fprintf('\n🛑 Communication stopped and ports cleared.\n');
    rethrow(ME);
end
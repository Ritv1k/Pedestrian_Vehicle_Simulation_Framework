% === MASTER HIL CONTROLLER: KALMAN PEDESTRIAN (OUT) + CAR DATA (IN) ===
clear; close all; clc;

% --- 1. GLOBAL SETTINGS & OFFSETS ---
car_X_offset = 15.0;
car_Y_offset = 15.0;
ped_X_offset = 20;   
ped_Y_offset = 20;   

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
m.PositionSensorEnabled = true;    % For Lat and Lon
m.OrientationSensorEnabled = true; % For Absolute Yaw (Azimuth)
m.AccelerationSensorEnabled = true;% For Kalman Filter prediction
m.SampleRate = 25;                 % 25 Hz for smooth acceleration
pause(2);
m.Logging = true;

disp('🛰️ Waiting for Phone GPS lock to set Origin... Walk outside!');
ped_origin_lat = NaN;
ped_origin_lon = NaN;

% Origin Lock Variables for the Car
car_origin_x = NaN;
car_origin_y = NaN;
car_origin_yaw_rad = NaN;

% Origin Lock Loop (Comment out the while loop if testing indoors without GPS)
while isnan(ped_origin_lat)
    [lat, lon, ~, ~, ~, ~, acc] = poslog(m); 
    if ~isempty(lat) && acc(end) < 20 
        ped_origin_lat = lat(end);
        ped_origin_lon = lon(end);
        fprintf('✅ Pedestrian Origin Locked! Accuracy: %.2f meters\n', acc(end));
    end
    pause(0.5);  
end

% --- 5. KALMAN FILTER SETUP ---
dt = 0.04; % 25 Hz loop pacing
x_est = [0; 0; 0; 0]; % State: [x, y, vx, vy]
P = eye(4); 
A = [1 0 dt 0; 0 1 0 dt; 0 0 0.85 0; 0 0 0 0.85]; 
B = [0.5*dt^2 0; 0 0.5*dt^2; dt 0; 0 dt];
C = [1 0 0 0; 0 1 0 0]; 
Q = eye(4) * 0.05;  
R = eye(2) * 2.0;   
last_gps_time = -1;

% --- 6. INITIALIZE WORKSPACE FOR SIMULINK ---
assignin('base', 'Ped_X', ped_X_offset);
assignin('base', 'Ped_Y', ped_Y_offset);
assignin('base', 'Ped_Yaw', 0.0);
assignin('base', 'Car_X', car_X_offset);
assignin('base', 'Car_Y', car_Y_offset);
assignin('base', 'Car_Yaw', 0.0);
assignin('base', 'Car_Status', 0.0);

% --- 7. UI STOP BUTTON ---
stop_fig = figure('Name', 'Master HIL Control', 'Position', [100, 100, 300, 100], 'MenuBar', 'none', 'ToolBar', 'none');
uicontrol('Style', 'pushbutton', 'String', '🛑 STOP TRACKING', ...
          'FontSize', 12, 'FontWeight', 'bold', 'Position', [25, 20, 250, 60], ...
          'Callback', 'delete(gcf)');

disp('======================================================');
disp('🔥 SYSTEM LIVE! Tracking Both Agents...');
disp('======================================================');

try
    while ishandle(stop_fig) 
        
        % ---------------------------------------------------------
        % AGENT 1: RECEIVE DATA & TRANSFORM CAR FRAME
        % ---------------------------------------------------------
        if u_recv.NumDatagramsAvailable > 0
            
            % 1. Read all datagrams to clear the backlog
            dgrams = read(u_recv, u_recv.NumDatagramsAvailable); 
            
            % 2. Open the "envelope" of the newest packet to get the raw bytes
            raw_bytes = dgrams(end).Data; 
            
            % 3. Check that the packet contains exactly 32 bytes (4 doubles * 8 bytes)
            if length(raw_bytes) == 32
                % Typecast bytes to double array, and swap bytes from Big-Endian
                parsed_doubles = swapbytes(typecast(uint8(raw_bytes), 'double'));
                
                % Extract the raw incoming values
                raw_x     = parsed_doubles(1);
                raw_y     = parsed_doubles(2);
                raw_yaw   = parsed_doubles(3);
                car_reset = parsed_doubles(4);
                
                % --- APPLY FIXED-POINT DECODING ---
                X_Fr = (raw_x / 100.0) - 10000.0;
                Y_Fr = (raw_y / 100.0) - 10000.0;
                Psi_Fr = (raw_yaw / 100.0)-180;
                
                % --- UNREAL ENGINE FRAME CONVERSION & ORIGIN LOCK ---
                % Lock the Car Origin on the very first received packet
                if isnan(car_origin_x)
                    car_origin_x = X_Fr;
                    car_origin_y = Y_Fr;
                    car_origin_yaw_rad = deg2rad(Psi_Fr);
                    disp('✅ Car Origin Locked! Translating & Rotating path.');
                end
                
                % Step A: Find distance from the very first point
                dx = X_Fr - car_origin_x;
                dy = Y_Fr - car_origin_y;
                
                % Step B: Rotate to +X Forward, AND add the 15.0 offset
                carx =  (dx * cos(car_origin_yaw_rad) + dy * sin(car_origin_yaw_rad)) + car_X_offset;
                cary = (-dx * sin(car_origin_yaw_rad) + dy * cos(car_origin_yaw_rad)) + car_Y_offset;
                
                % Step C: Calculate relative Yaw (Counter-clockwise)
                caryaw = -(Psi_Fr - rad2deg(car_origin_yaw_rad));
                
                % Push to Simulink base workspace
                assignin('base', 'Car_X', carx);
                assignin('base', 'Car_Y', cary);
                assignin('base', 'Car_Yaw', caryaw);
                assignin('base', 'Car_Status', car_reset);
                
                % Display the received values to the command window
                fprintf('🔽 [CAR IN]  X: %8.2f | Y: %8.2f | Yaw: %8.2f | Reset: %1.0f\n', ...
                        carx, cary, caryaw, car_reset);
            else
                disp("⚠️ Buffer flushed: Packet didn't contain exactly 32 bytes (4 doubles).");
            end
        end
        
        % ---------------------------------------------------------
        % AGENT 2: PEDESTRIAN KALMAN FILTER & SEND
        % ---------------------------------------------------------
        [lat, lon, t_gps, ~, ~, ~, gps_acc] = poslog(m);
        [accel, ~] = accellog(m);
        [orient, ~] = orientlog(m); 
        
        if ~isempty(accel) && ~isempty(orient) && ~isempty(lat)
            
            % 1. Get Yaw from Compass
            ped_yaw_deg = -orient(end, 1); 
            ped_yaw_rad = deg2rad(ped_yaw_deg);
            
            % 2. Get Forward Acceleration (Assuming Y-axis is forward on phone)
            raw_a = accel(end, 2); 
            if abs(raw_a) < 1.0, raw_a = 0; end 
            a_forward = raw_a * 0.1; 
            
            % 3. Kalman Prediction Step
            u_kalman = [a_forward * cos(ped_yaw_rad); a_forward * sin(ped_yaw_rad)];
            x_est = A * x_est + B * u_kalman;
            P = A * P * A' + Q;
            
            % 4. Kalman Update Step (When new GPS data arrives)
            if t_gps(end) ~= last_gps_time
                last_gps_time = t_gps(end);
                
                % Convert Lat/Lon to Local X/Y Meters relative to Origin Lock
                x_gps = (lon(end) - ped_origin_lon) * cos(ped_origin_lat * pi/180) * 111320;
                y_gps = (lat(end) - ped_origin_lat) * 110540;
                
                R_dynamic = R * (gps_acc(end) / 5); 
                K = P * C' / (C * P * C' + R_dynamic); 
                x_est = x_est + K * ([x_gps; y_gps] - C * x_est); 
                P = (eye(4) - K * C) * P; 
            end
            
            % 5. Apply the Pedestrian Offsets
            final_ped_X = x_est(1) + ped_X_offset;
            final_ped_Y = x_est(2) + ped_Y_offset;
            
            % Push to Simulink base workspace
            assignin('base', 'Ped_X', final_ped_X);
            assignin('base', 'Ped_Y', final_ped_Y);
            assignin('base', 'Ped_Yaw', ped_yaw_deg); 
            
            % 6. Send 3 doubles out over UDP (X, Y, Yaw)
            reply = double([final_ped_X, final_ped_Y, ped_yaw_deg]);
            write(u_send, reply, "double", simulinkIP, simulinkPort);
            
            % Display the sent values to the command window
            fprintf('🔼 [PED OUT] X: %8.2f | Y: %8.2f | Yaw: %8.2f\n', ...
                    final_ped_X, final_ped_Y, ped_yaw_deg);
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
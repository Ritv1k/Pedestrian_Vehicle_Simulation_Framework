% === MASTER HIL CONTROLLER: DEAD-RECKONING PEDESTRIAN + CAR DATA (IN) ===
clear; close all; clc;

% --- 1. GLOBAL SETTINGS & OFFSETS ---
car_X_offset = 15.0;
car_Y_offset = 15.0;
ped_X_offset = 25.0;   
ped_Y_offset = 25.0;   

% --- 2. Port Routing ---
matlabListenPort = 5000;      % MATLAB listens here (receiving Car data)
simulinkIP = "192.168.0.127"; % MATLAB sends to this IP (sending Pedestrian data)
simulinkPort = 8002;          % MATLAB sends to this Port

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
m.PositionSensorEnabled = true;    % For initial Origin Lock only
m.OrientationSensorEnabled = true; % For Absolute Yaw (Azimuth)
m.AccelerationSensorEnabled = true;% To detect stepping/walking
m.SampleRate = 100;                 %
pause(2);
m.Logging = true;

disp('🛰️ Waiting for Phone GPS lock to set Origin... Walk outside!');
ped_origin_lat = NaN;

% Origin Lock Loop (Comment out the while loop if testing indoors)
while isnan(ped_origin_lat)
    [lat, ~, ~, ~, ~, ~, acc] = poslog(m); 
    if ~isempty(lat) && acc(end) < 20 
        ped_origin_lat = lat(end);
        fprintf('✅ Pedestrian Origin Locked! Switching to smooth dead-reckoning.\n');
    end
    pause(0.5); 
end

% --- 5. DEAD RECKONING SETUP ---
dt = 0.04;          % 25 Hz loop pacing
ped_local_x = 0.0;  % Starting local X
ped_local_y = 0.0;  % Starting local Y
walk_speed = .5;   % Fixed walking speed in m/s (~3.1 mph)
accel_threshold = 0.8; % m/s^2 variance to detect a footstep/movement

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
        % AGENT 1: RECEIVE DATA (4 Doubles from Simulink)
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
                
                assignin('base', 'Car_X', carx);
                assignin('base', 'Car_Y', cary);
                assignin('base', 'Car_Yaw', caryaw);
                assignin('base', 'Car_Status', car_reset);
                
                fprintf('🔽 [CAR IN]  X: %8.2f | Y: %8.2f | Yaw: %8.2f | Reset: %1.0f\n', ...
                        carx, cary, caryaw, car_reset);
            else
                disp("⚠️ Buffer flushed: Packet didn't contain exactly 32 bytes.");
            end
        end
        
        % ---------------------------------------------------------
        % AGENT 2: PEDESTRIAN DEAD-RECKONING & SEND
        % ---------------------------------------------------------
        [accel, ~] = accellog(m);
        [orient, ~] = orientlog(m); 
        
        if ~isempty(accel) && ~isempty(orient)
            
            % 1. Get Absolute Yaw from Compass
            ped_yaw_deg = -orient(end, 1); 
            
            % Convert to Unreal Engine coordinate orientation if needed
            % Depending on UE setup, you might need to flip the sign: ped_yaw_rad = deg2rad(-ped_yaw_deg);
            ped_yaw_rad = deg2rad(ped_yaw_deg);
            
            % 2. Detect Movement via Accelerometer Magnitude (ignores gravity tilt)
            % Taking the lateral and longitudinal movement of the phone
            accel_mag = sqrt(accel(end, 1)^2 + accel(end, 2)^2);
            
            % 3. Apply Speed
            if accel_mag > accel_threshold
                current_speed = walk_speed;
            else
                current_speed = 0.0; % Standing still
            end
            
            % 4. Dead Reckoning Integration (Position = Speed * Time * Direction)
            ped_local_x = ped_local_x + (current_speed * cos(ped_yaw_rad) * dt);
            ped_local_y = ped_local_y + (current_speed * sin(ped_yaw_rad) * dt);
            
            % 5. Apply the 15.0 Offset for Simulink
            final_ped_X = ped_local_x + ped_X_offset;
            final_ped_Y = ped_local_y + ped_Y_offset;
            
            % Push to Simulink base workspace
            assignin('base', 'Ped_X', final_ped_X);
            assignin('base', 'Ped_Y', final_ped_Y);
            assignin('base', 'Ped_Yaw', ped_yaw_deg); 
            
            % 6. Send 3 doubles out over UDP (X, Y, Yaw)
            reply = double([final_ped_X, final_ped_Y, ped_yaw_deg]);
            write(u_send, reply, "double", simulinkIP, simulinkPort);
            
            % Display the sent values
            fprintf('🔼 [PED OUT] X: %8.2f | Y: %8.2f | Yaw: %8.2f | Speed: %3.1f m/s\n', ...
                    final_ped_X, final_ped_Y, ped_yaw_deg, current_speed);
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
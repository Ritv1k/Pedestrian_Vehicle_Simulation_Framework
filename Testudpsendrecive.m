clear; close all; clc;

% --- 1. Port Routing ---
matlabListenPort = 5000;      % MATLAB listens here (receiving Car data)
simulinkIP = "192.168.1.77"; % MATLAB sends to this IP (sending Pedestrian data)
simulinkPort = 20009;          % MATLAB sends to this Port

% --- 2. Create UDP Objects ---
u_recv = udpport("datagram", "IPv4", "LocalPort", matlabListenPort);
u_recv.ByteOrder = "big-endian"; 
u_send = udpport("datagram", "IPv4");
u_send.ByteOrder = "big-endian";
flush(u_recv);

% --- 3. Setup Phone Sensors ---
disp('📱 Connecting to Mobile Device...');
m = mobiledev;
m.PositionSensorEnabled = true;    % For Lat and Lon
m.OrientationSensorEnabled = true; % For Yaw (Azimuth)
m.SampleRate = 10;                 % 10 Hz update rate
pause(2);
m.Logging = true;

disp(['📡 MATLAB listening on port ', num2str(matlabListenPort), '...']);
disp("Waiting for 4 doubles (Car) and sending 3 doubles (Pedestrian). Press Ctrl+C to stop.");

try
    while true
        % ---------------------------------------------------------
        % A. RECEIVE DATA (4 Doubles: X, Y, Yaw, Reset)
        % ---------------------------------------------------------
        if u_recv.NumDatagramsAvailable > 0
            
            % Read exactly 4 doubles (32 bytes)
            data = read(u_recv, 4, "double"); 
            
            if length(data) == 4
                car_x     = data(1);
                car_y     = data(2);
                car_yaw   = data(3);
                car_reset = data(4);
                
                % Print to command window
                disp(car_x);
                disp(car_y);
                disp(car_yaw);
                disp(car_reset);

            else
                flush(u_recv); 
                disp("⚠️ Buffer flushed: Unexpected packet size.");
            end
        else
            % Optional: Print when waiting for data
            % disp('⏳ No data received from Car...'); 
        end
        
        % ---------------------------------------------------------
        % B. READ PHONE SENSORS & SEND (3 Doubles: Lat, Lon, Yaw)
        % ---------------------------------------------------------
        % Pull latest logs from the phone
        [lat, lon, ~, ~, ~, ~, ~] = poslog(m);
        [orient, ~] = orientlog(m); 
        
        % Only send if the phone has successfully started logging data
        if ~isempty(lat) && ~isempty(orient)
            
            % Extract the most recent values
            ped_lat = lat(end);
            ped_lon = lon(end);
            ped_yaw = orient(end, 1); % The 1st column of orientation is Azimuth (Yaw)
            
            % Package the 3 doubles
            reply = [ped_lat, ped_lon, ped_yaw];
            
            % Send 3 doubles out over UDP
            write(u_send, reply, "double", simulinkIP, simulinkPort);
            
            % Print to command window
            fprintf('🔼 [PED OUT] Lat: %9.5f | Lon: %9.5f | Yaw: %8.2f\n', ...
                    ped_lat, ped_lon, ped_yaw);
        end
        
        % Pace the loop (~20 Hz) to prevent MATLAB from freezing
        pause(0.05); 
    end
    
catch ME
    % --- Clean up safely if you press Ctrl+C or an error occurs ---
    m.Logging = false;
    clear u_recv u_send;
    fprintf('\n🛑 Communication stopped and ports cleared.\n');
    rethrow(ME);
end
% === MASTER HIL CONTROLLER: TIMER-BASED SMOOTH WALK (NO STUTTER) ===
clear; close all; clc;

% --- EMERGENCY CLEANUP ---
delete(timerfindall); 

% --- 1. GLOBAL SETTINGS & OFFSETS ---
car_X_offset = 15.0;
car_Y_offset = 15.0;
ped_X_offset = 20.0;   % UPDATED to 20
ped_Y_offset = 20.0;   % UPDATED to 20

% --- 2. Port Routing --
matlabListenPort = 5000;     
simulinkIP = "192.168.1.77"; 
simulinkPort = 20009;        

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
m.PositionSensorEnabled = false;       
m.AccelerationSensorEnabled = false;   
m.OrientationSensorEnabled = true;     
m.AngularVelocitySensorEnabled = true; 
m.SampleRate = 100;                 
pause(2);
m.Logging = true;

disp('🧭 Waiting for Phone Compass lock to set initial orientation...');
ped_origin_yaw_rad = NaN; 
while isnan(ped_origin_yaw_rad)
    [orient, ~] = orientlog(m); 
    if ~isempty(orient)
        ped_origin_yaw_rad = deg2rad(-orient(end, 1));
        disp('✅ Initial Yaw Locked! You are now facing Forward (0°).');
    end
    pause(0.5); 
end

% --- 5. INITIALIZE WORKSPACE FOR SIMULINK ---
assignin('base', 'Ped_X', ped_X_offset);
assignin('base', 'Ped_Y', ped_Y_offset);
assignin('base', 'Ped_Yaw', 0.0);
assignin('base', 'Ped_Speed', 0.0);
assignin('base', 'Ped_YawRate', 0.0);
assignin('base', 'Car_X', car_X_offset);
assignin('base', 'Car_Y', car_Y_offset);
assignin('base', 'Car_Yaw', 0.0);
assignin('base', 'Car_Status', 0.0);

% --- 6. UI DASHBOARD ---
fig_main = figure('Name', 'Smooth Timer Walk Dashboard', 'Position', [100, 100, 1000, 500], 'MenuBar', 'none');
setappdata(fig_main, 'TriggerReset', false);

uicontrol('Style', 'pushbutton', 'String', '🔄 RESET ORIGINS & YAW', ...
          'FontSize', 12, 'FontWeight', 'bold', 'Position', [20, 20, 250, 50], ...
          'Callback', @(src, event) setappdata(fig_main, 'TriggerReset', true));
uicontrol('Style', 'text', 'String', '(Close this window to Stop Tracking)', ...
          'FontSize', 10, 'Position', [280, 20, 250, 40], ...
          'HorizontalAlignment', 'left', 'BackgroundColor', get(fig_main, 'Color'));

ax1 = subplot(1, 2, 1, 'Parent', fig_main);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
title(ax1, 'Live Trajectory', 'FontWeight', 'bold', 'FontSize', 12);
xlabel(ax1, 'Local X [m]', 'FontWeight', 'bold'); 
ylabel(ax1, 'Local Y [m]', 'FontWeight', 'bold');
line_traj = animatedline(ax1, 'Color', [0.0 0.4470 0.7410], 'LineWidth', 2.5);
plot(ax1, ped_X_offset, ped_Y_offset, 'g^', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

ax2 = subplot(1, 2, 2, 'Parent', fig_main);
hold(ax2, 'on'); grid(ax2, 'on');
title(ax2, 'Live Relative Yaw Angle', 'FontWeight', 'bold', 'FontSize', 12);
xlabel(ax2, 'Time [s]', 'FontWeight', 'bold'); 
ylabel(ax2, 'Heading (Degrees)', 'FontWeight', 'bold');
yline(ax2, 0, 'k-', 'Forward (0°)', 'LabelHorizontalAlignment', 'left');
line_yaw = animatedline(ax2, 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 2);
xlim(ax2, [0, 10]);

% --- 7. SETUP STATE VARIABLES FOR TIMER ---
state = struct();
state.ped_local_x = 0.0;
state.ped_local_y = 0.0;
state.ped_origin_yaw_rad = ped_origin_yaw_rad;
state.t_elapsed = 0;
state.prev_car_reset = 0;
state.ped_reset_out = 0;
state.last_tick = tic;
state.ui_tick = tic;
state.walk_speed = 1.4; % 1.4 m/s continuous

% --- 8. START BACKGROUND TIMER (50 Hz / 0.02s Period) ---
% Explicitly passing ax1 and ax2 to prevent "invalid graphics object" errors
t = timer('ExecutionMode', 'fixedRate', 'Period', 0.02, ... 
          'TimerFcn', {@hil_physics_step, m, u_recv, u_send, simulinkIP, simulinkPort, fig_main, ax1, ax2, line_traj, line_yaw, car_X_offset, car_Y_offset, ped_X_offset, ped_Y_offset});
t.UserData = state;

% Bind the safety shutdown to the Window Close button
set(fig_main, 'CloseRequestFcn', @(src, event) shutdown_system(src, t, m, u_recv, u_send));

disp('======================================================');
disp('🔥 SYSTEM LIVE! Background Physics Timer Active (50Hz)');
disp('======================================================');
start(t);


% =========================================================================
% BACKGROUND PHYSICS FUNCTION (Runs strictly every 0.02 seconds)
% =========================================================================
function hil_physics_step(obj, ~, m, u_recv, u_send, simulinkIP, simulinkPort, fig_main, ax1, ax2, line_traj, line_yaw, car_X_offset, car_Y_offset, ped_X_offset, ped_Y_offset)
    
    % Safety check: If UI is closed abruptly, do nothing
    if ~isgraphics(fig_main)
        return;
    end
    
    state = obj.UserData;
    actual_dt = toc(state.last_tick);
    state.last_tick = tic;
    state.t_elapsed = state.t_elapsed + actual_dt;
    
    % --- 1. SYSTEM RESET LOGIC ---
    if getappdata(fig_main, 'TriggerReset') == true
        state.ped_reset_out = 1; 
        state.ped_local_x = 0.0;
        state.ped_local_y = 0.0;
        state.ped_origin_yaw_rad = NaN; 
        state.t_elapsed = 0; 
        
        if isgraphics(line_traj) && isgraphics(line_yaw)
            clearpoints(line_traj);
            clearpoints(line_yaw);
        end
        setappdata(fig_main, 'TriggerReset', false);
    else
        state.ped_reset_out = 0; 
    end
    
    % --- 2. RECEIVE CAR DATA ---
    if u_recv.NumDatagramsAvailable > 0
        dgrams = read(u_recv, u_recv.NumDatagramsAvailable); 
        raw_bytes = dgrams(end).Data; 
        
        if length(raw_bytes) == 32
            parsed_doubles = swapbytes(typecast(uint8(raw_bytes), 'double'));
            carx      = parsed_doubles(1) + car_X_offset;
            cary      = parsed_doubles(2) + car_Y_offset;
            caryaw    = parsed_doubles(3); 
            car_reset = parsed_doubles(4); 
            
            if car_reset == 1 && state.prev_car_reset == 0
                setappdata(fig_main, 'TriggerReset', true);
            end
            state.prev_car_reset = car_reset;
            
            assignin('base', 'Car_X', carx);
            assignin('base', 'Car_Y', cary);
            assignin('base', 'Car_Yaw', caryaw);
            assignin('base', 'Car_Status', car_reset);
        end
    end
    
    % --- 3. PEDESTRIAN KINEMATICS ---
    [orient, ~] = orientlog(m); 
    [angvel, ~] = angvellog(m); 
    
    if ~isempty(orient) && ~isempty(angvel)
        
        % Relative Heading
        raw_yaw_rad = deg2rad(-orient(end, 1));
        if isnan(state.ped_origin_yaw_rad)
            state.ped_origin_yaw_rad = raw_yaw_rad;
        end
        ped_yaw_rad = raw_yaw_rad - state.ped_origin_yaw_rad;
        ped_yaw_deg = rad2deg(ped_yaw_rad);
        
        % Smooth Angular Velocity (YawRate) from Z-axis gyro
        yaw_rate_rad = -angvel(end, 3);
        yaw_rate_deg = rad2deg(yaw_rate_rad);
        
        % True-Time Integration
        state.ped_local_x = state.ped_local_x + (state.walk_speed * cos(ped_yaw_rad) * actual_dt);
        state.ped_local_y = state.ped_local_y + (state.walk_speed * sin(ped_yaw_rad) * actual_dt);
        
        final_ped_X = state.ped_local_x + ped_X_offset;
        final_ped_Y = state.ped_local_y + ped_Y_offset;
        
        % Push to Base Workspace
        assignin('base', 'Ped_X', final_ped_X);
        assignin('base', 'Ped_Y', final_ped_Y);
        assignin('base', 'Ped_Yaw', ped_yaw_deg); 
        assignin('base', 'Ped_Speed', state.walk_speed);
        assignin('base', 'Ped_YawRate', yaw_rate_deg);
        
        % Send UDP (Size 6)
        reply = double([final_ped_X, final_ped_Y, ped_yaw_deg, state.walk_speed, yaw_rate_deg, state.ped_reset_out]);
        write(u_send, reply, "double", simulinkIP, simulinkPort);
        
        % --- 4. DECOUPLED UI UPDATES (Updates at 10Hz to prevent lag) ---
        if toc(state.ui_tick) > 0.1
            % Double check graphics exist before drawing to prevent timer crashes
            if isgraphics(line_yaw) && isgraphics(ax2) && isgraphics(line_traj)
                addpoints(line_yaw, state.t_elapsed, ped_yaw_deg);
                xlim(ax2, [0, max(state.t_elapsed, 10)]); 
                addpoints(line_traj, final_ped_X, final_ped_Y);
                
                fprintf('🔼 [PED OUT] X: %8.2f | Y: %8.2f | Yaw: %8.2f° | Res: %d\n', ...
                        final_ped_X, final_ped_Y, ped_yaw_deg, state.ped_reset_out);
                
                drawnow limitrate;
            end
            state.ui_tick = tic;
        end
    end
    
    % Save state back to timer object
    obj.UserData = state;
end

% =========================================================================
% SAFETY SHUTDOWN FUNCTION
% =========================================================================
function shutdown_system(fig, t, m, u_recv, u_send)
    disp('🛑 Shutting down safely...');
    try stop(t); catch; end
    try delete(t); catch; end
    m.Logging = false;
    clear u_recv u_send m;
    delete(fig);
    disp('✅ Background Timer killed. Ports cleared.');
end
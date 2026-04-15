
function startPhoneGPS_Timer()
% =========================================================
% REAL-TIME PHONE GPS → MATLAB → SIMULINK / UNREAL
% Outputs: pedX, pedY, pedYaw
% Origin fixed at (10,10)
% =========================================================

    clc;

    % -----------------------------
    % Connect to phone
    % -----------------------------
    m = mobiledev;
    m.PositionSensorEnabled    = true;
    m.OrientationSensorEnabled = true;
    m.Logging = true;

    pause(1);
    disp("✅ Phone connected");

    % -----------------------------
    % Projection: UTM Zone 17N (Ohio)
    % -----------------------------
    p = projcrs(32617);

    % -----------------------------
    % Store shared objects
    % -----------------------------
    assignin('base','phone_m',m);
    assignin('base','proj_p',p);

    % First GPS fix (reference)
    assignin('base','x0',[]);
    assignin('base','y0',[]);

    % Scene origin offset (SAFE SPAWN)
    assignin('base','X_offset',10.0);
    assignin('base','Y_offset',10.0);

    % Previous state (rate limiting)
    assignin('base','X_prev',10.0);
    assignin('base','Y_prev',10.0);

    % Outputs (MATCH YOUR SIMULINK MODEL)
    assignin('base','pedX',10.0);
    assignin('base','pedY',10.0);
    assignin('base','pedYaw',0.0);

    % -----------------------------
    % Background timer (20 Hz)
    % -----------------------------
    tmr = timer( ...
        'ExecutionMode','fixedSpacing', ...
        'Period',0.05, ...
        'TimerFcn',@phoneStep);

    assignin('base','phoneTimer',tmr);
    start(tmr);

    disp("🚀 Phone GPS timer started (outputs: pedX, pedY, pedYaw)");
end

% =========================================================
% TIMER CALLBACK
% =========================================================
function phoneStep(~,~)
    try
        % -----------------------------
        % Load shared data
        % -----------------------------
        m        = evalin('base','phone_m');
        p        = evalin('base','proj_p');
        x0       = evalin('base','x0');
        y0       = evalin('base','y0');
        X_prev   = evalin('base','X_prev');
        Y_prev   = evalin('base','Y_prev');
        X_offset = evalin('base','X_offset');
        Y_offset = evalin('base','Y_offset');

        lat = m.Latitude;
        lon = m.Longitude;
        ori = m.Orientation;   % [yaw pitch roll] in degrees

        if isempty(lat) || isempty(lon) || isempty(ori)
            return;
        end

        % -----------------------------
        % GPS → UTM (meters)
        % -----------------------------
        [xEast, yNorth] = projfwd(p, lat, lon);

        % -----------------------------
        % Set GPS origin on first fix
        % -----------------------------
        if isempty(x0)
            assignin('base','x0',xEast);
            assignin('base','y0',yNorth);
            return;
        end

        % -----------------------------
        % Local coordinates
        % -----------------------------
        X_raw =  (yNorth - y0);      % forward
        Y_raw = -(xEast  - x0);      % left

        % -----------------------------
        % Shift origin to (10,10)
        % -----------------------------
        X_raw = X_raw + X_offset;
        Y_raw = Y_raw + Y_offset;

        % -----------------------------
        % SAFETY 1: Scene bounds
        % -----------------------------
        maxRange = 50;  % meters
        X_raw = max(min(X_raw, maxRange), -maxRange);
        Y_raw = max(min(Y_raw, maxRange), -maxRange);

        % -----------------------------
        % SAFETY 2: Rate limiting
        % -----------------------------
        maxStep = 0.5;  % meters per step (20 Hz)

        dX = X_raw - X_prev;
        dY = Y_raw - Y_prev;

        dX = max(min(dX, maxStep), -maxStep);
        dY = max(min(dY, maxStep), -maxStep);

        X = X_prev + dX;
        Y = Y_prev + dY;

        % -----------------------------
        % Orientation
        % -----------------------------
        pedYaw = ori(1);   % degrees (yaw)

        % -----------------------------
        % Publish outputs (SIMULINK READS THESE)
        % -----------------------------
        assignin('base','pedX',X);
        assignin('base','pedY',Y);
        assignin('base','pedYaw',pedYaw);

        assignin('base','X_prev',X);
        assignin('base','Y_prev',Y);

    catch
        % Never crash the timer
    end
end

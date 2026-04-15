classdef PhoneGPSSystem < matlab.System
    % PhoneGPSSystem
    % Live phone GPS/orientation -> scene-safe pedX/pedY/pedYaw
    % Uses UTM 17N (Ohio) and anchors first fix to (10,10).

    properties
        SampleTime = 0.05;   % Simulink step (s)
        OriginX = 10;        % scene spawn X (m)
        OriginY = 10;        % scene spawn Y (m)
        MaxRange = 50;       % clamp absolute position (m)
        MaxStep  = 0.5;      % rate limit per step (m)
    end

    properties(Access = private)
        m          % mobiledev handle
        p          % projcrs handle
        x0         % first-fix easting
        y0         % first-fix northing
        xPrev
        yPrev
        initialized logical
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Connect to phone
            obj.m = mobiledev;
            obj.m.PositionSensorEnabled    = true;
            obj.m.OrientationSensorEnabled = true;
            obj.m.Logging = false;  % IMPORTANT: reduce load/disconnect risk

            pause(1);

            % Projection: UTM Zone 17N (Ohio)
            obj.p = projcrs(32617);

            % Init state
            obj.x0 = [];
            obj.y0 = [];
            obj.xPrev = obj.OriginX;
            obj.yPrev = obj.OriginY;
            obj.initialized = true;
        end

        function [pedX, pedY, pedYaw] = stepImpl(obj)
            % Defaults (hold last good)
            pedX = obj.xPrev;
            pedY = obj.yPrev;
            pedYaw = 0.0;

            if ~obj.initialized
                return;
            end

            lat = obj.m.Latitude;
            lon = obj.m.Longitude;
            ori = obj.m.Orientation; % [yaw pitch roll] deg

            if isempty(lat) || isempty(lon) || isempty(ori)
                % no new data -> hold last
                pedYaw = 0.0;
                return;
            end

            % GPS -> UTM meters
            [xEast, yNorth] = projfwd(obj.p, lat, lon);

            % First fix establishes local reference
            if isempty(obj.x0)
                obj.x0 = xEast;
                obj.y0 = yNorth;
                pedYaw = ori(1);
                return;
            end

            % Local coordinates (forward/left convention you used)
            X_raw =  (yNorth - obj.y0);     % forward
            Y_raw = -(xEast  - obj.x0);     % left

            % Shift origin to safe spawn (10,10)
            X_raw = X_raw + obj.OriginX;
            Y_raw = Y_raw + obj.OriginY;

            % Clamp absolute position
            X_raw = max(min(X_raw, obj.MaxRange), -obj.MaxRange);
            Y_raw = max(min(Y_raw, obj.MaxRange), -obj.MaxRange);

            % Rate-limit per step
            dX = X_raw - obj.xPrev;
            dY = Y_raw - obj.yPrev;

            dX = max(min(dX, obj.MaxStep), -obj.MaxStep);
            dY = max(min(dY, obj.MaxStep), -obj.MaxStep);

            pedX = obj.xPrev + dX;
            pedY = obj.yPrev + dY;

            % Update stored state
            obj.xPrev = pedX;
            obj.yPrev = pedY;

            % Yaw (deg)
            pedYaw = ori(1);
        end

        function releaseImpl(obj)
            % Turn off sensors cleanly
            try
                obj.m.PositionSensorEnabled = false;
                obj.m.OrientationSensorEnabled = false;
                obj.m.Logging = false;
            catch
            end
        end

        function sts = getSampleTimeImpl(obj)
            % Make block discrete with user-defined sample time
            sts = createSampleTime(obj,'Type','Discrete','SampleTime',obj.SampleTime);
        end
    end
end
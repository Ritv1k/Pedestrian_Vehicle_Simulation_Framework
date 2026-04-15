clc; clear;

%% =================================================
% 1. Read pedestrian GPS data from Excel
%% =================================================
data = readtable("pedestrian.xlsx");   % <-- your file

t   = data.Time_s;        % time (s)
lat = data.Latitude;      % degrees
lon = data.Longitude;     % degrees

% Ensure column vectors
t   = t(:);
lat = lat(:);
lon = lon(:);

%% =================================================
% 2. Normalize time (CRITICAL)
%% =================================================
t = t - t(1);

%% =================================================
% 3. Define local reference (ENU origin)
%    Unreal/Simulink wants LOCAL meters
%% =================================================
lat0 = lat(1);
lon0 = lon(1);
h0   = 0;                 % ground level

wgs84 = wgs84Ellipsoid("meters");

%% =================================================
% 4. Convert GPS → Local ENU (meters)
%% =================================================
[xEast, yNorth, ~] = geodetic2enu( ...
    lat, lon, zeros(size(lat)), ...
    lat0, lon0, h0, wgs84);

%% =================================================
% 5. Smooth GPS jitter
%% =================================================
xEast  = smoothdata(xEast,  "movmean", 5);
yNorth = smoothdata(yNorth, "movmean", 5);

%% =================================================
% 6. Shift trajectory to start at (0,0)
%% =================================================
xEast  = xEast  - xEast(1);
yNorth = yNorth - yNorth(1);

%% =================================================
% 7. Move origin to SAFE SPAWN location (10,10)
%% =================================================
xOffset = 10;   % meters
yOffset = 10;   % meters

xEast  = xEast  + xOffset;
yNorth = yNorth + yOffset;

%% =================================================
% 8. Safety bounds (keeps pedestrian in scene)
%% =================================================
xEast  = max(min(xEast,  30), -30);
yNorth = max(min(yNorth, 30), -30);

%% =================================================
% 9. Compute pedestrian yaw (degrees)
%% =================================================
dx = diff(xEast);
dy = diff(yNorth);

dx(end+1) = dx(end);
dy(end+1) = dy(end);

yawRad = atan2(dy, dx);
yawDeg = rad2deg(yawRad);

yawDeg = fillmissing(yawDeg, "previous");

%% =================================================
% 10. Create Simulink-compatible timeseries
%% =================================================
pedX   = timeseries(xEast,  t);     % meters
pedY   = timeseries(yNorth, t);     % meters
pedYaw = timeseries(yawDeg, t);     % degrees

disp("✅ pedX, pedY, pedYaw created (origin = 10,10)");

%% =================================================
% 11. Save for reuse
%% =================================================
save pedestrianSignals.mat pedX pedY pedYaw

%% =================================================
% 12. Sanity check plots (recommended)
%% =================================================
figure;
subplot(1,2,1)
plot(xEast, yNorth, '-o')
axis equal; grid on
xlabel('X East (m)')
ylabel('Y North (m)')
title('Pedestrian Trajectory (Origin = 10,10)')

subplot(1,2,2)
plot(t, yawDeg)
grid on
xlabel('Time (s)')
ylabel('Yaw (deg)')
title('Pedestrian Yaw')

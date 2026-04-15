% --- FAKE UDP PATH SENDER (X, Y, YAW) ---
clear all; close all; clc;

% Setup UDP Port (Sending to localhost/your own PC)
targetIP = "127.0.0.1";
targetPort = 5000;
u = udpport("IPv4", "LocalPort", 9090);
set_param(bdroot, 'EnablePacing', 'on');

disp('🚗 Blasting Fake [X, Y, Yaw] Telemetry to Simulink...');
disp('Press Ctrl+C in the Command Window to stop.');

t = 0;
dt = 0.05;  % 20Hz update rate
v = 5.0;    % Speed in m/s
R = 30.0;   % Turning radius in meters

while true
    % 1. Calculate Fake Geometry (A continuous circle)
    yaw_rad = (v / R) * t; 
    
    % The block documentation explicitly states Yaw must be in DEGREES
    yaw_deg = yaw_rad * (180/pi); 
    
    X = R * sin(yaw_rad);
    Y = R * (1 - cos(yaw_rad));
    
    % 2. Package into a single array: [X, Y, Yaw]
    telemetry_packet = [X, Y, yaw_deg];
    
    % 3. Blast it over UDP
    write(u, telemetry_packet, "double", targetIP, targetPort);
    
    % Advance time and pause
    t = t + dt;
    pause(dt); 
end
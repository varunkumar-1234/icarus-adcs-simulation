% sensor_noise_config.m
% Sets up the variance and bias for the custom 3-sensor suite (Magnetometer, Sun Sensor, Gyroscope)
% This implements Day 2 Sensor noise requirements.

disp('--- Setting up Day 2 Sensor Noise Models ---');

% Magnetometer Noise (Tesla)
% Typical CubeSat Magnetometer noise is around 100 nT
mag_variance = (100e-9)^2; 
assignin('base', 'mag_variance', mag_variance);

% Gyroscope Noise (rad/s)
% Typical low-cost MEMS gyro noise is around 0.1 deg/s
gyro_variance = (0.1 * pi/180)^2;
assignin('base', 'gyro_variance', gyro_variance);

% Sun Sensor Noise (unit vector)
% Error variance in pointing vector estimation (unitless since it's a vector)
sun_variance = 0.01^2;
assignin('base', 'sun_variance', sun_variance);

disp('Sensor noise parameters loaded into workspace.');

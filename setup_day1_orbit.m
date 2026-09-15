% setup_day1_orbit.m
% This script configures the initial orbit parameters for the CubeSat Simulation Project.
% It sets up a 500 km circular Low Earth Orbit (LEO) as requested in the Day 1 Sprint.

disp('--- Setting up Day 1 Orbit Parameters ---');

% Earth Physical Constants
EarthRadius = 6371000; % meters

% Target Orbit: ~500 km circular LEO
altitude = 500000; % 500 km in meters
semi_major_axis = EarthRadius + altitude;
eccentricity = 0; % 0 means perfectly circular
inclination = 51.6; % degrees (similar to the ISS)

% The asbCubeSat model uses a specific structure or workspace variables.
% We will inject these standard Keplarian elements into the base workspace.
assignin('base', 'inc', inclination);
assignin('base', 'ecc', eccentricity);
assignin('base', 'sma', semi_major_axis);

disp('Orbit set to ~500 km Circular LEO.');
disp('Run the Simulink model to verify 3D Animation and orbit propagation.');

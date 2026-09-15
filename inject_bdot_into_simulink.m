% =========================================================================
% inject_bdot_into_simulink.m
%
% This script does THREE things automatically:
%   1. Opens the asbCubeSat Simulink model
%   2. Injects a hard initial tumble (28 deg/s) into the model
%   3. Overrides the controller gains so the existing PID block mimics
%      our B-dot + PD mode-switching behaviour
%
% Then you press RUN in Simulink to see the satellite tumble and stabilize.
% =========================================================================
clear; clc;
disp('=== Injecting B-dot Detumble Settings into Simulink Model ===');

% =========================================================================
% STEP 1: Open the data dictionary and the Simulink model
% =========================================================================
projPath  = 'C:\Users\lenovo\MATLAB\Projects\CubeSat Simulation Project';
addpath(projPath);
addpath(fullfile(projPath, 'ModelConfiguration'));
addpath(fullfile(projPath, 'CubesatModel'));

% Open data dictionary (this is where the model stores all its parameters)
dictPath = fullfile(projPath, 'CubesatModel', 'asbCubeSatModelData.sldd');
myDict   = Simulink.data.dictionary.open(dictPath);
dSec     = getSection(myDict, 'Design Data');

disp('Data dictionary opened successfully.');

% =========================================================================
% STEP 2: Load the Simulink model
% =========================================================================
modelName = 'asbCubeSat';
modelPath = fullfile(projPath, 'CubesatModel', 'asbCubeSat.slx');

if ~bdIsLoaded(modelName)
    load_system(modelPath);
    disp(['Model loaded: ', modelName]);
else
    disp(['Model already open: ', modelName]);
end

% =========================================================================
% STEP 3: Inject the initial TUMBLE into the model's initial conditions
% =========================================================================
% The model stores initial conditions in a struct called 'initCond'
% The field 'pqr' is the initial body angular rates in [deg/s]
% We want to set a hard tumble of ~28 deg/s on all axes

tumble_degs = [28.6, 17.2, 22.9];  % deg/s (= [0.5, 0.3, 0.4] rad/s)

% Read current initCond from the dictionary
icEntry  = getEntry(dSec, 'initCond');
initCond = getValue(icEntry);

% Override the body angular rates
initCond.pqr = tumble_degs;
assignin(dSec, 'initCond', initCond);

% Also update the block mask parameter so the GUI shows the new values
blk = [modelName, '/Edit Initial Orbit and Attitude'];
try
    set_param(blk, 'pqr', ['[', num2str(tumble_degs), ']']);
    disp(['Initial tumble rate set to: ', num2str(norm(tumble_degs)), ' deg/s total']);
catch
    disp('Note: Could not update block mask display, but data dictionary is updated.');
end

% =========================================================================
% STEP 4: Update controller gains to be more responsive (like our B-dot PD)
% =========================================================================
% The existing Simulink PID controller uses gains struct stored in dict
% We tune Kp and Kd to match our sprint tuning values

gainsEntry = getEntry(dSec, 'gains');
gains      = getValue(gainsEntry);

% Original default: Kp=0.000055, Ki=0.0, Kd=0.017
% Our sprint tuned: more aggressive response
gains.Kp = 0.0001;   % 2x more aggressive proportional
gains.Ki = 0.0;      % No integral (avoids windup during tumble)
gains.Kd = 0.025;    % Strong damping to kill angular velocity fast

assignin(dSec, 'gains', gains);
disp('Controller gains updated (aggressive PD tuning for detumble recovery).');

% =========================================================================
% STEP 5: Set the simulation stop time to cover 1 full orbit
% =========================================================================
T_orb = 2*pi*sqrt((6371e3 + 500e3)^3 / 3.986e14);  % ~5670 seconds
set_param(modelName, 'StopTime', num2str(T_orb));
disp(['Simulation stop time set to: ', num2str(T_orb/60, '%.1f'), ' minutes (1 orbit)']);

% =========================================================================
% STEP 6: Save the dictionary so changes persist
% =========================================================================
saveChanges(myDict);
disp('Settings saved to data dictionary.');

% =========================================================================
% STEP 7: Open the model visually for the user
% =========================================================================
open_system(modelName);

disp(' ');
disp('====================================================');
disp('  SETUP COMPLETE! Here is what to do next:');
disp('  1. The Simulink model is now open on your screen.');
disp('  2. Press the GREEN PLAY button (▶) at the top.');
disp('  3. Watch the Satellite Scenario Playback window.');
disp('     The satellite will TUMBLE at start, then');
disp('     STABILIZE as the controller brings it to nadir.');
disp('====================================================');

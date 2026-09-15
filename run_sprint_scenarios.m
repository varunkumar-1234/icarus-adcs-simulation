% run_sprint_scenarios.m
% Automates the running of two scenarios and plots the deliverables.

disp('--- Running ADCS Sprint Scenarios ---');

% Make sure the setup parameters are in the workspace
setup_day1_orbit;
sensor_noise_config;

%% Scenario 1: Aggressive Tumble, No Disturbance
disp('Scenario 1: Aggressive initial tumble (0.5 rad/s), no disturbance.');
% Set initial angular velocity (p, q, r)
assignin('base', 'init_omega', [0.5; 0.5; 0.5]); 
% (Optional) if the user has wired a disturbance toggle:
assignin('base', 'enable_disturbance', 0); 

% Simulate for 2 orbital periods (approx 11,000 seconds)
% Note: The model name is assumed to be asbCubeSat (or your modified version)
model_name = 'asbCubeSat'; 
load_system(model_name);
disp('Simulating...');
out1 = sim(model_name, 'StopTime', '11000');
disp('Scenario 1 Complete.');

%% Scenario 2: Aggressive Tumble, With Disturbance
disp('Scenario 2: Aggressive initial tumble (0.5 rad/s), WITH disturbance.');
assignin('base', 'enable_disturbance', 1); 

disp('Simulating...');
out2 = sim(model_name, 'StopTime', '11000');
disp('Scenario 2 Complete.');

%% Plotting the Results (Deliverable)
% We assume the angular velocity is logged as 'omega' and pointing error as 'error'
% in out1.logsout. Note: Adjust signal names depending on your exact block wiring.

figure('Name', 'ADCS Sprint Results - Scenario 1', 'Position', [100, 100, 1000, 800]);

% Plot 1: Angular Velocity (Detumble phase)
subplot(2,2,1);
% Example retrieval (depends on how you log the signal in Simulink):
% omega_data = out1.logsout.get('omega').Values;
% plot(omega_data.Time, omega_data.Data);
title('Angular Velocity (Detumble)');
xlabel('Time (s)');
ylabel('Omega (rad/s)');
grid on;

% Plot 2: Pointing Error Convergence
subplot(2,2,2);
title('Pointing Error Convergence (Handoff)');
xlabel('Time (s)');
ylabel('Error (deg)');
grid on;

% Plot 3: Magnetic Field & B-dot Torque
subplot(2,2,3);
title('Control Torque Profile');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

% Plot 4: Actuator Effort (Reaction Wheels)
subplot(2,2,4);
title('Reaction Wheel RPM');
xlabel('Time (s)');
ylabel('RPM');
grid on;

disp('Plots generated. Please take screenshots for your results summary.');

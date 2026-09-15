% =========================================================================
% ADCS_Detumble_Simulation.m  (v4 — Stable pointing mode, bug fix)
% Self-contained B-dot detumbling simulation for a 3U CubeSat.
% Run this script directly in MATLAB. No Simulink wiring needed.
%
% WHAT THIS SIMULATES:
%   Phase 1 (B-dot): Magnetorquers brake the tumble using Earth's B field
%   Phase 2 (PD):    Reaction wheels point satellite at Earth (nadir)
% =========================================================================
clear; clc; close all;
disp('=== ADCS Detumbling Simulation v4 Starting ===');

% =========================================================================
% PHYSICAL PROPERTIES (3U CubeSat — approx 3kg, 10x10x34cm)
% =========================================================================
Ixx = 0.002;  % [kg.m^2] — roll (short axis, easy to spin)
Iyy = 0.010;  % [kg.m^2] — pitch (long axis, harder)
Izz = 0.010;  % [kg.m^2] — yaw  (long axis, harder)
I     = diag([Ixx, Iyy, Izz]);
I_inv = inv(I);
max_dipole  = 0.1;           % Magnetorquer max dipole [Am^2]

% =========================================================================
% ORBITAL PARAMETERS (~500 km LEO)
% =========================================================================
mu_e  = 3.986e14;
R_e   = 6.371e6;
alt   = 500e3;
r_orb = R_e + alt;
T_orb = 2*pi*sqrt(r_orb^3/mu_e);  % ~5670 s
w_orb = 2*pi/T_orb;
B0    = 3.12e-5;  % Earth equatorial dipole field [Tesla]

disp(['Orbital Period : ', num2str(T_orb/60,'%.1f'), ' minutes']);

% =========================================================================
% CONTROL PARAMETERS
% =========================================================================
k_bdot    = 8000;   % B-dot gain — higher = more aggressive braking
Kp        = 0.004;  % PD pointing: proportional gain
Kd        = 0.012;  % PD pointing: derivative (damping) gain
threshold = 0.052;  % Mode switch threshold [rad/s] (~3 deg/s)

% Hysteresis: must stay below threshold for this many steps before latching
hysteresis_needed = 120;  % 120 x 0.5s = 60 continuous seconds stable

% =========================================================================
% INITIAL CONDITIONS — satellite tumbling out of the rocket
% =========================================================================
omega0 = [0.5; 0.3; 0.4];  % Angular velocity [rad/s] (hard tumble)
q0     = [0.1; 0.05; 0.0; 0.993]; q0 = q0/norm(q0); % slight initial tilt
x0     = [q0; omega0];     % State = [quaternion(4); omega(3)]

% =========================================================================
% SIMULATION — RK4 fixed-step integrator
% State vector: x = [qx, qy, qz, qw, wx, wy, wz]  (7 elements)
% =========================================================================
dt    = 0.5;
t_end = 2 * T_orb;   % Simulate 2 full orbits
N     = round(t_end / dt);

T_log      = zeros(N,1);
omega_log  = zeros(N,3);
q_log      = zeros(N,4);
torque_log = zeros(N,3);
mode_log   = zeros(N,1);

x    = x0;
mode = 0;               % 0 = B-dot detumble,  1 = Earth pointing
hyst = 0;               % Hysteresis counter

disp(['Simulating   : ', num2str(t_end/60,'%.0f'), ' minutes (', num2str(N), ' steps)']);

for k = 1:N
    t = (k-1)*dt;

    q     = x(1:4); q = q/norm(q);
    omega = x(5:7);

    % --- Earth's magnetic field in satellite body frame ---
    theta  = w_orb * t;
    B_eci  = B0 * [-2*sin(theta); cos(theta); 0];
    qw=q(4); qx=q(1); qy=q(2); qz=q(3);
    Rbody  = [1-2*(qy^2+qz^2),  2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw);
               2*(qx*qy+qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz-qx*qw);
               2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx^2+qy^2)];
    B_body = Rbody * B_eci;

    om_mag = norm(omega);

    % --- Mode switch with hysteresis (prevents false early latch) ---
    if mode == 0
        if om_mag < threshold
            hyst = hyst + 1;
            if hyst >= hysteresis_needed
                mode = 1;
                disp(['  *** B-dot -> Earth Pointing at T=', ...
                      num2str(t/60,'%.1f'), ' min | omega=', ...
                      num2str(om_mag*180/pi,'%.2f'), ' deg/s ***']);
            end
        else
            hyst = 0;  % Reset if tumble picks back up
        end
    end
    % mode latches — never goes back to 0

    % --- Compute control torque ---
    if mode == 0
        % ---- B-DOT DETUMBLE ----
        % B_dot in body frame ≈ -(omega × B)
        % Commanded dipole: M = k * (omega × B)
        % Resulting torque:  tau = M × B
        M   = k_bdot * cross(omega, B_body);
        M   = max(-max_dipole, min(max_dipole, M));  % saturate
        tau = cross(M, B_body);

    else
        % ---- EARTH POINTING (PD — direct torque, ideal actuators) ----
        % Target: quaternion = [0,0,0,1] (body aligned with orbital frame)
        % Error: the vector part of the quaternion IS the rotation error
        q_ev = q(1:3);
        if q(4) < 0, q_ev = -q_ev; end  % shortest path

        % PD law: tau = -Kp*error - Kd*angular_rate
        tau = -Kp * q_ev - Kd * omega;

        % Soft saturation to prevent huge transient at mode switch
        tau_max = 5e-4;   % [Nm] realistic reaction wheel torque
        tau_mag = norm(tau);
        if tau_mag > tau_max
            tau = tau * (tau_max / tau_mag);
        end
    end

    % --- Log ---
    T_log(k)       = t;
    omega_log(k,:) = omega';
    q_log(k,:)     = q';
    torque_log(k,:)= tau';
    mode_log(k)    = mode;

    % --- RK4 step (state = [q; omega], no wheel states) ---
    f1 = sat_dyn(x,       tau, I, I_inv);
    f2 = sat_dyn(x+dt/2*f1, tau, I, I_inv);
    f3 = sat_dyn(x+dt/2*f2, tau, I, I_inv);
    f4 = sat_dyn(x+dt*f3,   tau, I, I_inv);
    x  = x + (dt/6)*(f1 + 2*f2 + 2*f3 + f4);
    x(1:4) = x(1:4)/norm(x(1:4));  % re-normalize quaternion

    % Progress report every 10%
    if mod(k, round(N/10)) == 0
        ms = 'B-dot'; if mode==1, ms='Pointing'; end
        disp(['  ', num2str(k/N*100,'%3.0f'),'% | T=', ...
              num2str(t/60,'%6.1f'), 'min | omega=', ...
              num2str(om_mag*180/pi,'%6.2f'), ' deg/s | Mode: ', ms]);
    end
end

disp('Simulation complete! Generating plots...');

% =========================================================================
% DERIVED QUANTITIES
% =========================================================================
omega_mag      = sqrt(sum(omega_log.^2, 2));
pointing_error = 2*acosd(min(abs(q_log(:,4)), 1));
torque_mag     = sqrt(sum(torque_log.^2, 2));

sw = find(mode_log == 1, 1, 'first');
if isempty(sw), sw = N; end

% =========================================================================
% PLOTS — 4 subplots saved as PNG
% =========================================================================
fig = figure('Name','ADCS Detumble Results','Position',[40 40 1150 800],'Visible','off');

% 1. Angular velocity
ax1 = subplot(2,2,1);
hold on;
plot(T_log/60, omega_log(:,1)*180/pi, 'r-',  'LineWidth',1.5,'DisplayName','\omega_x');
plot(T_log/60, omega_log(:,2)*180/pi, 'g-',  'LineWidth',1.5,'DisplayName','\omega_y');
plot(T_log/60, omega_log(:,3)*180/pi, 'b-',  'LineWidth',1.5,'DisplayName','\omega_z');
plot(T_log/60, omega_mag*180/pi,       'y--', 'LineWidth',2.5,'DisplayName','|\omega| (magnitude)');
yline(threshold*180/pi,'m-.','LineWidth',1.5,'DisplayName','Switch Threshold');
xline(T_log(sw)/60,    'k:','LineWidth',2.0,'DisplayName','Mode Switch Event');
hold off; grid on; box on;
xlabel('Time (min)'); ylabel('Angular Velocity (deg/s)');
title('\bfAngular Velocity During B-dot Detumble');
legend('Location','northeast','FontSize',7);

% 2. Pointing error + mode
ax2 = subplot(2,2,2);
yyaxis left;
plot(T_log/60, pointing_error,'b-','LineWidth',2);
ylabel('Pointing Error (deg)');
yyaxis right;
plot(T_log/60, mode_log,'r-','LineWidth',2);
ylabel('Mode  (0 = B-dot | 1 = Nadir Pointing)','Color','r');
grid on; box on;
xlabel('Time (min)');
title('\bfPointing Error & Mode Switching');
legend({'Pointing Error','Control Mode'},'Location','east','FontSize',8);

% 3. Control torque
ax3 = subplot(2,2,3);
hold on;
plot(T_log/60, torque_log(:,1)*1e6,'r-','LineWidth',1.2,'DisplayName','T_x');
plot(T_log/60, torque_log(:,2)*1e6,'g-','LineWidth',1.2,'DisplayName','T_y');
plot(T_log/60, torque_log(:,3)*1e6,'b-','LineWidth',1.2,'DisplayName','T_z');
plot(T_log/60, torque_mag*1e6,'k--','LineWidth',2.0,'DisplayName','|T| (magnitude)');
xline(T_log(sw)/60,'k:','LineWidth',2,'DisplayName','Mode Switch');
hold off; grid on; box on;
xlabel('Time (min)'); ylabel('Control Torque (\muNm)');
title('\bfControl Torque (B-dot MTQ → Reaction Wheel)');
legend('Location','northeast','FontSize',7);

% 4. Attitude angles (Euler from quaternion)
ax4 = subplot(2,2,4);
roll  = atan2d(2*(q_log(:,4).*q_log(:,1)+q_log(:,2).*q_log(:,3)), ...
               1-2*(q_log(:,1).^2+q_log(:,2).^2));
pitch = asind(max(-1,min(1, 2*(q_log(:,4).*q_log(:,2)-q_log(:,3).*q_log(:,1)))));
yaw   = atan2d(2*(q_log(:,4).*q_log(:,3)+q_log(:,1).*q_log(:,2)), ...
               1-2*(q_log(:,2).^2+q_log(:,3).^2));
hold on;
plot(T_log/60, roll, 'r-','LineWidth',1.5,'DisplayName','Roll');
plot(T_log/60, pitch,'g-','LineWidth',1.5,'DisplayName','Pitch');
plot(T_log/60, yaw,  'b-','LineWidth',1.5,'DisplayName','Yaw');
xline(T_log(sw)/60,'k:','LineWidth',2,'DisplayName','Mode Switch');
hold off; grid on; box on;
xlabel('Time (min)'); ylabel('Angle (deg)');
title('\bfAttitude Angles (Roll / Pitch / Yaw)');
legend('Location','northeast','FontSize',7);

sgtitle('CubeSat ADCS Sprint — B-dot Detumble + Earth Pointing','FontSize',13,'FontWeight','bold');

% Save PNG
out_png = 'C:\Users\lenovo\MATLAB\Projects\CubeSat Simulation Project\ADCS_Results.png';
set(fig,'PaperPositionMode','auto');
print(fig, out_png, '-dpng', '-r150');
disp(['Graph saved to: ', out_png]);

% =========================================================================
% SUMMARY
% =========================================================================
disp(' ');
disp('====== RESULTS SUMMARY ======');
disp(['  Initial tumble rate  : ', num2str(norm(omega0)*180/pi,'%.1f'), ' deg/s']);
disp(['  Mode switch at T     : ', num2str(T_log(sw)/60,'%.1f'), ' minutes']);
disp(['  Switch omega         : ', num2str(omega_mag(sw)*180/pi,'%.3f'), ' deg/s']);
disp(['  Final tumble rate    : ', num2str(omega_mag(end)*180/pi,'%.4f'), ' deg/s']);
disp(['  Final pointing error : ', num2str(pointing_error(end),'%.2f'), ' degrees']);
disp('=============================');

% =========================================================================
% RIGID BODY DYNAMICS (no wheel states — direct torque application)
% =========================================================================
function dxdt = sat_dyn(x, tau, I, I_inv)
    q     = x(1:4); q = q/norm(q);
    omega = x(5:7);
    % Euler equation: I*w_dot = tau - w×(I*w)
    omega_dot = I_inv * (tau - cross(omega, I*omega));
    % Quaternion kinematics: dq/dt = 0.5 * [0 w; -w^T 0] * q
    ox=omega(1); oy=omega(2); oz=omega(3);
    Om = [ 0,  oz, -oy, ox;
          -oz,  0,  ox, oy;
           oy,-ox,   0, oz;
          -ox,-oy, -oz,  0];
    q_dot = 0.5 * Om * q;
    dxdt  = [q_dot; omega_dot];
end

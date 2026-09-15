function magnetic_dipole = bdot_detumble(B_body, omega_body)
    % bdot_detumble: Implements the B-dot control law for detumbling.
    % Inputs:
    %   B_body     : Magnetic field vector in the satellite's body frame (3x1) [Tesla]
    %   omega_body : Angular velocity of the satellite (3x1) [rad/s]
    %
    % Outputs:
    %   magnetic_dipole : Commanded magnetic dipole for the magnetorquers (3x1) [Am^2]

    % Control Gain (k). A higher number means more aggressive detumbling.
    % Value depends on satellite inertia and magnetorquer strength.
    k = 40000; 

    % The fundamental B-dot law using the cross product approximation.
    % Since B is relatively constant over small intervals, the derivative of B 
    % in the body frame is approximately exactly opposite to the cross product 
    % of omega and B. (B_dot = -omega x B).
    % Therefore, M = -k * B_dot becomes M = k * (omega x B)
    
    B_dot = cross(omega_body, B_body);
    
    % Commanded dipole moment
    magnetic_dipole = k * B_dot;
    
    % Limit the output to realistic magnetorquer limits (e.g., 0.1 Am^2)
    max_dipole = 0.1;
    for i = 1:3
        if magnetic_dipole(i) > max_dipole
            magnetic_dipole(i) = max_dipole;
        elseif magnetic_dipole(i) < -max_dipole
            magnetic_dipole(i) = -max_dipole;
        end
    end
end

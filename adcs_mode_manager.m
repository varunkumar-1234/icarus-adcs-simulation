function mode = adcs_mode_manager(omega_body)
    % adcs_mode_manager: Switches control mode based on tumble rate.
    % Inputs:
    %   omega_body : Angular velocity of the satellite (3x1) [rad/s]
    %
    % Outputs:
    %   mode : 0 for Detumble Mode (B-dot), 1 for Pointing Mode

    % Calculate the magnitude (speed) of the tumble
    tumble_rate = norm(omega_body);
    
    % Threshold: Hand off to pointing mode when tumble rate drops below 0.05 rad/s (~2.8 deg/s)
    threshold = 0.05; 
    
    if tumble_rate > threshold
        mode = 0; % Stay in Detumble Mode
    else
        mode = 1; % Hand off to Pointing Mode
    end
end

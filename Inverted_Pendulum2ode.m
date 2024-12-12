function dxdt = Inverted_Pendulum2ode(X, X_dot, u, M, m, g, l, c, b, I)
    % Extract the state variables
    theta     = X(2);       % Pendulum angle (theta)
    x_dot     = X_dot(1);   % Cart velocity
    theta_dot = X_dot(2);   % Pendulum angular velocity

    % Control input force
    F = u;

    %% Precompute constants for efficiency
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    sin_theta_squared = sin_theta^2;
    cos_theta_squared = cos_theta^2;

    % Denominator term used in equations of motion
    alpha_a = (m^2 * l^2 * sin_theta_squared) + M * m * l^2 + (M + m) * I;

    % Ensure alpha_a is not too small (to avoid numerical instability)
    if abs(alpha_a) < 1e-10
        warning('Denominator alpha_a is close to zero. Numerical instability may occur.');
        alpha_a = sign(alpha_a) * 1e-10; % Avoid division by zero
    end

    %% Compute accelerations
    % Cart acceleration (x_ddot)
    x_ddot = (b * m * l * theta_dot * cos_theta + m^2 * l^2 * g * sin_theta * cos_theta + ...
              (I + m * l^2) * (F - c * x_dot + m * l * sin_theta * theta_dot^2)) / alpha_a;

    % Pendulum angular acceleration (theta_ddot)
    theta_ddot = -(F * m * l * cos_theta - c * m * l * x_dot * cos_theta + ...
                   m^2 * l^2 * theta_dot^2 * sin_theta * cos_theta + (M + m) * ...
                   (b * theta_dot + m * g * l * sin_theta)) / alpha_a;

    %% Output the rate of change of state
    dxdt = [x_ddot; theta_ddot];   % Return as a column vector
end

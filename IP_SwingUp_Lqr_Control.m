clear all
close all
clc

%% Parameters
r   = 0.006;       % Radius of motor shaft
M   = 0.135;       % Mass of cart
m   = 0.1;         % Mass of pendulum
I   = 0.0007176;   % MOI of pendulum
l   = 0.2;         % COM of pendulum
g   = 9.81;        % Gravity constant
b   = 0.00007892;  % Viscous damping at pivot of pendulum
L   = 0.046;       % Motor inductance
Rm  = 12.5;        % Motor resistance
kb  = 0.031;       % Motor back emf constant
kt  = 0.031;       % Motor torque constant
c   = 0.63;        % Friction coefficient of cart

Er  = 2*m*g*l;     % Potential energy of pendulum
n   = 3;
k_swing = 1.2;

%% LQR Control Design
% Calculate A Matrix
AA = I*(M+m) + M*m*(l^2);
aa = (((m*l)^2)*g)/AA;
bb = ((I + m*(l^2))/AA)*(c + (kb*kt)/(Rm*(r^2)));
cc = (b*m*l)/AA;
dd = (m*g*l*(M+m))/AA;
ee = ((m*l)/AA)*(c + (kb*kt)/(Rm*(r^2)));
ff = ((M+m)*b)/AA;
mm = ((I + m*(l^2))*kt)/(AA*Rm*r);
nn = (m*l*kt)/(AA*Rm*r);
A = [0 0 1 0; 0 0 0 1; 0 aa -bb -cc; 0 dd -ee -ff];
B = [0; 0; mm; nn];

% Calculate LQR gain
Q = diag([200 1000 0 0]);
R = 0.035;
KK = lqr(A, B, Q, R);

%% Simulation Parameters
Ts = 0.01; % Sample time
Tf = 9;    % Simulation duration
X0 = [0; 1*(pi/180); 0; 0]; % Initial state
X_des = [0; pi; 0; 0];      % Desired state
u0 = 0;

% Helper function: Saturation
sat = @(x, x_max, x_min) min(x_max, max(x_min, x));

%% Simulation Loop
i = 0;
for k = 0:Ts:Tf
    i = i + 1;
    new_state = RK4_2nd_order(X0, Ts, u0, M, m, g, l, c, b, I);
    
    % Adjust theta for LQR control
    if new_state(2) < 0
        th = 2*pi - abs(new_state(2));
        updated_state = [new_state(1); th; new_state(3); new_state(4)];
    else
        updated_state = new_state;
    end
    
    % Store simulation data
    Xp(i, :) = updated_state';
    t(i) = k;
    X0 = new_state;
    
    % Energy-based swing-up control
    E = m*g*l*(1 - cos(updated_state(2))) + 0.5*(I + m*l^2)*(updated_state(4)^2);
    accel = 2*(E - Er)*sign(updated_state(4)*cos(updated_state(2)));
    accel = k_swing * g * sat(accel, n*g, -n*g);
    u_swing = (M + m)*(accel) - m*l*((updated_state(4))^2)*sin(updated_state(2)) ...
              - m*l*cos(updated_state(2))*((b*updated_state(4) + m*l*accel*cos(updated_state(2)) ...
              + m*g*l*sin(updated_state(2)))/(I + m*l^2));
    
    % LQR control
    u_volt = -KK*(updated_state - X_des);
    u_volt = sat(u_volt, 12, -12);
    u_lqr = volt2force(u_volt, X0(3), kt, kb, Rm, r);
    
    % Control switching
    if (abs(X_des(2) - updated_state(2)))*(180/pi) <= 30
        u0 = u_lqr;
    else
        u0 = u_swing;
    end
end

%% Animation
figure();
frame_skip = 8;  % Reduce the frame rate (skip 8 frames for faster animation)
for i = 1:frame_skip:length(Xp)
    IP_Animation(Xp(i, 1), Xp(i, 2));
    pause(0.005);  % Reduce the pause to speed up the animation
end

%% Plot Results
figure();
subplot(2, 2, 1);
plot(t, Xp(:, 1)); grid on;
ylabel('X (m)');
xlabel('Time (sec)');

subplot(2, 2, 2);
plot(t, (180/pi) * Xp(:, 2)); grid on;
ylabel('\theta (deg)');
xlabel('Time (sec)');

subplot(2, 2, 3);
plot(t, Xp(:, 3)); grid on;
ylabel('X dot (m/sec)');
xlabel('Time (sec)');

subplot(2, 2, 4);
plot(t, (180/pi) * Xp(:, 4)); grid on;
ylabel('\theta dot (deg/sec)');
xlabel('Time (sec)');

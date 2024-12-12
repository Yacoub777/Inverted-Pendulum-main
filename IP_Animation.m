function animation = IP_Animation(x, th)

persistent j
persistent pjx
persistent pjy

% Initialize j if it's the first call
if isempty(j)
    j = 0;
    pjx = [];  % Initialize pjx for storing pendulum trajectory points
    pjy = [];  % Initialize pjy for storing pendulum trajectory points
end

% Increment j for each new call
j = j + 1;

% Define constants for the animation
W  = 0.2; % width of cart
H  = 0.08; % height of cart
L  = 0.35; % length of pendulum  
wr = 0.04; % wheel radius

% Position of wheels and pendulum
y = H / 2 + wr / 2;   % Vertical position of the cart
w1x = x - 0.9 * W / 2; % Position of the first wheel
w1y = 0;
w2x = x + 0.9 * W / 2 - wr; % Position of the second wheel
w2y = 0;

% Position of pendulum mass
px = x + L * sin(th); 
py = y - L * cos(th); 

% Store pendulum trajectory points for later visualization
pjx(j) = px;
pjy(j) = py;

% Create the base line (ground) of the system
base = plot([-1 1], [0 0], 'k', 'LineWidth', 2); 
hold on;

% Create the cart rectangle
cart = rectangle('Position', [x - W / 2, y - H / 2, W, H], 'Curvature', 0.1, 'FaceColor', [1 0.1 0.1], 'EdgeColor', [1 1 1]);

% Create the wheels
left_wheel = rectangle('Position', [w1x, w1y, wr, wr], 'Curvature', 1, 'FaceColor', [1 1 1], 'EdgeColor', [1 1 1]);
right_wheel = rectangle('Position', [w2x, w2y, wr, wr], 'Curvature', 1, 'FaceColor', [1 1 1], 'EdgeColor', [1 1 1]);

% Create the pendulum rod
pendulum = plot([x px], [y py], 'b', 'LineWidth', 2.5);

% Create a circle representing the pendulum mass
p_cir = viscircles([px py], 0.02, 'Color', [1 0.1 0.1], 'LineWidth', 2.5);

% Create a circle representing the center of the cart
p_cir1 = viscircles([x y], 0.02, 'Color', 'w', 'LineWidth', 0.2);

% Optionally, you can plot the pendulum's trajectory
line_traj = plot(pjx(1:j), pjy(1:j), 'm--', 'LineWidth', 1);

% Set labels and title for the plot
xlabel('X (m)');
ylabel('Y (m)');
title('Inverted Pendulum: SwingUp control');
axis(gca, 'equal');
xlim([-1 1]);
ylim([-0.4 0.5]);

% Enable grid
grid on;

% Optionally, you can enable real-time updates and visualization
drawnow;
pause(0.01);

% Return output for animation if needed
animation = struct();
animation.pjx = pjx;
animation.pjy = pjy;

end

% graphs.m  Visualization helpers for the autonomous object-sorting project.
%
%   Generates the figures used in the project report:
%     1) Roadmap layout (pre-plow points and object positions)
%     2) Wall-avoidance ramp profile (collision-avoidance velocity law)
%     3) Overhead XY plot of the robot trajectory
%     4) Command-movement overlay (roadmap + executed trajectory)
%
%   Sections 3 and 4 expect the simulation logs xPose / yPose in the
%   workspace (exported from the Simulink model). Run the model first, then
%   run the desired section of this script.

%% Roadmap layout -----------------------------------------------------------
% Pre-plow points (in order) and object center for each object slot.
P = [-1.0,  -1.25;
     -0.4,  -2.00;
      0.75, -2.20];
C = [0, -1.5];

P2 = [-0.75, 1.35;
      -0.5,  2.20;
       0.62, 2.10];
C2 = [0, 1.5];

figure('Color', 'w'); hold on; axis equal; grid on

% Object 0 polyline + center
plot(P(:,1), P(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'Color', [0 0.4470 0.7410], 'MarkerFaceColor', [0.3010 0.7450 0.9330])
plot(C(1), C(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'b')

% Object 1 polyline + center
plot(P2(:,1), P2(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'Color', [0 0.4470 0.7410], 'MarkerFaceColor', [0.3010 0.7450 0.9330])
plot(C2(1), C2(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'b')

xlabel('X'); ylabel('Y'); title('Roadmap proposed version')
legend('Pre-plow points', 'Object', 'Location', 'best'); hold off

%% Wall-avoidance ramp ------------------------------------------------------
% Saturated ramp that maps wall distance to a motor-velocity command. The
% command rises linearly until distance 'dt' then saturates at 'yMax',
% letting the robot ease away from a wall instead of stopping abruptly.
dt   = 0.8;     % distance [m] at which the ramp saturates
yMax = 127;     % maximum motor-velocity command
t = 0:0.01:5*dt;
rampFunction = min(yMax, max(0, (t/dt) .* yMax));

figure('Color', 'w');
plot(t, rampFunction, 'LineWidth', 1.5, 'Color', 'r');
xlabel('Distance (m)'); ylabel('Motor velocity');
title('Wall avoidance algorithm'); grid on

%% Robot trajectory (requires xPose, yPose from simulation) -----------------
figure('Color', 'w')
plot(xPose(:,1), yPose(:,1))
title('Overhead XY Plot of Robot Trajectory')
xlabel('X [m]'); ylabel('Y [m]')
axis equal; xlim([-2.5 2.5]); ylim([-2.5 2.5])

%% Command-movement overlay (roadmap + executed trajectory) -----------------
figure('Color', 'w'); hold on; axis equal; grid on
plot(P(:,1), P(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'Color', [0 0.4470 0.7410], 'MarkerFaceColor', [0.3010 0.7450 0.9330])
plot(C(1), C(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'b')
plot(xPose(:,1), yPose(:,1), 'Color', 'b')
axis equal; xlim([-2.5 2.5]); ylim([-2.5 2.5])
xlabel('X'); ylabel('Y'); title('Command movement graph')
legend('Pre-plow points', 'Object', 'Location', 'best'); hold off

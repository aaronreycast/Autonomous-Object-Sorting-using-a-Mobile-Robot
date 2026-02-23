% Points (in requested order) and center
P = [-1.0,  -1.25;
     -0.4,  -2.00;
      0.75, -2.20
      ];
C = [0, -1.5
    ];

P2 = [-0.75, 1.35;
      -0.5, 2.2
      0.62, 2.1];
C2 = [0, 1.5];

figure('Color','w')
hold on
axis equal
grid on


% Object 0
% Plot connected polyline for the three points
plot(P(:,1), P(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'Color',[0 0.4470 0.7410], 'MarkerFaceColor',[0.3010 0.7450 0.9330])

% Plot center point (not connected)
plot(C(1), C(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor','b')


% Plot connected polyline for the three points
plot(P(:,1), P(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'Color',[0 0.4470 0.7410], 'MarkerFaceColor',[0.3010 0.7450 0.9330])

% Plot center point (not connected)
plot(C(1), C(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor','b')


%Object 1
% Plot connected polyline for the three points
plot(P2(:,1), P2(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'Color',[0 0.4470 0.7410], 'MarkerFaceColor',[0.3010 0.7450 0.9330])

% Plot center point (not connected)
plot(C2(1), C2(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor','b')


% Plot connected polyline for the three points
plot(P2(:,1), P2(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'Color',[0 0.4470 0.7410], 'MarkerFaceColor',[0.3010 0.7450 0.9330])

% Plot center point (not connected)
plot(C2(1), C2(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor','b')



% Optional: annotate points
%text(P(:,1)+0.03, P(:,2), arrayfun(@(i) sprintf('(%g,%g)',P(i,1),P(i,2)), (1:3)','UniformOutput',false))
%text(C(1)+0.03, C(2), sprintf('Center (%g,%g)', C(1), C(2)))

xlabel('X')
ylabel('Y')
title('Roadmap proposed version')
legend('Pre-plow points','Object','Location','best')
hold off





%Wall avoidance
% We will define a ramp function that begins at point 0,0  and becomes constant 
% at time x = 'dt', then gets a value y= 127
% Define the ramp function parameters
dt = 0.8; % time at which the ramp becomes constant
yMax = 127; % maximum value of the ramp
% Define the ramp function
t = 0:0.01:5*dt; % time vector
rampFunction = min(yMax, max(0, (t/dt) .* yMax)); % ramp function calculation
% Plot the ramp function
figure('Color','w');
plot(t, rampFunction, 'LineWidth', 1.5, 'Color', 'r');
xlabel('Distance (m)');
ylabel('Motor velocity');
title('Wall avoidance algorithm');
grid on;



%Plot robot's trajectory
% Plot XY
figure(1)
plot(xPose(:,1),yPose(:,1))
title('Overhead XY Plot of Robot Trajectory - 1st Green')
xlabel ('X [m]')
ylabel ('Y [m]')
axis equal                 % optional: equal scaling on both axes
xlim([-2.5 2.5])
ylim([-2.5 2.5])

% Plot theta vs. time
%figure(2)
%plot(tout,180/pi*tPose)
%title('Robot Orientation \theta vs. Time')
%xlabel ('Time [sec]')
%ylabel ('\theta [deg]')













%Command movement graph

figure('Color','w')
hold on
axis equal
grid on

plot(P(:,1), P(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
    'Color',[0 0.4470 0.7410], 'MarkerFaceColor',[0.3010 0.7450 0.9330])



% Plot center point (not connected)
plot(C(1), C(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor','b')


% Plot connected polyline for the three points
plot(P(:,1), P(:,2), '-o', 'LineWidth', 1.5, 'MarkerSize', 8, ...
     'Color',[0 0.4470 0.7410], 'MarkerFaceColor',[0.3010 0.7450 0.9330])


% Plot center point (not connected)
plot(C(1), C(2), 'o', 'MarkerSize', 12, 'MarkerFaceColor','b')


%Plot trajectory of the robot   
plot(xPose(:,1),yPose(:,1), 'Color','b')
%title('Overhead XY Plot of Robot Trajectory - 1st Green')
%xlabel ('X [m]')
%ylabel ('Y [m]')
axis equal                 % optional: equal scaling on both axes
xlim([-2.5 2.5])
ylim([-2.5 2.5])


xlabel('X')
ylabel('Y')
title('Command movement graph - 1st object blue')
legend('Pre-plow points','Object','Location','best')
hold off


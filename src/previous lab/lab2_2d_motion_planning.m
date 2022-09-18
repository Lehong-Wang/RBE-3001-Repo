dataPackageXZ = readmatrix('xz_joint_position.csv');

timeStamps = dataPackageXZ(:, 1);
theta1 = dataPackageXZ(:, 2);
theta2 = dataPackageXZ(:, 3);
theta3 = dataPackageXZ(:, 4);
tipX = dataPackageXZ(:, 5);
tipY = dataPackageXZ(:, 6);
tipZ = dataPackageXZ(:, 7);

% Create a plot with joint angles vs. time
figure
plot(timeStamps, theta1, timeStamps, theta2, timeStamps, theta3)
hold on
title('Joint Values vs. Time')
xlabel('Time (ms)')
ylabel('Joint Angle (deg)')
legend('Joint 1', 'Joint 2', 'Joint 3')   
hold off

% Create a plot with tip positions (x and z axes) vs. time
figure
plot(timeStamps, tipX, timeStamps, tipZ)
hold on
title('Tip Positions vs. Time')
xlabel('Time (ms)')
ylabel('Tip Position (mm)')
legend('X Position', 'Z Position')
hold off

% 2D path for the trajectory in the x-z plane
figure
plot(tipX, tipZ)
hold on
plot(123.5, 159.5, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
plot(169.6, 156.3, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
plot(90.7, 62.5, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
title('X-Z Plane Trajectory of the Arm')
xlabel('X Position (mm)')
ylabel('Z Position (mm)')
hold off

% 2D path for the trajectory in the x-y plane
figure
plot(tipX, tipY)
hold on
plot(56.94, -83.4, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
plot(150.88, -17.14, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
plot(85.86, 52.99, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k')
title('X-Y Plane Trajectory of the Arm')
xlabel('X Position (mm)')
ylabel('Y Position (mm)')
hold off

% Define the coordinates of the blue bob and pink bob, and lengths of pendulum segments
blue_points = [10, 10];
pink_points = [0, 0];
L1 = 10; % Replace with the actual length of L1
L2 = 8;  % Replace with the actual length of L2

% Call the forward kinematics function
[green_points, theta1, theta2] = forwardKinematicsDoublePendulum(blue_points, pink_points, L1, L2);

% Display the results
fprintf('Theta1: %.2f degrees\n', theta1);
fprintf('Theta2: %.2f degrees\n', theta2);

function [green_points, theta1, theta2] = forwardKinematicsDoublePendulum(blue_points, pink_points, L1, L2)

    % Calculate θ2 using trigonometry
    theta2 = atan2(blue_points(2) - pink_points(2), blue_points(1) - pink_points(1));

    % Calculate θ1 using law of cosines
    cos_theta1 = ((blue_points(1) - pink_points(1))^2 + (blue_points(2) - pink_points(2))^2 - L1^2 - L2^2) / (2 * L1 * L2);
    sin_theta1 = sqrt(1 - cos_theta1^2);
    
    % Calculate θ1
    theta1 = atan2(sin_theta1, cos_theta1);

    % Calculate the coordinates of the green bob
    green_x = pink_points(1) + L1 * cos(theta1);
    green_y = pink_points(2) + L1 * sin(theta1);
    
    green_points = [green_x, green_y];

    % Convert angles from radians to degrees
    theta1 = rad2deg(theta1);
    theta2 = rad2deg(theta2);

end

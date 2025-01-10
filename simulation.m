% Main script
% Request user input for pattern choice
pattern_type = input('Please input 1-4 (1-square, 2-triangle, 3-five point star, 4-circle): ');

% Function to process G-code and extract coordinates
function coords = process_gcode(file_path)
    % Initialize variables for the last valid coordinates
    last_coords = [0, 0, 0];
    coords = [];
    
    % Read G-code file line by line
    fid = fopen(file_path, 'r');
    if fid == -1
        error('Could not open the file.');
    end
    
    while ~feof(fid)
        line = fgetl(fid);
        % Check for G0 or G1 commands and extract X, Y, Z values
        if startsWith(line, 'G0') || startsWith(line, 'G1')
            % Split the line into parts and find X, Y, Z coordinates
            tokens = split(line);
            temp_coords = last_coords;  % Copy the last valid coordinates
            
            for i = 2:length(tokens)
                token = tokens{i};
                if startsWith(token, 'X')
                    temp_coords(1) = str2double(token(2:end));
                elseif startsWith(token, 'Y')
                    temp_coords(2) = str2double(token(2:end));
                elseif startsWith(token, 'Z')
                    temp_coords(3) = str2double(token(2:end));
                end
            end
            
            % Update the last valid coordinates
            last_coords = temp_coords;
            coords = [coords; temp_coords];  % Append to the list of coordinates
        end
    end
    
    fclose(fid);
end

% Function to compute joint positions using the equations from the image
function [q1, q2, q3, base, joint1, joint2, end_effector] = computeJointPositionsUsingImages(px, py, pz, l1, l2, l3)
    % px, py, pz: Cartesian coordinates of the end-effector
    % l1, l2, l3: Lengths of the links
    
    % Step 1: Calculate planar distance in the XY plane
    r = sqrt(px^2 + py^2);
    
    % Step 2: Calculate q1 (Base rotation angle)
    q1 = atan2d(py, px); % Arctangent of y/x to find angle in the XY-plane

    % Step 3: Calculate 3D distance to the end-effector
    R = sqrt(r^2 + pz^2);
    
    % Step 4: Compute cos(q3) using the law of cosines
    cos_q3 = (R^2 - l1^2 - l2^2) / (2 * l1 * l2);
    
    % Ensure cos(q3) is within the valid range [-1, 1]
    cos_q3 = max(min(cos_q3, 1), -1);

    % Step 5: Calculate q3 (Elbow angle)
    q3 = acosd(cos_q3);
    
    % Step 6: Compute beta (angle of the line to the target point)
    beta = atan2d(pz, r);
    
    % Step 7: Compute alpha (adjustment for the shoulder angle)
    alpha = acosd((l1^2 + R^2 - l2^2) / (2 * l1 * R));
    
    % Step 8: Calculate q2 (Shoulder angle)
    q2 = beta - alpha;

    q1=-q1; q2=-q2; q3=-q3;

    % Step 9: Compute joint positions
    % Base (fixed at origin)
    base = [0, 0, 0];
    
    % Joint 1 (position of the first joint)
    joint1 = [l1 * cosd(q1) * cosd(q2), l1 * sind(q1) * cosd(q2), l1 * sind(q2)];
    
    % Joint 2 (position of the second joint)
    joint2 = joint1 + [l2 * cosd(q1) * cosd(q2 + q3), l2 * sind(q1) * cosd(q2 + q3), l2 * sind(q2 + q3)];
    
    % End-effector position
    end_effector = joint2 + [l3 * cosd(q1), l3 * sind(q1), 0];
end

% Choose the file based on user input
switch pattern_type
    case 1
        coords = process_gcode('square.nc');
    case 2
        coords = process_gcode('triangle.nc');
    case 3
        coords = process_gcode('five_point_star.nc');
    case 4
        coords = process_gcode('circle.nc');
    otherwise
        disp('Invalid choice');
        return;
end

% Link lengths
distances = sqrt(coords(:, 1).^2 + coords(:, 2).^2 + coords(:, 3).^2);
max_distance = max(distances);
l1 = max_distance / 2;
l2 = max_distance / 2;
l3 = 0;
l4=-100

% Create a figure for the visualization
figure;
hold on;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3DOF Robotic Arm and Trajectory Simulation with Error Calculation');
axis equal;
view(3);

% Plot the trajectory
plot3(coords(:, 1), coords(:, 2), coords(:, 3), '--k', 'LineWidth', 1.5); % Dashed line for the trajectory
scatter3(coords(:, 1), coords(:, 2), coords(:, 3), 'r', 'filled'); % Highlight waypoints
legend_entries = {'Trajectory', 'Waypoints'};

% Simulate the robot's movement along the trajectory
reached_points = []; % Store reached points for comparison

for i = 1:size(coords, 1)
    x = coords(i, 1);
    y = coords(i, 2);
    z = coords(i, 3);
    
    % Compute joint angles and positions
    [q1, q2, q3, base, joint1, joint2, end_effector] = computeJointPositionsUsingImages(x, y, z, l1, l2, l3);

    % Compute fixed joint position
    fixed_joint = joint2 + [0, 0, l4]; % Fixed joint is offset in Z direction
    fixed_joint_trajectory = [fixed_joint_trajectory; fixed_joint];
    
    % Append the reached point
    reached_points = [reached_points; end_effector];
    
    % Compute error
    error = abs([x, y, z] - end_effector);
    
    % Plot the arm
    link1 = plot3([base(1), joint1(1)], [base(2), joint1(2)], [base(3), joint1(3)], 'r', 'LineWidth', 2); % Link 1
    link2 = plot3([joint1(1), joint2(1)], [joint1(2), joint2(2)], [joint1(3), joint2(3)], 'b', 'LineWidth', 2); % Link 2
    link_fixed = plot3([joint2(1), fixed_joint(1)], [joint2(2), fixed_joint(2)], [joint2(3), fixed_joint(3)], 'g', 'LineWidth', 2); % Fixed Link
    
    
    % Plot the joints
    base_marker = plot3(base(1), base(2), base(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Base
    joint_marker = plot3(joint1(1), joint1(2), joint1(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Joint
    end_effector_marker = plot3(end_effector(1), end_effector(2), end_effector(3), 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm'); % End effector

    % Display joint angles, reached point, desired point, and error
    fprintf('Point %d:\n', i);
    fprintf('  q1 = %.2f°, q2 = %.2f°, q3 = %.2f°\n, q4 = 180°, q5 = 0°', q1, q2, q3);
    fprintf('  Reached Point: [%.2f, %.2f, %.2f]\n', end_effector(1), end_effector(2), end_effector(3));
    fprintf('  Desired Point: [%.2f, %.2f, %.2f]\n', x, y, z);
    fprintf('  Error: [%.2f, %.2f, %.2f]\n\n', error(1), error(2), error(3));
    
    % Pause for visualization
    pause(0.2);
    
    % Delete previous links and markers to animate
    if i < size(coords, 1)
        delete(link1);
        delete(link2);
        delete(link_fixed);
        delete(base_marker);
        delete(joint_marker);
        delete(end_effector_marker);
    end
end

% Plot the reached points
scatter3(reached_points(:, 1), reached_points(:, 2), reached_points(:, 3), 'g', 'filled');
legend_entries{end+1} = 'Reached Points';

% Finalize the plot
legend(legend_entries);
hold off;

fprintf('Visualization complete.\n');

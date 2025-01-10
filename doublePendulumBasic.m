% Input:
%          O  (Pivot Point - Final) --> green
%          |
%          O  (Pendulum Bob - En medio de la segunda viga) --> blue
%          |
%          O  (Pivot Point - Articulación) --> green
%          |
%          O  (Pendulum Bob - En la base) -->blue


% Load the input image
data = imread(['aaa.jpg']);

% Convert image to HSV color space
data_hsv = rgb2hsv(data);

% Extract the H, S, V channels
H = data_hsv(:,:,1);
S = data_hsv(:,:,2);
V = data_hsv(:,:,3);

% Define HSV thresholds for each color
green_threshold_H = [0.25, 0.5];   
green_threshold_S = [0.1, 1];      
green_threshold_V = [0.3, 1];      

blue_threshold_H = [0.5, 0.6];    
blue_threshold_S = [0.3, 1];       
blue_threshold_V = [0.3, 1];  

pink_threshold_H = [0.9, 1];     
pink_threshold_S = [0.2, 1];      
pink_threshold_V = [0.3, 1];      

% Create binary masks for each color
green_mask = (H >= green_threshold_H(1) & H <= green_threshold_H(2)) & ...
             (S >= green_threshold_S(1) & S <= green_threshold_S(2)) & ...
             (V >= green_threshold_V(1) & V <= green_threshold_V(2));

blue_mask = (H >= blue_threshold_H(1) & H <= blue_threshold_H(2)) & ...
            (S >= blue_threshold_S(1) & S <= blue_threshold_S(2)) & ...
            (V >= blue_threshold_V(1) & V <= blue_threshold_V(2));

pink_mask = (H >= pink_threshold_H(1) & H <= pink_threshold_H(2)) & ...
            (S >= pink_threshold_S(1) & S <= pink_threshold_S(2)) & ...
            (V >= pink_threshold_V(1) & V <= pink_threshold_V(2));

%% Perform morphological operations on the masks 

% Apply morphological operations to the masks

% Green mask
se = strel('disk', 5); % Define the structuring element (adjust the size as needed)
green_mask_opened = imopen(green_mask, se);
green_mask_cleaned = imclose(green_mask_opened, se);

% Blue mask
se = strel('disk', 5); % Define the structuring element (adjust the size as needed)
blue_mask_opened = imopen(blue_mask, se);
blue_mask_cleaned = imclose(blue_mask_opened, se);

% Pink mask
se = strel('disk', 5); % Define the structuring element (adjust the size as needed)
pink_mask_opened = imopen(pink_mask, se);
pink_mask_cleaned = imclose(pink_mask_opened, se);


%% Find connected components and compute region properties

% Find connected components and compute region properties for each mask

% Green mask
bw_green_labeled = bwlabel(green_mask_cleaned, 8);
stats_green = regionprops(bw_green_labeled, 'BoundingBox', 'Centroid');

% Blue mask
bw_blue_labeled = bwlabel(blue_mask_cleaned, 8);
stats_blue = regionprops(bw_blue_labeled, 'BoundingBox', 'Centroid');

% Pink mask
bw_pink_labeled = bwlabel(pink_mask_cleaned, 8);
stats_pink = regionprops(bw_pink_labeled, 'BoundingBox', 'Centroid');


%% Display the original image with detected objects

% Display the original image with detected objects

% Show the original image
imshow(data);
hold on;

% Display the detected objects for each color

% Green objects
for object = 1:length(stats_green)
    bb = stats_green(object).BoundingBox;
    bc = stats_green(object).Centroid;
    rectangle('Position', bb, 'EdgeColor', 'g', 'LineWidth', 2);
    plot(bc(1), bc(2), '-m+');
    text(bc(1)+15, bc(2), 'Green', 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'black');
end

% Blue objects
for object = 1:length(stats_blue)
    bb = stats_blue(object).BoundingBox;
    bc = stats_blue(object).Centroid;
    rectangle('Position', bb, 'EdgeColor', 'b', 'LineWidth', 2);
    plot(bc(1), bc(2), '-m+');
    text(bc(1)+15, bc(2), 'Blue', 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'black');
end

% Pink objects
for object = 1:length(stats_pink)
    bb = stats_pink(object).BoundingBox;
    bc = stats_pink(object).Centroid;
    rectangle('Position', bb, 'EdgeColor', 'r', 'LineWidth', 2);
    plot(bc(1), bc(2), '-m+');
    text(bc(1)+15, bc(2), 'Pink', 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'black');
end


hold off;

%% Call the inverseKinematicsDoublePendulum function here to calculate the angles

% Call the inverseKinematicsDoublePendulum function to calculate the angles
[theta1, theta2] = inverseKinematicsDoublePendulum(stats_green(1).Centroid, stats_blue(1).Centroid, stats_pink(1).Centroid);

% Display the calculated angles
disp(['Theta1 (degrees): ', num2str(theta1)]);
disp(['Theta2 (degrees): ', num2str(theta2)]);


% Save the coordinates for each color
green_points = stats_green(1).Centroid;
blue_points = stats_blue(1).Centroid;
pink_points = stats_pink(1).Centroid;

% Save the coordinates to a file
% save('detected_coordinates.mat', 'green_points', 'blue_points', 'pink_points');


function [theta1, theta2] = inverseKinematicsDoublePendulum(green_points, blue_points, pink_points)

    % Shift the coordinates relative to the pink point (origin)
    green_x = green_points(1) - pink_points(1);
    green_y = pink_points(2) - green_points(2); % Invert Y-axis
    blue_x = blue_points(1) - pink_points(1);
    blue_y = pink_points(2) - blue_points(2); % Invert Y-axis

    % Calculate L1 and L2
    L1 = sqrt(green_x^2 + green_y^2);
    L2 = sqrt((blue_x - green_x)^2 + (blue_y - green_y)^2);
    
    % Calculate θ1 and θ2 using trigonometry
    theta1 = atan2(green_y, green_x);
    theta2 = atan2(blue_y - green_y, blue_x - green_x);

    % Convert angles from radians to degrees
    theta1 = rad2deg(theta1);
    theta2 = rad2deg(theta2);

end


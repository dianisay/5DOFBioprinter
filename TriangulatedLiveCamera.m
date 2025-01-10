% Input:
%          O  (Pivot Point - Final) --> green
%          |
%          O  (Pendulum Bob - En medio de la segunda viga) --> blue
%          |
%          O  (Pivot Point - Articulación) --> green
%          |
%          O  (Pendulum Bob - En la base) -->blue



% Define a list of image filenames
image_filenames = {'ttt.jpg', 'tt.jpg'};

% Initialize variables to store coordinates for each object
green_points_I1 = [];
blue_points_I1 = [];
pink_points_I1 = [];
green_points_I2 = [];
blue_points_I2 = [];
pink_points_I2 = [];

% Loop through the image filenames
for i = 1:length(image_filenames)
    % Load the input image
    data = imread(image_filenames{i});

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
pink_threshold_S = [0.1, 1];      
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


%% NEW
    % Determine which set of variables to populate based on the value of i
    if i == 1
        green_points_I1 = stats_green(1).Centroid;
        blue_points_I1 = stats_blue(1).Centroid;
        pink_points_I1 = stats_pink(1).Centroid;
    else
        green_points_I2 = stats_green(1).Centroid;
        blue_points_I2 = stats_blue(1).Centroid;
        pink_points_I2 = stats_pink(1).Centroid;
    end
% Check if pink_points_I2 is not empty
if ~isempty(pink_points_I2)
    load("stereoParams.mat")
    % Perform triangulation for green, blue, and pink points
    [worldPoints_green, ~, reprojErrors_green] = triangulate(green_points_I1, green_points_I2, stereoParams);
    [worldPoints_blue, ~, reprojErrors_blue] = triangulate(blue_points_I1, blue_points_I2, stereoParams);
    [worldPoints_pink, ~, reprojErrors_pink] = triangulate(pink_points_I1, pink_points_I2, stereoParams);

    %% Call the inverseKinematicsDoublePendulum function here to calculate the angles

        % Call the inverseKinematicsDoublePendulum function to calculate the angles
        [theta1, theta2] = inverseKinematicsDoublePendulum(worldPoints_green, worldPoints_blue, worldPoints_pink);

        % Display the calculated angles
        disp(['Theta1 (degrees): ', num2str(q1)]);
        disp(['Theta2 (degrees): ', num2str(q2)]);
        disp(['Theta3 (degrees): ', num2str(theta2)]);

else
    % Continue with analysis
end
end

function [q1, q2, q3] = inverseKinematicsDoublePendulum(worldPoints_green, worldPoints_blue, worldPoints_pink, I2, I3)
    % Extract the X, Y, and Z coordinates of the green and blue bobs
    % Calculate the differences between original and pink coordinates
    green_x = worldPoints_green(1) - worldPoints_pink(1);
    green_y = worldPoints_green(2) - worldPoints_pink(2);
    green_z = worldPoints_green(3) - worldPoints_pink(3);
    
    blue_x = worldPoints_blue(1) - worldPoints_pink(1);
    blue_y = worldPoints_blue(2) - worldPoints_pink(2);
    blue_z = worldPoints_blue(3) - worldPoints_pink(3);

    % Calculate r, q3, and q2 using the provided equations
    r = sqrt(green_x^2 + green_y^2);
    cos_q3 = (green_x^2 + green_y^2 + green_z^2 - I2^2 - I3^2) / (2 * I2 * I3);
    sin_q3 = sqrt(1 - cos_q3^2);
    q3 = atan2(sin_q3, cos_q3);

    % Calculate q2
    beta = atan2(green_z, r);
    alpha = atan2(I3 * sin_q3, I2 + I3 * cos_q3);
    q2 = beta - alpha;

    % Calculate q1
    q1 = atan2(green_y, green_x);

    % Convert angles from radians to degrees
    q1 = rad2deg(q1);
    q2 = rad2deg(q2);
    q3 = rad2deg(q3);
end



% Input:
%          O  (Pivot Point - Final) --> blue
%          |
%          O  (Pivot Point - Articulación) --> green
%          |
%          O  (Pendulum Bob - En la base) -->pink --> origen [0,0,0]


%PASO1: GUARDAR LOS 2 DATOS
%ACOMODAR LAS CÁMARAS . 10 IMÁGENES C/U NO AUTOAJUSTE

% Define a list of image filenames
image_filenames = {'aa.JPG', 'aaa.JPG'};

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
green_threshold_S = [0.2, 0.5];      
green_threshold_V = [0.3, 1];      

blue_threshold_H = [0.5, 0.6];    
blue_threshold_S = [0.4, 1];       
blue_threshold_V = [0.3, 1];  

pink_threshold_H = [0.9, 1];     
pink_threshold_S = [0.3, 1];      
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
           % Initialize variables to store 3D coordinates
            green_3D = zeros(3, 1);
            blue_3D = zeros(3, 1);
            pink_3D = zeros(3, 1);

             % Calculate 3D coordinates for the pink object
            pink_3D(1) = pink_points_I1(1); % x-coordinate from I1
            pink_3D(2) = pink_points_I2(1); % y-coordinate from I2
            pink_3D(3) = (pink_points_I1(2) + pink_points_I2(2)) / 2; % Average z-coordinate 

            
            % Calculate 3D coordinates for the green object
            green_3D(1) = green_points_I1(1)- pink_3D(1); % x-coordinate from I1
            green_3D(2) = green_points_I2(1)- pink_3D(2); % y-coordinate from I2
            green_3D(3) = pink_3D(3)-((green_points_I1(2) + green_points_I2(2)) / 2); % Average z-coordinate
            
            % Calculate 3D coordinates for the blue object
            blue_3D(1) = blue_points_I1(1)-pink_3D(1); % x-coordinate from I1
            blue_3D(2) = blue_points_I2(1)- pink_3D(2); % y-coordinate from I2
            blue_3D(3) = pink_3D(3)-((blue_points_I1(2) + blue_points_I2(2)) / 2); % Average z-coordinate
  
             % Calculate 3D coordinates for the pink object
            pink_3D(1) = pink_3D(1)-pink_3D(1); % x-coordinate from I1
            pink_3D(2) = pink_3D(2)- pink_3D(2); % y-coordinate from I2
            pink_3D(3) = pink_3D(3)-pink_3D(3); % Average z-coordinate 
          


           % Print the coordinates matrix for the green bob
            disp('Coordinates matrix for the green bob:');
            disp(green_3D);
            
            % Print the coordinates matrix for the blue bob
            disp('Coordinates matrix for the blue bob:');
            disp(blue_3D);
            
            % Print the coordinates matrix for the pink bob
            disp('Coordinates matrix for the pink bob:');
            disp(pink_3D);

            %% Call the inverseKinematicsDoublePendulum function here to calculate the angles

            % Call the inverseKinematicsDoublePendulum function to calculate the angles
            [theta0,theta1, theta2] = inverseKinematicsDoublePendulum(green_3D, blue_3D);
            
            % Display the calculated angles
            disp(['Theta0 (degrees): ', num2str(theta0)]);
            disp(['Theta1 (degrees): ', num2str(theta1)]);
            disp(['Theta2 (degrees): ', num2str(theta2)]);

else
    % Continue with analysis
end
end

function [theta0, theta1, theta2] = inverseKinematicsDoublePendulum(green_3D, blue_3D)

    % Vectors relative to the pink object
    green_x = green_3D(1);
    green_y = green_3D(2);
    green_z = green_3D(2);
    blue_x = blue_3D(1);
    blue_y = blue_3D(2);
    blue_z = blue_3D(3);

    % Calculate the base angle (theta0) based on the green object's position
    theta0 = atan2(green_y, green_x);
    theta0 = rad2deg(theta0);

    % Calculate the lengths of the pendulum links
    L1 = norm([green_x, green_y, green_z]);
    L2 = norm([blue_x, blue_y, blue_z]);

    % Calculate the angle of the first pendulum link (theta1)
    theta1 = atan2(green_z, L1);
    theta1 = rad2deg(theta1);

    % Calculate the angle of the second pendulum link (theta2)
    theta2 = atan2(blue_z - green_z, L2);
    theta2 = rad2deg(theta2);

    % Correct θ1 and θ2 based on regression (Excel)
    theta1 = (theta1 + 17.249)/1.0207;
    theta2 = (theta2 + 7.4303)/1.0218;

end



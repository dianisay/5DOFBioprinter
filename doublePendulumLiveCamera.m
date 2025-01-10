% Input:
% O (Pivot Point - Final) --> green
% |
% O (Pendulum Bob - En medio de la segunda viga) --> blue
% |
% O (Pivot Point - Articulación) --> green
% |
% O (Pendulum Bob - En la base) --> blue% Listar las cámaras conectadas
cam_list = webcamlist;

% Crear un objeto para la primera cámara Logi C270 HD WebCam
cam1 = webcam(2); % La primera 'Logi C270 HD WebCam'

% Crear un objeto para la segunda cámara Logi C270 HD WebCam
cam2 = webcam(3); % La segunda 'Logi C270 HD WebCam'

% Define a cell array to hold webcam objects
cams = {cam1, cam2};

% Initialize variables to store coordinates for each object
green_points_I1 = [];
blue_points_I1 = [];
pink_points_I1 = [];
green_points_I2 = [];
blue_points_I2 = [];
pink_points_I2 = [];

% Initialize a video player
videoPlayer = vision.VideoPlayer;

    % Capture frames from both cameras
    frames = cell(1, 2);
    for i = 1:2
        frames{i} = snapshot(cams{i});
    end

    % Process frames one by one
    for i = 1:2
        data = frames{i};

        % Convert image to HSV color space
        data_hsv = rgb2hsv(data);

        % Extract the H, S, V channels
        H = data_hsv(:,:,1);
        S = data_hsv(:,:,2);
        V = data_hsv(:,:,3);

        % Define HSV thresholds for each color
        %green_threshold_H = [0.25, 0.5];
        %green_threshold_S = [0.1, 1];
        %green_threshold_V = [0.3, 1];
        
        %blue_threshold_H = [0.5, 0.6];
        %blue_threshold_S = [0.3, 1];
        %blue_threshold_V = [0.3, 1];

        %pink_threshold_H = [0.9, 1];
        %pink_threshold_S = [0.1, 1];
        %pink_threshold_V = [0.3, 1];

        % Define the HSV thresholds for each color

        green_threshold_H = [0.13, 0.17];    
        green_threshold_S = [0.4, 1];       % Saturation range (can adjust based on how pure the yellow is)
        green_threshold_V = [0.4, 1];       % Value range (can adjust based on brightness of yellow)
        
        blue_threshold_H = [0.56, 0.75];    
        blue_threshold_S = [0.4, 1];        
        blue_threshold_V = [0.4, 1];        
        
        % Define HSV thresholds for green
        %green_threshold_H = [0.25, 0.40];   
        %green_threshold_S = [0.4, 1];       
        %green_threshold_V = [0.4, 1];       
        
        pink_threshold_H = [0.83, 0.92];    
        pink_threshold_S = [0.4, 1];        
        pink_threshold_V = [0.4, 1];   


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

        % Apply morphological operations to the masks
        se = strel('disk', 5); % Define the structuring element (adjust the size as needed)
        green_mask_opened = imopen(green_mask, se);
        green_mask_cleaned = imclose(green_mask_opened, se);

        blue_mask_opened = imopen(blue_mask, se);
        blue_mask_cleaned = imclose(blue_mask_opened, se);

        pink_mask_opened = imopen(pink_mask, se);
        pink_mask_cleaned = imclose(pink_mask_opened, se);

        % Find connected components and compute region properties for each mask
        bw_green_labeled = bwlabel(green_mask_cleaned, 8);
        stats_green = regionprops(bw_green_labeled, 'BoundingBox', 'Centroid');

        bw_blue_labeled = bwlabel(blue_mask_cleaned, 8);
        stats_blue = regionprops(bw_blue_labeled, 'BoundingBox', 'Centroid');

        bw_pink_labeled = bwlabel(pink_mask_cleaned, 8);
        stats_pink = regionprops(bw_pink_labeled, 'BoundingBox', 'Centroid');

        % Display the original frame with detected objects
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
%% Points

        % Determine which set of variables to populate based on the value of i
        if i == 1
            %verificar si hay objetos detectados antes de intentar acceder a sus propiedades.
                if ~isempty(stats_green)
                    green_points_I1 = stats_green(1).Centroid;
                end
                
                if ~isempty(stats_blue)
                    blue_points_I1 = stats_blue(1).Centroid;
                end
                
                if ~isempty(stats_pink)
                    pink_points_I1 = stats_pink(1).Centroid;
                end
        else
            
        %verificar si hay objetos detectados antes de intentar acceder a sus propiedades.
                if ~isempty(stats_green)
                    green_points_I2 = stats_green(1).Centroid;
                end
                
                if ~isempty(stats_blue)
                    blue_points_I2 = stats_blue(1).Centroid;
                end
                
                if ~isempty(stats_pink)
                    pink_points_I2 = stats_pink(1).Centroid;
                end
        end

        % Check if pink_points_I2 is not empty
        if ~isempty(pink_points_I2)
            % Perform triangulation for green, blue, and pink points
            [worldPoints_green, ~, ~] = triangulate(green_points_I1, green_points_I2, stereoParams); % Use your stereoParams
            [worldPoints_blue, ~, ~] = triangulate(blue_points_I1, blue_points_I2, stereoParams); % Use your stereoParams
            [worldPoints_pink, ~, ~] = triangulate(pink_points_I1, pink_points_I2, stereoParams); % Use your stereoParams

            % Call the inverseKinematicsDoublePendulum function to calculate the angles
            [theta1, theta2] = inverseKinematicsDoublePendulum(worldPoints_green, worldPoints_blue, worldPoints_pink);

            % Display the calculated angles
            disp(['Theta1 (degrees): ', num2str(theta1)]);
            disp(['Theta2 (degrees): ', num2str(theta2)]);
        else
            % Continue with analysis
        end
    end

    % Display the processed frames
    step(videoPlayer, data);


% Release the video player and close the cameras when done
release(videoPlayer);
clear cam1 cam2;

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
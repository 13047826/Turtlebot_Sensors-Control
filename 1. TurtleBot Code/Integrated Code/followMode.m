function followMode(camera, controller, LIDAR)

% Update rgbImg, centre, corners and distance
camera.updateRGBImg;
camera.updateFeatureCentreCorners;


% If feature centre was not detected - toggle to search mode
    if (isnan(get(camera, "featureCorners")) == true)
        
        controller.stop; % stop moving the turtlebot

        set(camera, 'mode', 1);

        disp("-----------------------------------")
        disp("Follower Turtlebot is now searching")
        disp("-----------------------------------")
        
        return;  % exit this function
    end

% If feature centre was detected - complete visual servoing
    if (isnan(get(camera, "featureCorners")) == false)
        
        % Interpolate distance with camera calibration
        camera.interpolateDistance;
        Z = double(get(camera, "featureDistance")/100); % Lidar
        
        % Get Distance If Feature Detected
        % Z = LIDAR.lidardepth();
        m = get(camera, "featureCorners");
        [vLinear, vAngular] = controller.computeVelocity(m,Z);     % Compute appropriate velocities
        controller.publishVelMatrix(vLinear, vAngular);           % Publisher cmdvel 
       
    end

end

function searchMode(camera, controller, LIDAR)

% Capture images and process for feature's centre and corners
camera.updateRGBImg;
camera.updateFeatureCentreCorners; % IMAGE CLOSES HERE

    % If feature is not available (STATE = 1)
    if (isnan(get(camera, "featureCentre")) == true)
        % WAIT FOR LEADER
    end

    % If is available turn towards the leader and follow (STATE = 2)
    if (isnan(get(camera, "featureCentre")) == false)
        
        % Set up
        centre = get(camera, "p");
        featureCentre = get(camera, "featureCentre");
        cmdVelPub = get(controller,"cmdVelPub");
        velMsg = rosmessage(cmdVelPub);
        kp = 0.009;                                                % Additional kp2 variable required to control turn rate proportional to pixel size
        velMsg.Linear.X = 0;
        velMsg.Angular.Z = kp * (centre(1)-featureCentre(1));
        send(cmdVelPub,velMsg);
    
        % If the follower's camera is centred on the features centre 
        if abs((centre(1)-featureCentre(1))/centre(1)) < 0.02 % if error is less than 1 %
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = 0;
            send(cmdVelPub,velMsg);
            set(camera, "mode", 2); % Follow
            disp("-----------------------------------")
            disp("Follower Turtlebot is now following")
            disp("-----------------------------------")
            return;
        end
    end
end



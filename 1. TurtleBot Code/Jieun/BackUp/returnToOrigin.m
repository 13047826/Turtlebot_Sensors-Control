function returnToOrigin(odomSub, velPub, origin, kp, rgbImg, rgbSub, figureMsg, searchMode)
    
    velMsg = rosmessage(velPub);

    while true
        
        % Try to get odometry data with a timeout (prevent waiting
        % indefinitely)
        try
            odomMsg = receive(odomSub,2); % waits for up to 2 seconds
        catch
            disp('Failed to receive an odometry message within 2 seconds.');
            continue;  % skip the rest of this loop iteration
        end

        orientation = odomMsg.Pose.Pose.Orientation;
        eulZYX = quat2eul([orientation.X, orientation.Y, orientation.Z,orientation.W]);
        yaw = eulZYX(3);
        
        targetRad = deg2rad(origin);
        velMsg.Angular.Z = kp * (targetRad - yaw);
        send(velPub,velMsg);

        % Try to get image data with a timeout
        try
            imMsg2 = receive(rgbSub, 2);  % waits for up to 2 seconds
            img = readImage(imMsg2);
            set(rgbImg,'CData',img);
        catch
            disp('Failed to receive an image within 2 seconds.');
            % No need to continue here since I want to handle the case where the marker isn't found
        end

        updateFigureText(rgbImg, figureMsg);

        % Break out if marker is found only while searching
        % only need to check xm so used ~ to ignore other outputs
        if searchMode
            [xm, ~] = blueMarkerCentre(img);
            if ~isempty(xm)
                break; 
            end
        end
        
        % Also break out if at origin
        if ~searchMode && abs(yaw - targetRad) < (origin+0.01)
            break;
        end

    end

end
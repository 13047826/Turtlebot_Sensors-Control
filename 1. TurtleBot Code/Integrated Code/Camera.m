classdef Camera < matlab.mixin.SetGet

    properties %(SetAccess=protected) % Not able to work with "private"
        f % focal length
        p % principal point
        resolution % image size [x,y]
        fov % horizontal, vertical field of view
        featureCentre % centre of feature
        featureCorners % current observed corners of feature
        featureDistance % current distance to the feature
        targetCorners % target corners of feature for visual servoing
        targetDistance % target distance for visual servoing
        rgbImgSub % rgb Image Subscriber 
        rgbImg % rgb Image
        cmdVelPub % cmdVel Publisher
        velM % velocity matrix from visual servoing
        
        mode % Search (1), Follow (2), End (0)
        followModeThreshold % Threshold for accuracy
    end

    methods

        % ----------- %
        % Constructor %
        % ----------- %

        function camera = Camera()
        end

        % -------------------------- %
        % Image Processing Functions %
        % -------------------------- %

        function updateRGBImg(camera)
            %imMsg2 = receive(camera.rgbImgSub);
            %imMsg2.Format = 'bgr8; jpeg compressed bgr8';
            %var = readImage(imMsg2);
            var = readImage(receive(camera.rgbImgSub));
            set(camera, "rgbImg", var);
        end

        function updateFeatureCentreCorners(camera)
            rgbImg = get(camera, "rgbImg");
            
            % Breaking rgbImg into colour components
            r1 = rgbImg(:,:,1);
            g1 = rgbImg(:,:,2);
            b1 = rgbImg(:,:,3);
            
            justBlue = b1 - r1/2 - g1/2; 
            bw = justBlue > 40; % Threshold that turns the image to a binary image
            
            % tempCorners = detectHarrisFeatures(bw);
             tempCorners = detectSURFFeatures(bw); % Detected no corners for bw
            
            % Get corners
            corners = [min(tempCorners.Location(:,1)) min(tempCorners.Location(:,2));
                          min(tempCorners.Location(:,1)) max(tempCorners.Location(:,2)); ...
                          max(tempCorners.Location(:,1)) min(tempCorners.Location(:,2)); ...
                          max(tempCorners.Location(:,1)) max(tempCorners.Location(:,2))];   
            
            centre = [mean(corners(:,1)) mean(corners(:,2))];

            if isnan(centre) == true
                set(camera, "featureCentre", nan);
                set(camera, "featureCorners", nan);
            end
            if isnan(centre) == false
                set(camera, "featureCentre", centre);
                set(camera, "featureCorners", corners);
            end
        end

        function interpolateDistance(camera)

            corners = get(camera,"featureCorners");
            load("SimTargetCorners.mat", 'targets');               % Load targets  % load("SimTargetCorners.mat", 'targets');
            L = corners(2,2) - corners(1,2);                       % Compute bottom edge length      
            for k = 5:5:90 % Compare bottom edge length with available bottom edge lengths 
                if L > targets(k,5)
                    break
                end
            end
            d1 = targets(k,3);
            L1 = targets(k,5);
            d2 = targets(k-5,3);
            L2 = targets(k-5,5);
            d = d1 + (L-L1) * (d2-d1) / (L2 - L1);                % Interpolate
            set(camera,"featureDistance", d);
        end

    end
end
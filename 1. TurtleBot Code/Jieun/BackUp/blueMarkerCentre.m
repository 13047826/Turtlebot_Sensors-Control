function [xm, ym] = blueMarkerCentre(img)
    
    r = img(:,:,1);
    g = img(:,:,2);
    b = img(:,:,3);
    % calculate blue
    blue = b - r/2 - g/2;
    bw = blue > 30;
    % Find the center of the image
    [x, y] = find(bw);
    xm = [];
    ym = [];
    if ~isempty(x) && ~isempty(y)
        xm = round(mean(x));
        ym = round(mean(y));
    end
end

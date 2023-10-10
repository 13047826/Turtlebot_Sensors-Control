function updateFigureText(rgbImg, figureMsg)
    persistent textHandle;
    
    if isempty(textHandle) || ~ishandle(textHandle)
        % Initial creation of the text handle
        hold on;
        textHandle = text(10, 10, figureMsg, 'Color', 'white', 'FontSize', 15);
        hold off;
    else
        % Updating the existing handle
        set(textHandle, 'String', figureMsg);
    end

    if ishandle(rgbImg)
        drawnow;
    end
end

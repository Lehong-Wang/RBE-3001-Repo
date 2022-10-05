
function [func, func2] = yellow_test()
    func = @createMask;
    func2 = @detectBall;
end

function [BW,maskedRGBImage] = createMask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 04-Oct-2022
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.078;
channel1Max = 0.156;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.235;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.432;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;


end








function detectBall(BW, RGB)
    magnification = 25;

    
    % Find connected components.
    blobAnalysis = vision.BlobAnalysis(AreaOutputPort = true,...
        CentroidOutputPort = false,...
        BoundingBoxOutputPort = true,...
        MinimumBlobArea = 200, ExcludeBorderBlobs = true)
    [areas, boxes] = step(blobAnalysis, BW)
    
    % Sort connected components in descending order by area
    [~, idx] = sort(areas, "Descend")
    
    % Get the two largest components.
%     boxes = double(boxes(idx(1:2), :))
    boxes = double(boxes(idx, :))
%     boxes = [boxes(1) boxes(2) 10;
%                 boxes(3) boxes(4) 10]
    
% Reduce the size of the image for display.
% scale = magnification / 25;
% imDetectedCoins = imresize(RGB, scale);
% 
% for i = (1:size(boxes,1))
%     boxes(i,2) = boxes(i,2) - boxes(i,4)/2; 
% end
% 

% Insert labels for the coins.
imDetectedCoins = insertObjectAnnotation(RGB, "rectangle", ...
     boxes, "penny");
% imDetectedCoins = RGB;
figure; imshow(imDetectedCoins);
title("Detected Coins");

end







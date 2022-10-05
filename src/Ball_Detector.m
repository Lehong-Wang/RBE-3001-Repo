

classdef Ball_Detector

  properties
  end


  methods (Static)


    % take in a full color image and a color (R,O,Y,G)
    % return one black white and one full color image of the masked image
    function [BW,maskedRGBImage] = createMaskedImage(RGB, color)
      % Convert RGB image to chosen color space
      I = rgb2hsv(RGB);

      switch color
        case 'R'
          channel1Min = 0.932;
          channel1Max = 0.006;
          
          channel2Min = 0.192;
          channel2Max = 1.000;
          
          channel3Min = 0.373;
          channel3Max = 1.000;

        case 'O'
          channel1Min = 0.031;
          channel1Max = 0.087;
          
          channel2Min = 0.224;
          channel2Max = 1.000;
          
          channel3Min = 0.395;
          channel3Max = 1.000;

        case 'Y'
          channel1Min = 0.103;
          channel1Max = 0.154;

          channel2Min = 0.235;
          channel2Max = 1.000;

          channel3Min = 0.405;
          channel3Max = 1.000;


        case 'G'
          channel1Min = 0.171;
          channel1Max = 0.215;

          channel2Min = 0.219;
          channel2Max = 1.000;

          channel3Min = 0.373;
          channel3Max = 1.000;

        otherwise
          warning("Unexpected color type");

      end

%       disp(channel1Min);
%       disp(channel2Min);
%       disp(channel3Min);

      % Create mask based on chosen histogram thresholds
      sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
          (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
          (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
      BW = sliderBW;

      % Initialize output masked image based on input image.
      maskedRGBImage = RGB;

      % Set background pixels where BW is false to zero.
      maskedRGBImage(repmat(~BW,[1 1 3])) = 0;



    end








    function detectBall(BW, RGB)
    
      % Find connected components.
      blobAnalysis = vision.BlobAnalysis(AreaOutputPort = true,...
          CentroidOutputPort = false,...
          BoundingBoxOutputPort = true,...
          MinimumBlobArea = 200, ExcludeBorderBlobs = true)
      [areas, boxes] = step(blobAnalysis, BW)
      
      % Sort connected components in descending order by area
      [~, idx] = sort(areas, "Descend")
      
      % Get the two largest components.
      % boxes = double(boxes(idx(1:2), :))
      boxes = double(boxes(idx, :))

      % Insert labels for the coins.
      imDetectedCoins = insertObjectAnnotation(RGB, "rectangle", boxes, "penny");
      figure; 
      imshow(imDetectedCoins);

    end










  end


end

















































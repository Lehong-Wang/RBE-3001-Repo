

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
          channel1Min = 0.9;
          channel1Max = 1;
          
        case 'O'
          channel1Min = 0.031;
          channel1Max = 0.087;


        case 'Y'
          channel1Min = 0.103;
          channel1Max = 0.154;

        case 'G'
          channel1Min = 0.171;
          channel1Max = 0.215;

          otherwise
           channel1Min = 0;
          channel1Max = 1; 
          warning("Unexpected color type");

      end
        % white channel
      channel2Min = 0.235;
      channel2Max = 1.000;
        % black channel
      channel3Min = 0.35;
      channel3Max = 1.000;

%       disp(channel1Min);
%       disp(channel2Min);
%       disp(channel3Min);


      % Create mask based on chosen histogram thresholds
      sliderBW = ( (I(:,:,1) >= channel1Min) & (I(:,:,1) <= channel1Max) ) & ...
          (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
          (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
      BW = sliderBW;

      % Initialize output masked image based on input image.
      maskedRGBImage = RGB;

      % Set background pixels where BW is false to zero.
      maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

    end







    % given the black white and RGB picture from createMaskedImage
    % return a n*2 matrix of the centers of the ball in distorted image coordinate
    function centers = detectBall(BW, RGB)
    
      % Find connected components.
      blobAnalysis = vision.BlobAnalysis(AreaOutputPort = true,...
          CentroidOutputPort = false,...
          BoundingBoxOutputPort = true,...
          MinimumBlobArea = 5000, ExcludeBorderBlobs = true);
      [areas, boxes] = step(blobAnalysis, BW)
      
      % Sort connected components in descending order by area
      [~, idx] = sort(areas, "Descend");
      
      % Get the two largest components.
      % boxes = double(boxes(idx(1:2), :))
      boxes = double(boxes(idx, :));

      centers = zeros(0);
      for i = (1 : size(boxes,1))
          centers(i,1) = boxes(i,1) + boxes(i,3)/2;
          centers(i,2) = boxes(i,2) + boxes(i,4)/2;
          centers(i,3) = 10;
      end

      % Insert labels for the coins.
      imDetectedCoins = insertObjectAnnotation(RGB, "rectangle", boxes, "Ball");
      imDetectedCoins = insertObjectAnnotation(imDetectedCoins, "circle", centers, "Center");
      figure; 
      imshow(imDetectedCoins);

      centers = centers(:,1:2);

    end



    function reals = getRealCoord(centers, cam)
        cam_height = 180;
        cam_pos = [200, 0];
        center_height = 10;

        reals = zeros(0);

        for i = (1 : size(centers,1))
            pixel_center = centers(i,:)
            proj_center = cam.getWorldCoord(pixel_center)

            dis_to_cam = proj_center - cam_pos;

            real_center = proj_center - dis_to_cam * (center_height/cam_height)
            reals(i,:) = real_center;
        end
        disp(reals);
    end










  end


end

















































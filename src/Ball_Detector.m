

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

          case 'A'
              channel1Min = 0.9;
              channel1Max = 0.215;
          otherwise
          channel1Min =-0.5;
          channel1Max = 0.499; 
          warning("Unexpected color type");

      end
        % white channel
      channel2Min = 0.4;
      channel2Max = 1.000;
        % black channel
      channel3Min = 0.4;
      channel3Max = 1.000;

      disp(mod(channel1Min+0.5, 1));
      disp(mod(channel1Max+0.5, 1));
%       disp(I(:,:,1)+0.5);

      % Create mask based on chosen histogram thresholds
      % to deal will red being 0.9-1, 
      % rotate the sepctrom (+0.5) to get continuous thresholds
      sliderBW = ( (mod(I(:,:,1)+0.5, 1) >= mod(channel1Min+0.5, 1)) & (mod(I(:,:,1)+0.5, 1) <= mod(channel1Max+0.5, 1)) ) & ...
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
          BoundingBoxOutputPort = true,...  % get bounding box
          MinimumBlobArea = 5000,...        % minium area
          ExcludeBorderBlobs = false);
      [areas, boxes] = step(blobAnalysis, BW);
      
      % Sort connected components in descending order by area
      [~, idx] = sort(areas, "Descend");
      
      % get the x,y,w,h of each box (blub)
      %     x,y is top left of the box
      % boxes = double(boxes(idx(1:2), :))
      boxes = double(boxes(idx, :));

      % center of boxes
      centers = zeros(0);
      for i = (1 : size(boxes,1))
          centers(i,1) = boxes(i,1) + boxes(i,3)/2;
          centers(i,2) = boxes(i,2) + boxes(i,4)/2;
          centers(i,3) = 10;
      end

      % Insert box and center for the balls.
      labeled_img = insertObjectAnnotation(RGB, "rectangle", boxes, "Ball");
      labeled_img = insertObjectAnnotation(labeled_img, "circle", centers, "Center");
%       figure; 
%       imshow(labeled_img);
      % get center x,y coord
      centers = centers(:,1:2);
      disp(centers);

    end


    % use calibration data from Camera class to map balls to base coord
    function reals = getRealCoord(centers, cam)
        cam_height = 180;
        cam_pos = [200, 0];
        center_height = 10;

        reals = zeros(0);

        for i = (1 : size(centers,1))
            pixel_center = centers(i,:);
            proj_center = cam.getWorldCoord(pixel_center);

            dis_to_cam = proj_center - cam_pos;

            real_center = proj_center - dis_to_cam * (center_height/cam_height);
            reals(i,:) = real_center;
        end
        disp(reals);
    end



    % wrapper
    function coords = getBallPose(img, color, cam)
        [BW,RGB] = Ball_Detector.createMaskedImage(img, color);
%         imshow(BW);
%         imshow(RGB);
        try
            centers = Ball_Detector.detectBall(BW, RGB);
            coords = Ball_Detector.getRealCoord(centers, cam);
        catch exception
            getReport(exception)
            disp(["No ball with color ", color]);
            coords = zeros(0);
        end

    end








  end


end

















































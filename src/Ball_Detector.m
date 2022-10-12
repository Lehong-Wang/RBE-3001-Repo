

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
      channel2Min = 0.3;
      channel2Max = 1.000;
        % black channel
      channel3Min = 0.3;
      channel3Max = 1.000;
    
    %       disp(mod(channel1Min+0.5, 1));
    %       disp(mod(channel1Max+0.5, 1));
    
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
    
    
      se = strel('square',5);
%         se = strel('cube',3)

      maskedRGBImage = imerode(maskedRGBImage,se);
      BW = imerode(BW,se);

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
%       pause;
%       clf;
      % get center x,y coord
      centers = centers(:,1:2);
%       disp("Ball Centers:");
%       disp(centers);

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
%         disp(reals);
    end



    % wrapper
    % given original image and color and camera object
    % return a list of coordinates of balls in n*2 matrix
    function coords = getBallPose(img, color, cam)
        [BW,RGB] = Ball_Detector.createMaskedImage(img, color);
%         imshow(BW);
%         imshow(RGB);
        try
            centers = Ball_Detector.detectBall(BW, RGB);
            original_coords = Ball_Detector.getRealCoord(centers, cam);
            coords = zeros(0);
            for i = 1:size(original_coords,1)
                this_coord = original_coords(i,:);
                if Ball_Detector.checkOnBoard(this_coord)
                    coords = [coords; this_coord];
                end
            end
        catch exception
            getReport(exception);
            disp(["No ball with color ", color]);
            coords = zeros(0);
        end

    end



    % Function for debugging
    % same as getBallPose
    % plot the coordinates of balls on a graph
    function reals = plotError(img, color, cam)

        [BW,RGB] = Ball_Detector.createMaskedImage(img, color);
%         imshow(BW);
%         imshow(RGB);
        centers = Ball_Detector.detectBall(BW, RGB);
        reals = Ball_Detector.getRealCoord(centers, cam)

        x_grid_coord = zeros(0);
        y_gird_coord = zeros(0);
        for i = 1:55
            x_grid_coord(i) = 25 * mod(i-1, 5) + 50; 
            y_gird_coord(i) = 25 * floor((i-1)/5) - 125;
        end

        hold on
        plot(y_gird_coord, x_grid_coord, ".", 'MarkerSize',20);
        axis([-150 150 0 200])

        plot(0, 0, '.k', 'MarkerSize',70)
        rectangle('Position',[-20 180 40 40])

        plot(reals(:,2), reals(:,1), ".r", 'MarkerSize',20)

    end




    function on_checker = checkOnBoard(coord)
        x = coord(1);
        y = coord(2);

        on_checker = 40 < x && x < 160 ...
                    && -130 < y && y < 130;
          
    end






    % given a original image, color for computer and human piece and camera object
    % return the current layout in 3*3*2 [compter_layout; human_layout]
    %       if two ball in a cell, get a warning, ignore human ball
    function layout = getLayout(img, computer_color, human_color, cam)

        
        comp_coords = Ball_Detector.getBallPose(img, computer_color, cam);
        comp_layout = zeros(3);
        for i = 1:size(comp_coords,1)
            [row,col] = Ball_Detector.coord2Board(comp_coords(i,:));
            if row ~= 0 && col ~= 0
                comp_layout(row, col) = 1;
            end
        end


        hum_coords = Ball_Detector.getBallPose(img, human_color, cam);
        hum_layout = zeros(3);
        for i = 1:size(hum_coords,1)
            [row,col] = Ball_Detector.coord2Board(hum_coords(i,:));
            if row ~= 0 && col ~= 0
                % if conflict, ignore human ball
                if comp_layout(row, col) ~= 1
                    hum_layout(row, col) = 1;
                end
            end
        end

        layout(:,:,1) = comp_layout;
        layout(:,:,2) = hum_layout;
        disp("Layout:");
        disp(layout);

    end



    % given the coordinate of a ball in base coord
    % return    row, col in tic-tac-toe board
    %           [0,0] if out of board
    function [row,col] = coord2Board(coord)
        row_boundary = [200 140 90 50];
        col_boundary = [75 25 -25 -75];
        coord_x = coord(1);
        coord_y = coord(2);

        if coord_x > row_boundary(1) ...
            || coord_x < row_boundary(length(row_boundary)) ...
            || coord_y > col_boundary(1) ...
            || coord_y < col_boundary(length(col_boundary)) 
            
            fprintf("Out Of Bound. Coord: %d, %d\n", coord_x, coord_y);
            row = 0;
            col = 0;
        end

        for i = 1:length(row_boundary)-1
            row_upper = row_boundary(i);
            row_lower = row_boundary(i+1);
            if coord_x >= row_lower && coord_x < row_upper
                row = i;
            end

            col_upper = col_boundary(i);
            col_lower = col_boundary(i+1);
            if coord_y >= col_lower && coord_y < col_upper
                col = i;
            end
        end


    end


    % given row, col in board
    % return coord in base frame
    function coord = board2Coord(row, col)
        row_coord = [160 115 60];
        col_coord = [50 0 -50];
        coord = [row_coord(row) col_coord(col)];
    end

    

    % given image, color, camera
    % return    coord of first ball outside board
    %           [] if no extra ball
    function pickup_coord = getPickupCoord(img, color, cam)
        row_boundary = [200 50];
        col_boundary = [75 -75];
        all_coords = Ball_Detector.getBallPose(img, color, cam);
        for i = 1:size(all_coords, 1)
            coord = all_coords(i,:);
            coord_x = coord(1);
            coord_y = coord(2);
    
            if coord_x > row_boundary(1) ...
                || coord_x < row_boundary(length(row_boundary)) ...
                || coord_y > col_boundary(1) ...
                || coord_y < col_boundary(length(col_boundary)) 
                
                pickup_coord = coord;
                return
            end
        end
        warning("No extra ball with color %s", color);
        pickup_coord = zeros(0);
    end




  end


end

















































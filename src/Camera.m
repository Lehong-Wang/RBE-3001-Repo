classdef Camera < handle
    % CAMERA Example Camera class for RBE 3001 Lab 5
    %   You can add your image processing in this camera class,
    %   as well as any other functions related to the camera.
    
    properties        
        % Properties
        params;
        cam;
        cam_pose;
        cam_IS;
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam('USB Camera'); % Get camera object
            self.params = self.calibrate(); % Run Calibration Function
            [self.cam_IS, self.cam_pose] = self.getCameraPose();
        end

        function shutdown(self)
            % SHUTDOWN shutdown script which clears camera variable
            clear self.cam;
        end
      
        function params = calibrate(self)
            % CALIBRATE Calibration function
            % This function will run the camera calibration, save the camera parameters,
            % and check to make sure calibration worked as expected
            % The calibrate function will ask if you are ready. To calibrate, you must press
            % any key, then the system will confirm if the calibration is successful

            % NOTE: This uses the camcalib.m file for camera calibration. If you have placed
            % your camera calibration script elsewhere, you will need to change the command below

            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                disp("Calibrating");
                camcalib; % Change this if you are using a different calibration script
%                 camcalib_fast; % less image for faster debugging
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                msg = getReport(exception);
                disp(msg)
                disp("No camera calibration file found. Plese run camera calibration");
            end          
        end
        
        % Returns an undistorted camera image
        function img = getImage(self)
            raw_img =  snapshot(self.cam);
            [img, new_origin] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
        end

        
        function [newIs, pose] = getCameraPose(self)
            % GETCAMERAPOSE Get transformation from camera to checkerboard frame
            % This function will get the camera position based on checkerboard.
            % You should run this function every time the camera position is changed.
            % It will calculate the extrinsics, and output to a transformation matrix.
            % Keep in mind: this transformation matrix is a transformation from pixels
            % to x-y coordinates in the checkerboard frame!

            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, newIs] = undistortFisheyeImage(raw_img, self.params.Intrinsics, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img, 'PartialDetections', false);
            % 4. Compute transformation
            self.params.WorldPoints = self.params.WorldPoints(self.params.WorldPoints(:, 2) <= (boardSize(1)-1)*25, :);
            disp(self.params.WorldPoints);
            disp(boardSize);
            % 4. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, newIs);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
        end
    
 

        % ---------------------------------- lab 5 functions ----------------------------------
    
        % take in the point in image coordinate
        % return the 1*2 point coordinate in base frame of robot
        % NOTE: DO NOT use the undistorted image returned from getImage
        %       That is equivolent to doing undistortion twice, and fuck up everything !!!  
        function coord = getWorldCoord(self, image_coord)
            % get rotation and translation from matrix
            rot_mat = self.cam_pose(1:3, 1:3);
            trans_mat = self.cam_pose(1:3, 4);
            cam_intrinsics = self.params.Intrinsics;

            checker_board_coord = pointsToWorld(cam_intrinsics, rot_mat, trans_mat, image_coord);
            disp(checker_board_coord);
            T_base_checker = [ 0 1 75;
                               1 0 -100;
                               0 0 1;];
            % make checker_board_coord 3*1
            checker_board_coord = [transpose(checker_board_coord); 1];
            base_coord = T_base_checker * checker_board_coord;
            % take out the extra row
            coord = transpose(base_coord(1:2, 1));
%             disp(coord);

        end 

    
    
    
    
    
    
    
    
    end
end


clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs); 
try
%     % calibrate a camera
%     camera = Camera();
    % load a already calibrated camera object
    load("cam_obj.mat")
    disp("Place the balls");
    pause;

    img = snapshot(webcam("USB Camera"));
%     imshow(img);


    drop_coord_red = [0 -100];
    drop_coord_orange = [50 -100];
    drop_coord_yellow = [0 100];
    drop_coord_green = [50 100];
    drop_mat = [drop_coord_red;
                drop_coord_orange;
                drop_coord_yellow;
                drop_coord_green;];
    color_array = ['R' 'O' 'Y' 'G'];


    coords = Ball_Detector.plotError(img, 's', camera);
%     coords = Ball_Detector.getBallPose(img, 's', camera);
%     while (size(coords,1) ~= 0)

        robot.pick_up_balls(coords, drop_coord_red);
%     end

% 
%     tic
%     while (size(coords,1) ~= 0)
%       if (toc > 1/30)
%       img = snapshot(webcam("USB Camera"));
%       coords = Ball_Detector.getBallPose(img, 's', camera);
%       tracking_coord = [coords(1,1); coords(1,2); 100];
%       robot.run_trajectory(tracking_coord, 1/30);
% 
%       tic
%       end
%     end







%     % live sorting
%     % take new picture after each sort
% 
%     coords = Ball_Detector.getBallPose(img, 'A', camera);
% 
%     while (size(coords,1) ~= 0)
%         img = snapshot(webcam("USB Camera"));
%         imshow(img);
% 
%         for i = (1 : length(color_array))
%             color = color_array(i);
%             drop = drop_mat(i, :);
%             coords = Ball_Detector.getBallPose(img, color, camera);
%             if (size(coords,1) ~= 0)
%                 robot.pick_up_balls(coords(1,:), drop_mat(i, :));
%                 break;
%             end
%         end
%         coords = Ball_Detector.getBallPose(img, 'A', camera);
% 
%     end






% 
%     coords = Ball_Detector.getBallPose(img, 'R', camera);
%     robot.pick_up_balls(coords, drop_coord_red);
% 
%     coords = Ball_Detector.getBallPose(img, 'O', camera);
%     robot.pick_up_balls(coords, drop_coord_orange);
% 
%     coords = Ball_Detector.getBallPose(img, 'Y', camera);
%     robot.pick_up_balls(coords, drop_coord_yellow);
% 
%     coords = Ball_Detector.getBallPose(img, 'G', camera);
%     robot.pick_up_balls(coords, drop_coord_green);






catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

robot.shutdown()








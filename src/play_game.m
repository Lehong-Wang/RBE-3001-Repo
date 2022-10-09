
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
 

%     robot.pick_up_balls([100 100], [100 -100]);


    % load camera object
    load("cam_obj.mat");
    
    human_color = 'Y';
    computer_color = 'G';

    img = snapshot(camera.cam);
    layout = Ball_Detector.getLayout(img, computer_color, human_color, camera);
    
    while ~ Game.checkWin(layout(:,:,1), layout(:,:,2))
        img = snapshot(camera.cam);
        layout = Ball_Detector.getLayout(img, computer_color, human_color, camera)
        pickup_coord = Ball_Detector.getPickupCoord(img, computer_color, camera);

        while length(pickup_coord) == 0
            disp("Place Extra balls for robot");
            pause;
            img = snapshot(camera.cam);
            pickup_coord = Ball_Detector.getPickupCoord(img, computer_color, camera);
        end
        imshow(img);
%         pause;

        computer_move_pos = Game.computerMove(layout(:,:,1), layout(:,:,2))
        % board filled
        if length(computer_move_pos) == 0
            disp("Board Filled");
            break;
        end
        % human / computer already won
        if length(computer_move_pos) == 1
            disp("Already Won");
            break;
        end

        disp("Robor Playing");

        place_coord = Ball_Detector.board2Coord(computer_move_pos(1), computer_move_pos(2));

        robot.pick_up_balls(pickup_coord, place_coord);
        pause(1);
        
        img = snapshot(camera.cam);
        imshow(img);
        layout = Ball_Detector.getLayout(img, computer_color, human_color, camera)
        if Game.checkWin(layout(:,:,1), layout(:,:,2))
            disp("Robot Won");
            break;
        end
        empty_layout = ~(layout(:,:,1) | layout(:,:,2));
        if ~any(any(empty_layout))
            disp("Board Filled");
            break;
        end



        disp("Place your move");
        pause;

        img = snapshot(camera.cam);
        layout = Ball_Detector.getLayout(img, computer_color, human_color, camera)

    end


    
 





catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

robot.shutdown()

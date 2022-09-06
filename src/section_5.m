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

% Create a PacketProcessor object to send data to the nucleo firmware
robot = Robot(myHIDSimplePacketComs); 
try
    SERV_ID = 1848;            % we will be talking to server ID 1848 on
    % the Nucleo
    SERVER_ID_READ =1910;% ID of the read packet
    DEBUG   = true;          % enables/disables debug prints

    status_mat = robot.measured_js(1,0);
    pos = status_mat(1,:);
    disp(pos);


    pos_mat = [32.1600   87.3400  -51.7300;
                -15.1200   74.1400   20.5100;
                -60.4800   70.0600  -13.0900;
                -11.0400  -42.2600  -81.4900];

    for a = 1:height(pos_mat)
        target_pos = pos_mat(a,:);
        robot.interpolate_jp(SERV_ID, [0,0,0], 1000);
        pause(1.2);
        robot.interpolate_jp(SERV_ID, target_pos, 1000);
        pause(1.2);
        disp(a);
    
    end
    robot.interpolate_jp(SERV_ID, [0,0,0], 1000);


catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown();

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
    SERV_ID = 1848;
    SERVER_ID_READ =1910;
  
%     robot.servo_jp([30 0 0]);

    tar_0 = [100; 0; 195];
    tar_1 = [70; 100; 20];
    tar_2 = [70; -100; 40];

    tar = [tar_0 tar_1 tar_2 tar_0];

    % record_mat = robot.run_trajectory(tar, 2)
%     robot.servo_jp([40,60,80]);
%     pause(0.5);

    record_mat = robot.run_function_trajectory(@Traj_Planner.verticle_half_circle ,-150, 150, 10)

    robot.plot_record(record_mat)


    % time_vector = record_mat(:,1);
    % x_vector = record_mat(:,2);
    % y_vector = record_mat(:,3);
    % z_vector = record_mat(:,4);

    % plot3(x_vector, y_vector, z_vector);
    % axis([-100 200 -200 200 0 300]);









    
 





catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

robot.shutdown()

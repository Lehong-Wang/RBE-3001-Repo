
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
  
%   a = robot.measured_js(1,1)
%   b = robot.measured_cp()
%   robot.servo_jp([40,50,-20]);
%   c = robot.setpoint_js()
%   d = robot.setpoint_cp()
%   e = robot.goal_cp()


    [make_plot, plot_pos, plot_arm] = Model;

    make_plot();
    for i = (1:1000)
        t_mat = robot.get_all_trans_mat()
        plot_arm(t_mat)
        pause(.1);
    end



    
 





catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

robot.shutdown()

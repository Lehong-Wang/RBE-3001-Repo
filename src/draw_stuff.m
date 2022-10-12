
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
    global end_effector_T;

    robot.run_trajectory([100;0;195], 1);
    disp("Place the Marker");
    pause;
    robot.run_trajectory([125;-45;40], 2);
    record1 = robot.run_function_trajectory(@Traj_Planner.draw_heart_upper, -45, 45, 5, false);
    record2 = robot.run_function_trajectory(@Traj_Planner.draw_heart_lower, 45, -45, 5, false);

    record = [record1; record2];

    robot.plot_record(record);

    robot.run_trajectory([125;-45;40], 1);
    robot.run_trajectory([100;0;195], 2);






catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

robot.shutdown()

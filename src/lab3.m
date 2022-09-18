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
model2 = Model2();

while true
    matrix = robot.measured_cp();
    x = matrix(1,4);
    y = matrix(2,4);
    z = matrix(3,4);
    disp("joint data from ik")
    calJointData = robot.ik3001([x y z])
    disp("transformation matrix")
    robot.fk3001(transpose(calJointData))
    disp("measure_cp")
    robot.measured_cp()
    pause(0.5)
end 


% Clear up memory upon termination
robot.shutdown()

%toc

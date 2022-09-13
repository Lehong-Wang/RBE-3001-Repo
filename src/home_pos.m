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

  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');
% following is home position code and rms value calculatiion

M = cell(10, 1);
for i = [1 : 10] 
    
    robot.interpolate_jp([15,20,30],1000);
    pause(2);
    robot.servo_jp([0,0,0]);
    pause(0.5);
    M{i} = robot.measured_cp();

end

for a = [1:10]
    disp(M{a})
end

writecell(M,'home_position.csv');
filename = 'home_position.csv';
% N = cell(10,1);
N = csvread(filename);

O = zeros(10,3);

for i = [1:10]
    O(i, 1) = N(i, 13);
    O(i, 2) = N(i, 14);
    O(i, 3) = N(i, 15);
end 

disp(O)

for i = [1:10]
    plot3(O(i,1),O(i,2),O(i,3), 'o');
    hold on
end
hold off

tic
while toc < 1.5
  robot.goal_cp()
  robot.setpoint_cp();
    robot.measured_cp()
end

mean_value = mean(O);
rms_value = rms(O);

disp(mean_value)
disp(rms_value)

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

% toc

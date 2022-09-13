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



% % Motion Planning Arm Movement
Pos1 = [0 0 0];
Pos2 = [0 0 55];
Pos3 = [0 30 0];
Pos4 = [0 0 0];

M = zeros(10000, 7)

t0 = clock;
ms = 0;
ms1 = 0;
x = 12000;
i = 1;
once = 1;
while ms <= x
    ms = etime(clock,t0) * 1000;
    
    if ms < 3000
        if(once)
            robot.interpolate_jp(Pos1, 3000);
            once = 0;
        end
    elseif ms > 3000 && ms < 6000
        if(once == 0)
            robot.interpolate_jp(Pos2, 3000);
            once = 1;
        end
        
    elseif ms > 6000 && ms < 9000
        if(once)
            robot.interpolate_jp(Pos3, 3000);
            once = 0;
        end
    elseif ms > 9000 && ms < 12000
        if(once == 0)
            robot.interpolate_jp(Pos4, 3000);
            once = 1;
        end
    end
        
    angle = robot.measured_js(1,0); 
    fkAngle = transpose(angle(1, :)) * 2 * pi/360;
    M(i, 1) = ms;
    M(i, 2) = angle(1,1);
    M(i, 3) = angle(1,2);
    M(i, 4) = angle(1,3);
    endpoint = Matrix.fk3001(fkAngle)* [0; 0; 0; 1];
    M(i,5) = endpoint(1,1);
    M(i,6) = endpoint(2,1);
    M(i,7) = endpoint(3,1);
    
    
    i = i + 1;
end

writematrix(M,'motion_planning.csv');


filename1 = 'motion_planning.csv';
motionData = csvread(filename1); 
time = motionData(:,1);
joint1 = motionData(:,2);
joint2 = motionData(:,3);
joint3 = motionData(:,4);

subplot(3,2,1)
plot(time, joint1);
hold on
plot(time, joint2);
plot(time, joint3);
hold off
title("Joint Angles vs Time");
xlabel('Time(ms)') ;
ylabel('Angle(degrees)'); 
legend('Joint1', 'Joint2', 'Joint3');

subplot(3,2,2)
endEffectorPosX = motionData(:, 5);
endEffectorPosZ = motionData(:, 7);
plot(time, endEffectorPosX);
hold on
plot(time, endEffectorPosZ);
hold off
title("End Effector position vs Time");
xlabel('Time(ms)') ;
ylabel('Position(mm)'); 
legend('X Position', 'Z Position');

subplot(3,2,3)
xPos = motionData(:,5);
yPos = motionData(:,6);
zPos = motionData(:,7);
plot(xPos, zPos);
hold on
plot(100,195,'r.');
plot(60,114,'r.');
plot(135,133,'r.');
hold off
title("X-Z plane trajectory");
xlabel('X position(mm)') ;
ylabel('Z position(mm)'); 

subplot(3,2,4)
plot(xPos, yPos);
title("X-Y plane trajectory");
xlabel('X position(mm)') ;
ylabel('Y position(mm)'); 

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

% toc
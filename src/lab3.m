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

%{
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
%}


% 3
  ifReach = 0;  %set a boolean if Reach the set point 
  position1 = []; %create an empty matrix to store the data of joints position
  timeStamp1 = [];
  jointAngle1 = [];
  input1 = robot.ik3001([140 -108 141]);
  robot.interpolate_jp(transpose(input1), 1500);  %using function to move arm with interpolate of 1.5s
  tStart1 = tic;
  while( ifReach < 1)   %using while loop
      currentT = robot.measured_cp(); %record current position
      currentPos = currentT(1:3,4);
      jointA1 = robot.ik3001(currentPos);
      tCurr1 = toc(tStart1);
      position1 = [position1; transpose(currentPos)]; %input position data to matrix
      timeStamp1 = [timeStamp1; tCurr1];
      jointAngle1 = [jointAngle1; transpose(jointA1)];
      if currentT(2,4) <= -105      %if reach the set point, stop the recording
          ifReach = 1;
      end
  end


  ifReach = 0;  %set a boolean if Reach the set point 
  position2 = []; %create an empty matrix to store the data of joints position
  timeStamp2 = [];
  jointAngle2 = [];
  input2 = robot.ik3001([145 1.8 84.8]);
  robot.interpolate_jp(transpose(input2), 1500);  %using function to move arm with interpolate of 1.5s
  while( ifReach < 1)   %using while loop
      currentT = robot.measured_cp(); %record current position
      currentPos = currentT(1:3,4);
      jointA2 = robot.ik3001(currentPos);
      tCurr2 = toc(tStart1);
      position2 = [position2; transpose(currentPos)]; %input position data to matrix
      timeStamp2 = [timeStamp2; tCurr2];
      jointAngle2 = [jointAngle2; transpose(jointA2)];
      if currentT(2,4) >= -4       %if reach the set point, stop the recording
          ifReach = 1;
      end
  end

  
  ifReach = 0;  %set a boolean if Reach the set point 
  position3 = []; %create an empty matrix to store the data of joints position
  timeStamp3 = [];
  jointAngle3 = [];
  input3 = robot.ik3001([100 132.7 149.1]);
  robot.interpolate_jp(transpose(input3), 1500);  %using function to move arm with interpolate of 1.5s
  while( ifReach < 1)   %using while loop
      currentT = robot.measured_cp(); %record current position
      currentPos = currentT(1:3,4);
      jointA3 = robot.ik3001(currentPos);
      tCurr3 = toc(tStart1);
      position3 = [position3; transpose(currentPos)]; %input position data to matrix
      timeStamp3 = [timeStamp3; tCurr3];
      jointAngle3 = [jointAngle3; transpose(jointA3)];
      if currentT(2,4) >= 130       %if reach the set point, stop the recording
          ifReach = 1;
      end
  end

  
  ifReach = 0;  %set a boolean if Reach the set point 
  position4 = []; %create an empty matrix to store the data of joints position
  timeStamp4 = [];
  jointAngle4 = [];
  input1 = robot.ik3001([140 -108 141]);
  robot.interpolate_jp(transpose(input1), 1500);  %using function to move arm with interpolate of 1.5s
  while( ifReach < 1)   %using while loop
      currentT = robot.measured_cp(); %record current position
      currentPos = currentT(1:3,4);
      jointA4 = robot.ik3001(currentPos);
      tCurr4 = toc(tStart1);
      position4 = [position4; transpose(currentPos)]; %input position data to matrix
      timeStamp4 = [timeStamp4; tCurr4];
      jointAngle4 = [jointAngle4; transpose(jointA4)];
      if currentT(2,4) <= -106      %if reach the set point, stop the recording
          ifReach = 1;
      end
  end

  positionT = [position1;position2;position3;position4];
  timeT = [timeStamp1; timeStamp2; timeStamp3; timeStamp4];
  jointT = [jointAngle1; jointAngle2; jointAngle3; jointAngle4];
  
figure
plot(timeT,positionT(:,1))
hold on
plot(timeT,positionT(:,2))
hold on
plot(timeT,positionT(:,3))
title("Position VS Time")
ylabel("Position(mm)");
xlabel("Time(s)");
legend('X','Y','Z');
hold off;

figure
plot(timeT,jointT(:,1))
hold on
plot(timeT,jointT(:,2))
hold on
plot(timeT,jointT(:,3))
title("Joint Angles VS Time");
xlabel("Time(s)");
ylabel("Joint Angle(degree)");
legend('Joint1','Joint2','Joint3');
hold off;

figure
plot3(positionT(:,1),positionT(:,2),positionT(:,3))
title("3D Position Trajectory");
xlabel('X-Position');
ylabel('Y-Position');
zlabel('Z-Position');
% Clear up memory upon termination
robot.shutdown()

%toc

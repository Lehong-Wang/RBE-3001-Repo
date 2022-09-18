%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
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

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  viaPts = [0,40,0];

  for k = viaPts
      %tic
      packet = zeros(15, 1, 'single');
      packet(1) = 1000;%one second time
      packet(2) = 0;%linear interpolation
      packet(3) = k;
      packet(4) = 0;% Second link to 0
      packet(5) = 0;% Third link to 0

      % Send packet to the server and get the response      
      %robot.write sends a 15 float packet to the micro controller
       robot.write(SERV_ID, packet); 
       %robot.read reads a returned 15 float backet from the micro controller.
       returnPacket = robot.read(SERVER_ID_READ);
      %toc

      if DEBUG
          disp('Sent Packet:')
          disp(packet);
          disp('Received Packet:');
          disp(returnPacket);
      end
      
      %toc
      pause(1) 
      
  end
  
  % Closes then opens the gripper
  robot.closeGripper()
  pause(1)
  robot.openGripper()

  

  % Q4
  % First Trial
  ifReach = 0;  %set a boolean if Reach the set point 
  trajectory1 = []; %create an empty matrix to store the data of joints position
  timeStamps1 = []; %create an empty matrix to store the data of time stamps
  robot.interpolate_jp([45 0 0], 2000)  %using function to move arm with interpolate of 2s
  tStart1 = tic;    %set tic for recording time
  while( ifReach < 1)   %using while loop
      currentPos = robot.measured_js(1, 0); %record current position
      tCurrent1 = toc(tStart1);
      trajectory1 = [trajectory1; currentPos(1,:)]; %input position data to matrix
      timeStamps1 = [timeStamps1; tCurrent1];   %input time stamps data to matrix

      if currentPos(1,1) >= 44      %if reach the set point, stop the recording
          ifReach = 1;
      end
  end

  % Second Trial
  robot.interpolate_jp([0 0 0],2000)
  pause(2.5)
  ifReach = 0;
  trajectory2 = [];
  timeStamps2 = [];
  robot.interpolate_jp([45 0 0], 2000)
  tStart2 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent2 = toc(tStart2);
      trajectory2 = [trajectory2; currentPos(1,:)];
      timeStamps2 = [timeStamps2; tCurrent2];

      if currentPos(1,1) >= 44
          ifReach = 1;
      end
  end

  % Third Trial
  robot.interpolate_jp([0 0 0], 2000)
  pause(2.5)
  ifReach = 0;
  trajectory3 = [];
  timeStamps3 = [];
  robot.interpolate_jp([45 0 0], 2000)
  tStart3 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent3 = toc(tStart3);
      trajectory3 = [trajectory3; currentPos(1,:)];
      timeStamps3 = [timeStamps3; tCurrent3];

      if currentPos(1,1) >= 44
          ifReach = 1;

      end
  end

  % Non-interpolation First Trial
  robot.interpolate_jp([0 0 0], 2000)
  pause(2.5)
  ifReach = 0;
  trajectory1_1 = [];
  timeStamps1_1 = [];
  robot.servo_jp([45 0 0])
  tStart1_1 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent1_1 = toc(tStart1_1);
      trajectory1_1 = [trajectory1_1; currentPos(1,:)];
      timeStamps1_1 = [timeStamps1_1; tCurrent1_1];

      if currentPos(1,1) >= 44
          ifReach = 1;

      end
  end

  % Non-interpolation Second Trial
  robot.interpolate_jp([0 0 0], 2000)
  pause(2.5)
  ifReach = 0;
  trajectory2_1 = [];
  timeStamps2_1 = [];
  robot.servo_jp([45 0 0])
  tStart2_1 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent2_1 = toc(tStart2_1);
      trajectory2_1 = [trajectory2_1; currentPos(1,:)];
      timeStamps2_1 = [timeStamps2_1; tCurrent2_1];

      if currentPos(1,1) >= 44
          ifReach = 1;

      end
  end

  % Non-interpolation Third Trial
  robot.interpolate_jp([0 0 0], 2000)
  pause(2.5)
  ifReach = 0;
  trajectory3_1 = [];
  timeStamps3_1 = [];
  robot.servo_jp([45 0 0])
  tStart3_1 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent3_1 = toc(tStart3_1);
      trajectory3_1 = [trajectory3_1; currentPos(1,:)];
      timeStamps3_1 = [timeStamps3_1; tCurrent3_1];

      if currentPos(1,1) >= 44
          ifReach = 1;

      end
  end
  
  m1 = [timeStamps1(:,1), trajectory1(:,1:3)];
  writematrix(m1, 'Timestamp and joint values.csv')

  % plot for the first joint
  subplot(2,3,1);   %plot the first plot of 2*3 grid
  plot(timeStamps1, trajectory1(:,1))
  hold on
  plot(timeStamps2, trajectory2(:,1))
  hold on
  plot(timeStamps3, trajectory3(:,1))
  hold on
  plot(timeStamps1_1, trajectory1_1(:,1))
  hold on
  plot(timeStamps2_1, trajectory2_1(:,1))
  hold on
  plot(timeStamps3_1, trajectory3_1(:,1))
  hold on
  ylim([-1 50])
  xlabel("time(s)");
  ylabel("postion value(degree)");
  title("Motion Profile of the First Joint")
  legend('1st Trial','2nd Trial','3rd Trial', '1st Trial(non-interpolation)','2nd Trial(non-interpolation)','3rd Trial(non-interpolation)')
  

  % plot for the second joint
  subplot(2,3,2);   %plot the second plot of 2*3 grid
  plot(timeStamps1, trajectory1(:,2))
  ylim([-1 50])
  xlabel("time(s)");
  ylabel("postion value(degree)");
  title("Motion Profile of the Second Joint")

  % plot for the third joint
  subplot(2,3,3);   %plot the third plot of 2*3 grid
  plot(timeStamps1, trajectory1(:,3))
  ylim([-1 50])
  xlabel("time(s)");
  ylabel("postion value(degree)");  
  title("Motion Profile of the Third Joint")


  %Q5
  subplot(2,3,4);   %plot the fourth plot of 2*3 grid(time stamps)
  histogram(diff(timeStamps1))
  xlabel("time(s)");
  ylabel("frequency(packets)");
  title("Reading Increment Frequency")

  subplot(2,3,5);   %plot the fifth plot of 2*3 grid(time stamps)
  histogram(diff(timeStamps1(3:200,1)))
  xlabel("time(s)");
  ylabel("frequency(packets)");
  title("Reading Increment Frequency(smaller range)")
  
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

%toc

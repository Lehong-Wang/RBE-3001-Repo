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

  % Set the arm to the configuration point and record the trajectory
  % #1 configuration 
  ifReach = 0;
  trajectory1 = [];
  timeStamps1 = [];
  robot.interpolate_jp([31.2 55.42 -91.33], 2000)
  tStart1 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent1 = toc(tStart1);
      trajectory1 = [trajectory1; currentPos(1,:)];
      timeStamps1 = [timeStamps1; tCurrent1];

      if currentPos(1,1) >= 30
          ifReach = 1;
      end
  end

  % #1 configuration non-interpolation
  robot.interpolate_jp([0 0 0],2000) % return to zero position 
  pause(2.5)
  ifReach = 0;
  trajectory0 = [];
  timeStamps0 = [];
  robot.servo_jp([31.2 55.42 -91.33])
  tStart0 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent0 = toc(tStart0);
      trajectory0 = [trajectory0; currentPos(1,:)];
      timeStamps0 = [timeStamps0; tCurrent0];

      if currentPos(1,3) <= -90
          ifReach = 1;
      end
  end

  % #2 configuration 
  robot.interpolate_jp([0 0 0],2000)
  pause(2.5)
  ifReach = 0;
  trajectory2 = [];
  timeStamps2 = [];
  robot.interpolate_jp([54.72 28.3 -91.57], 2000)
  tStart2 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent2 = toc(tStart2);
      trajectory2 = [trajectory2; currentPos(1,:)];
      timeStamps2 = [timeStamps2; tCurrent2];

      if currentPos(1,1) >= 54
          ifReach = 1;
      end
  end

  % #3 configuration 
  robot.interpolate_jp([0 0 0],2000)
  pause(2.5)
  ifReach = 0;
  trajectory3 = [];
  timeStamps3 = [];
  robot.interpolate_jp([-44.16 38.06 0.33], 2000)
  tStart3 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent3 = toc(tStart3);
      trajectory3 = [trajectory3; currentPos(1,:)];
      timeStamps3 = [timeStamps3; tCurrent3];

      if currentPos(1,2) >= 37
          ifReach = 1;
      end
  end

  % #4 configuratoin 
  robot.interpolate_jp([0 0 0],2000)
  pause(2.5)
  ifReach = 0;
  trajectory4 = [];
  timeStamps4 = [];
  robot.interpolate_jp([-26.88 38.54 -55.57], 2000)
  tStart4 = tic;
  while( ifReach < 1)
      currentPos = robot.measured_js(1, 0);
      tCurrent4 = toc(tStart4);
      trajectory4 = [trajectory4; currentPos(1,:)];
      timeStamps4 = [timeStamps4; tCurrent4];

      if currentPos(1,1) <= -26
          ifReach = 1;
      end
  end

  % Task finished, go back to zero point
  robot.interpolate_jp([0 0 0],2000)
  pause(2.5)


  % plot for the first configuration
  subplot(2,2,1);git push --set-upstream origin Part4
  plot(timeStamps1, trajectory1(:,1))
  hold on
  plot(timeStamps1, trajectory1(:,2))
  hold on
  plot(timeStamps1, trajectory1(:,3))
  hold on
  plot(timeStamps0, trajectory0(:,1))
  hold on
  plot(timeStamps0, trajectory0(:,2))
  hold on
  plot(timeStamps0, trajectory0(:,3))
  hold on
  xlabel("time(s)");
  ylabel("postion value(degree)");
  title("Motion Profile of the First Configuration")
  legend('1st Joint','2nd Joint','3rd Joint', ...
      '1st Joint(non-interpolation)','2nd Joint(non-interpolation)','3rd Joint(non-interpolation)')

  % plot for the second configuration
  subplot(2,2,2);
  plot(timeStamps2, trajectory2(:,1))
  hold on
  plot(timeStamps2, trajectory2(:,2))
  hold on
  plot(timeStamps2, trajectory2(:,3))
  hold on
  xlabel("time(s)");
  ylabel("postion value(degree)");
  title("Motion Profile of the Second Configuration")
  legend('1st Joint','2nd Joint','3rd Joint')

  % plot for the third configuration
  subplot(2,2,3);
  plot(timeStamps3, trajectory3(:,1))
  hold on
  plot(timeStamps3, trajectory3(:,2))
  hold on
  plot(timeStamps3, trajectory3(:,3))
  hold on
  xlabel("time(s)");
  ylabel("postion value(degree)");
  title("Motion Profile of the Third Configuration")
  legend('1st Joint','2nd Joint','3rd Joint')

  % plot for the fourth configuration
  subplot(2,2,4);
  plot(timeStamps4, trajectory4(:,1))
  hold on
  plot(timeStamps4, trajectory4(:,2))
  hold on
  plot(timeStamps4, trajectory4(:,3))
  hold on
  xlabel("time(s)");
  ylabel("postion value(degree)");
  title("Motion Profile of the Fourth Configuration")
  legend('1st Joint','2nd Joint','3rd Joint')


  % Get the configuration joint value
  pos = robot.measured_js(1,0);


  % Four configurations 
  %[31.2 55.42 -91.33]
  %[54.72 28.3 -91.57]
  %[-44.16 88.06 -91.33]
  %[-26.88 88.54 -55.57]

  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

%toc


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

    r = randi(90,[1,10]);   % first servo(-90,90)
    p = randi(45,[1,10]);   % second servo(-45,45)
    k = randi(45,[1,10]);   % thrid servo(-90,45)
    newT = zeros(4,4);
    robot.interpolate_jp([0 0 0],2000);
    pause(2.5);
    posVector = zeros(3,10);
    
    for i = 1:10

        inputArray = [r(i), p(i),k(i)];
        robot.interpolate_jp(inputArray,1000);
        pause(1.5);
        robot.interpolate_jp([0 0 0],1000);
        pause(1.5);
        returnT = robot.measured_cp();
        newT(:,:,i) = returnT 
        posVector(:,i) = [newT(1,4,i);newT(2,4,i);newT(3,4,i)] 
        
    end
    
    % plot a 3D position plot
    X = posVector(1,:);
    Y = posVector(2,:);
    Z = posVector(3,:);
    plot3(X,Y,Z,'d')
    axis([95 105 -5 5 190 200]);

    % calculate the RMS
        N = 10;
        avgPosX = sum(posVector(1,:))/N 
        avgPosY = sum(posVector(2,:))/N
        avgPosZ = sum(posVector(3,:))/N
        RMS_X = sqrt((1/N)*sum(abs(posVector(1,:)).^2))
        RMS_Y = sqrt((1/N)*sum(abs(posVector(2,:)).^2))
        RMS_Z = sqrt((1/N)*sum(abs(posVector(3,:)).^2))
   




catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

%toc
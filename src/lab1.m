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
      tic
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
      toc

      if DEBUG
          disp('Sent Packet:')
          disp(packet);
          disp('Received Packet:');
          disp(returnPacket);
      end
      
      toc
      pause(1) 
      
  end
  
  % Closes then opens the gripper
  robot.closeGripper()
  pause(1)
  robot.openGripper()
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

toc

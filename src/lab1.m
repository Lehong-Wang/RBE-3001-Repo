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
SERV_ID = 1848;            % we will be talking to server ID 1848 on
% the Nucleo
SERVER_ID_READ =1910;% ID of the read packet
DEBUG   = true;          % enables/disables debug prints
try
 
  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');
  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware.
  robot.servo_jp(SERV_ID, [0,0,0]);
  pause(1);
  %home pose
  %robot.interpolate_jp(SERV_ID, [60,0,0], 5000);
  %turn to 60 degree in 5s
  robot.servo_jp(SERV_ID, [50,50,50]);
  %pause(1);
  for a=1:5
      pause(0.1);
  robot.measured_js(1,1);
  end
  %return matrix
  
  robot.setpoint_js();
  
  robot.goal_js();

% 
%   robot.interpolate_jp(SERV_ID, [0,0,0], 1000);
%   pause(1);
%   plot_graph(robot, SERV_ID, [50,50,50], 1000);








catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

% toc

function plot_graph(robot, SERV_ID, pos, time)
    robot.interpolate_jp(SERV_ID, [0,0,0], time);
    pause(1);

    robot.interpolate_jp(SERV_ID, pos, time);
    
    data_tab = zeros(100,4);
    i = 1;
    tic
    while toc < time/1000
        pause(0.1);
        time_stamp = toc;
        disp(time_stamp);

        measure_mat = robot.measured_js(1,0);
        loc = transpose(measure_mat(:,1));
        disp(loc);
        data_tab(i,1) = time_stamp;
        data_tab(i,2) = loc(1);
        data_tab(i,3) = loc(2);
        data_tab(i,4) = loc(3);
        i = i+1;
        

    end
    disp(data_tab)
    plot(data_tab(:,1), data_tab(:,2));
    writematrix(data_tab, 'Robot_data.csv');
end






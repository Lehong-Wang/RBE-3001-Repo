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

    packet = zeros(15, 1, 'single');
    
    robot.interpolate_jp(SERV_ID, [0,0,0], 1000);
    pause(1.5);

    move_and_plot(robot, [45,0,0], 3000);    % Plot the movement of the arm
    pause(1);

    robot.interpolate_jp(SERV_ID, [0,0,0], 1000);
    pause(1.5);

    move_and_plot(robot, [45,0,0], 3000);    % Plot the movement of the arm
    pause(1);

    robot.interpolate_jp(SERV_ID, [0,0,0], 1000);
    pause(1.5);
    move_and_plot(robot, [45,0,0], 3000);    % Plot the movement of the arm
    pause(1);

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown();

% Function to move the robot using interpolation and to plot the 
% data in a graph and a .csv file
function move_and_plot(robot, targets, time)
    time=1;
    SERV_ID = 1848;          
    tab_data = zeros(10, 4);
%     tab_row = zeros(1,4);
    current_time = 0;
    i = 1;
    j = 1;
    ploting_time = (time+500)/1000;
    robot.servo_jp(SERV_ID, targets);   % Call interpolation function
    
    tic     % Start the timer
    while current_time < ploting_time   
        if mod(j,20) == 0
            status_mat = robot.measured_js(1,0);
            pos_mat = status_mat(1,:);
            %         disp(current_time)
            %         disp(pos_mat)
            % Add the data to the matrix
            tab_data(i,1) = current_time;
            tab_data(i,2) = pos_mat(1);
            tab_data(i,3) = pos_mat(2);
            tab_data(i,4) = pos_mat(3);
            i = i + 1;      % Increment the index
            current_time = toc;     % Set current time
        end
        j = j + 1;

    end
    disp(tab_data)      % Display the matrix

    writematrix(tab_data, 'pos_data.csv');      % Write the data to the .csv file
    
    % Plot the data
    hold on
    subplot(3,1,1);
    plot(tab_data(:,1), tab_data(:,2));
    title("Motor 1");
    xlabel("Time (s)");
    ylabel("Motor Position (deg)");
    legend(["First Run", "Second Run", "Third Run"]);

    hold on
    subplot(3,1,2);
    plot(tab_data(:,1), tab_data(:,3));
    title("Motor 2");
    xlabel("Time (s)");
    ylabel("Motor Position (deg)");
    legend(["First Run", "Second Run", "Third Run"]);

    hold on
    subplot(3,1,3);
    plot(tab_data(:,1), tab_data(:,4));
    title("Motor 3");
    xlabel("Time (s)");
    ylabel("Motor Position (deg)");
    legend(["First Run", "Second Run", "Third Run"]);


end













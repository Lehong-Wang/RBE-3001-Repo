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

    
    robot.interpolate_jp(SERV_ID, [0,0,0], 1000);
    pause(1);

    move_and_plot(robot, [45,0,0], 1000);    % Plot the movement of the arm

catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown();

% Function to move the robot using interpolation and to plot the 
% data in a graph and a .csv file
function move_and_plot(robot, targets, time)
    SERV_ID = 1848;          
    tab_data = zeros(10, 4);
%     tab_row = zeros(1,4);
    current_time = 0;
    i = 1;
    j = 1;
    ploting_time = (time+500)/1000;
    robot.interpolate_jp(SERV_ID, targets, time);   % Call interpolation function
    
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
    plot(tab_data(:,1), tab_data(:,2));
    plot(tab_data(:,1), tab_data(:,3));
    plot(tab_data(:,1), tab_data(:,4));
    hold off
    legend({"Motor 1", "Motor 2", "Motor 3"});

    tab_data_time = tab_data(:,1);

    % Create histogram using MATLAB function
    histogram(tab_data_time);

    % UNUSED CODE

    %     time_interval = 0.1;
%     frequency_data = zeros(1,2);
%     tab_data_time = tab_data(:,1);
% 
% 
%     for t = (1:length(tab_data_time))
%         current_t = tab_data_time(t);
%         his_index = floor(current_t / time_interval)+1;
%         try
%             frequency_data(his_index) = frequency_data(his_index)+1;
%         catch exception
%             frequency_data(his_index) = 1;
%         end
%     end
% 
% 
%     disp(frequency_data);
%     time_interval_vector = 0:time_interval:(length(frequency_data)-1)*time_interval;
% 
% %     time_interval_vector = time_interval:time_interval:(length(frequency_data))*time_interval;
% 
% %     plot(time_interval_vector, frequency_data);

end












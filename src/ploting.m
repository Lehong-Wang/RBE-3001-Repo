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


%     robot.interpolate_jp(SERV_ID, [0,0,0], 1000);
%     pause(1);
% 
%     move_and_plot(robot, [45,0,0], 1000);
% 


    pos_mat = [-11.0400  -42.2600  -81.4900];
%     [32.1600   87.3400  -51.7300]
%                 -15.1200   74.1400   20.5100;
%                 -60.4800   70.0600  -13.0900;
%                 -11.0400  -42.2600  -81.4900];

    for a = 1:height(pos_mat)
        target_pos = pos_mat(a,:);
        robot.interpolate_jp(SERV_ID, [0,0,0], 1000);
        pause(1.2);
        move_and_plot(robot, target_pos, 1000);
        pause(1.2);
        disp(a);
    
    end
%     robot.interpolate_jp(SERV_ID, [0,0,0], 1000);





catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown();





function move_and_plot(robot, targets, time)
    SERV_ID = 1848;          
    SERVER_ID_READ =1910;
    tab_data = zeros(10, 4);
%     tab_row = zeros(1,4);
    current_time = 0;
    i = 1;
    j = 1;
    ploting_time = (time+500)/1000;
    robot.interpolate_jp(SERV_ID, targets, time);
    
    tic
    while current_time < ploting_time
        if mod(j,20) == 0
            status_mat = robot.measured_js(1,0);
            pos_mat = status_mat(1,:);

            tab_data(i,1) = current_time;
            tab_data(i,2) = pos_mat(1);
            tab_data(i,3) = pos_mat(2);
            tab_data(i,4) = pos_mat(3);
            i = i + 1;
            current_time = toc;

        end
        j = j + 1;

    end
%     disp(tab_data)

%     writematrix(tab_data, 'pos_data.csv'); 
    
    hold on
    plot(tab_data(:,1), tab_data(:,2));
    plot(tab_data(:,1), tab_data(:,3));
    plot(tab_data(:,1), tab_data(:,4));
    hold off
    legend({"Motor 1", "Motor 2", "Motor 3"});
    xlabel("Time (s)");
    ylabel("Motor Position (deg)");
    title("Robot Arm Position Against Time");

   

%     tab_data_time = tab_data(:,1);
% 
%     histogram(tab_data_time);

end













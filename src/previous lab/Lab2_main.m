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
model2 = Model2();

    linkX = [0 0 0 0 0];
    linkY = [0 0 0 0 0];
    linkZ = [0 0 40 80 120];
    P0_1 = [0;0;0];
    P0_2 = [0;0;0];
    P0_3 = [0;0;0];
    P0_4 = [0;0;0];
    T0_1 = zeros(4,4);
    T0_2 = zeros(4,4);
    T0_3 = zeros(4,4);
    T0_4 = zeros(4,4);

    % For the active plotting part
    robot.interpolate_jp([0 15 0], 10000)

    % Plot joints
    pJ = plot3(linkX, linkY, linkZ, '.', 'Color', 'r', 'MarkerSize', 30);
    hold on
    pJ.XDataSource = 'linkX';
    pJ.YDataSource = 'linkY';
    pJ.ZDataSource = 'linkZ';

    % Plot links
    pL = plot3(linkX, linkY, linkZ, '-k', 'LineWidth', 2);
    pL.XDataSource = 'linkX';
    pL.YDataSource = 'linkY';
    pL.ZDataSource = 'linkZ';

    % Plot frames
    % Frame0
    plot3([0 30], [0 0], [0 0], '-r', 'LineWidth', 1);
    plot3([0 0], [0 30], [0 0], '-g', 'LineWidth', 1);
    plot3([0 0], [0 0], [0 30], '-b', 'LineWidth', 1);
    % All the XYZ value for each plot is set to be default value [0 0],
    % which will eventaully be refreshed by the set() in the for loop
    % Frame1
    pf1x = plot3([0 0], [0 0], [0 0], '-r', 'LineWidth', 1);
    pf1y = plot3([0 0], [0 0], [0 0], '-g', 'LineWidth', 1);
    pf1z = plot3([0 0], [0 0], [0 0], '-b', 'LineWidth', 1);
    % Frame2
    pf2x = plot3([0 0], [0 0], [0 0], '-r', 'LineWidth', 1);
    pf2y = plot3([0 0], [0 0], [0 0], '-g', 'LineWidth', 1);
    pf2z = plot3([0 0], [0 0], [0 0], '-b', 'LineWidth', 1);
    % Frame3
    pf3x = plot3([0 0], [0 0], [0 0], '-r', 'LineWidth', 1);
    pf3y = plot3([0 0], [0 0], [0 0], '-g', 'LineWidth', 1);
    pf3z = plot3([0 0], [0 0], [0 0], '-b', 'LineWidth', 1);
    % Frame4
    pf4x = plot3([0 0], [0 0], [0 0], '-r', 'LineWidth', 1);
    pf4y = plot3([0 0], [0 0], [0 0], '-g', 'LineWidth', 1);
    pf4z = plot3([0 0], [0 0], [0 0], '-b', 'LineWidth', 1);

    % Complete other informations for the plot
    title('3D Stick Model of the Robot')
    xlabel('X Axis(mm)')
    ylabel('Y Axis(mm)')
    zlabel('Z Axis(mm)')
    legend([pf1x pf1y pf1z], {'Frame X','Frame Y','Frame Z'})
    grid on
    axis([-200 200 -200 200 0 400]) % limit the size of the 3d frame
    hold off
    
    for i = 1:200 % limit the time that the plot runs
        currentMeasurement = robot.measured_js(1, 1)
        currentJointData = currentMeasurement(1, 1:3)
        model2.plot_arm(currentJointData(1), currentJointData(2), currentJointData(3)) % the plot_arm is only used to run all the calculations to refresh required data for plotting
        theta1 = currentJointData(1);
        theta2 = currentJointData(2);
        theta3 = currentJointData(3);
        % Reassign data after been refreshed by plot_arm 
        linkX = model2.linkX;
        linkY = model2.linkY;
        linkZ = model2.linkZ;
        P0_1 = model2.P0_1;
        P0_2 = model2.P0_2;
        P0_3 = model2.P0_3;
        P0_4 = model2.P0_4;
        T0_1 = model2.T0_1;
        T0_2 = model2.T0_2;
        T0_3 = model2.T0_3;
        T0_4 = model2.T0_4;

        % Update frame1 data
        set(pf1x, 'XData', [0 30*cosd(theta1)], 'YData', [0 30*sind(theta1)], 'ZData', [P0_1(3) P0_1(3)]);
        set(pf1y, 'XData', [0 -30*sind(theta1)], 'YData', [0 30*cosd(theta1)], 'ZData', [P0_1(3) P0_1(3)]);
        set(pf1z, 'XData', [0 0], 'YData', [0 0], 'ZData', [P0_1(3) P0_1(3)+30]);
        % Update frame2 data
        set(pf2x, 'XData', [P0_2(1) P0_2(1)+T0_2(1,1)*30], 'YData', [P0_2(2) P0_2(2)+T0_2(2,1)*30], 'ZData', [P0_2(3) P0_2(3)+T0_2(3,1)*30]);
        set(pf2y, 'XData', [P0_2(1) P0_2(1)+T0_2(1,2)*30], 'YData', [P0_2(2) P0_2(2)+T0_2(2,2)*30], 'ZData', [P0_2(3) P0_2(3)+T0_2(3,2)*30]);
        set(pf2z, 'XData', [P0_2(1) P0_2(1)+T0_2(1,3)*30], 'YData', [P0_2(2) P0_2(2)+T0_2(2,3)*30], 'ZData', [P0_2(3) P0_2(3)+T0_2(3,3)*30]);
        % Update Frame3 data
        set(pf3x, 'XData', [P0_3(1) P0_3(1)+T0_3(1,1)*30], 'YData', [P0_3(2) P0_3(2)+T0_3(2,1)*30], 'ZData', [P0_3(3) P0_3(3)+T0_3(3,1)*30]);
        set(pf3y, 'XData', [P0_3(1) P0_3(1)+T0_3(1,2)*30], 'YData', [P0_3(2) P0_3(2)+T0_3(2,2)*30], 'ZData', [P0_3(3) P0_3(3)+T0_3(3,2)*30]);
        set(pf3z, 'XData', [P0_3(1) P0_3(1)+T0_3(1,3)*30], 'YData', [P0_3(2) P0_3(2)+T0_3(2,3)*30], 'ZData', [P0_3(3) P0_3(3)+T0_3(3,3)*30]);
        % Update Frame4 data
        set(pf4x, 'XData', [P0_4(1) P0_4(1)+T0_4(1,1)*30], 'YData', [P0_4(2) P0_4(2)+T0_4(2,1)*30], 'ZData', [P0_4(3) P0_4(3)+T0_4(3,1)*30]);
        set(pf4y, 'XData', [P0_4(1) P0_4(1)+T0_4(1,2)*30], 'YData', [P0_4(2) P0_4(2)+T0_4(2,2)*30], 'ZData', [P0_4(3) P0_4(3)+T0_4(3,2)*30]);
        set(pf4z, 'XData', [P0_4(1) P0_4(1)+T0_4(1,3)*30], 'YData', [P0_4(2) P0_4(2)+T0_4(2,3)*30], 'ZData', [P0_4(3) P0_4(3)+T0_4(3,3)*30]);
        disp(P0_4(1))

        refreshdata
        drawnow
        pause(0.01)
    end
    
catch exception
    disp("ERROR!!!!!!!!!!!!!!!!!!!!!!!!");
    disp(exception);
end

% Clear up memory upon termination
robot.shutdown()

%toc
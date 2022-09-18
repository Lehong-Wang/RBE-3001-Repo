classdef Model < handle
    
    properties
        theta1;
        theta2;
        theta3;
    end
    
    methods
        function self = Model()
        end
        
        % input: 3*1 array of joint values and plot a stick model of the
        % arm showing all frames, joints, and links
        function plot_arm(self, q)
            theta1 = q(1);
            theta2 = q(2);
            theta3 = q(3);
            
            % getting T0_1, T0_2, T0_3, T0_4 with hardcode 
            T0_1 = [ 1     0     0     0;
                     0     1     0     0;
                     0     0     1    55;
                     0     0     0     1];

            T0_2 = [cos((pi*theta1)/180),  0, -sin((pi*theta1)/180),  0;
                    sin((pi*theta1)/180),  0,  cos((pi*theta1)/180),  0;
                                       0, -1,                     0, 95; 
                                       0,  0,                     0,  1];

            T0_3 = [cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180), -cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180), -sin((pi*theta1)/180), 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180);
                    sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180), -sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180),  cos((pi*theta1)/180), 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180);
                                        -sin((pi*(theta2 - 90))/180),                      -cos((pi*(theta2 - 90))/180),                     0,                 95 - 100*sin((pi*(theta2 - 90))/180);
                                                                   0,                                                 0,                     0,                                                    1];

            T0_4 = [ cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180), -sin((pi*theta1)/180), 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                     sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),  cos((pi*theta1)/180), 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                                                             - cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),                                             sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180),                     0,                                                           95 - 100*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - 100*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180) - 100*sin((pi*(theta2 - 90))/180);
                                                                                                                                                                               0,                                                                                                                                                             0,                     0,                                                                                                                                                                                                                          1];

            % assign position value to each joint for plotting
            joint1XYZ = [T0_1(1,4) T0_1(2,4) T0_1(3,4)];
            joint2XYZ = [T0_2(1,4) T0_2(2,4) T0_2(3,4)];
            joint3XYZ = [T0_3(1,4) T0_3(2,4) T0_3(3,4)];
            joint4XYZ = [T0_4(1,4) T0_4(2,4) T0_4(3,4)];

            % store joint cordinates into xyz matrices for inputting into
            % plot3() func
            linkX = [0, joint1XYZ(1), joint2XYZ(1), joint3XYZ(1), joint4XYZ(1)];
            linkY = [0, joint1XYZ(2), joint2XYZ(2), joint3XYZ(2), joint4XYZ(2)];
            linkZ = [0, joint1XYZ(3), joint2XYZ(3), joint3XYZ(3), joint4XYZ(3)];

            
            clf; % deletes all children of the current figure regardless of their handle visibility

            ax = axes;
            ax.ColorOrder = [0 0 1];
            
            % plot joints
            plot3(0, 0, 0, 'o', 'MarkerFaceColor', 'k', 'MarkerSize', 7)
            hold on
            plot3(joint1XYZ(1), joint1XYZ(2), joint1XYZ(3), 'o', 'MarkerFaceColor', 'k', 'MarkerSize', 7)
            hold on
            plot3(joint2XYZ(1), joint2XYZ(2), joint2XYZ(3), 'o', 'MarkerFaceColor', 'k', 'MarkerSize', 7)
            hold on
            plot3(joint3XYZ(1), joint3XYZ(2), joint3XYZ(3), 'o', 'MarkerFaceColor', 'k', 'MarkerSize', 7)
            hold on
            plot3(joint4XYZ(1), joint4XYZ(2), joint4XYZ(3), 'o', 'MarkerFaceColor', 'k', 'MarkerSize', 7)
            hold on
            

            % Plot links
            plot3(linkX, linkY, linkZ, '-k', 'LineWidth', 3);            
            
            title('3D Stick Model of the Robot')
            xlabel('X Axis(mm)')
            ylabel('Y Axis(mm)')
            zlabel('Z Axis(mm)')
            grid on
            axis([0 400 -200 200 0 400]) % limit the size of the 3d frame
            hold off
            drawnow
        end
        
    end
end
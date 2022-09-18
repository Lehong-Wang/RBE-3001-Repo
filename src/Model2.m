classdef Model2 < handle % This model2 is the model2 class for live 3d plot
    
    properties
        joint1XYZ = [0 0 0];
        joint2XYZ = [0 0 0];
        joint3XYZ = [0 0 0];
        joint4XYZ = [0 0 0];
        linkX = [0 0 0];
        linkY = [0 0 0];
        linkZ = [0 0 0];
        P0_1;
        P0_2;
        P0_3;
        P0_4;
        T0_1;
        T0_2;
        T0_3;
        T0_4;
    end
    
    methods
        function self = Model2()
        end
        
        % input: no input, the func only do the calculation with the givin
        % theta123
        function plot_arm(self, theta1, theta2, theta3)
            
            % getting T0_1, T0_2, T0_3, T0_4 with hardcode 
            self.T0_1 = [ 1     0     0     0;
                     0     1     0     0;
                     0     0     1    55;
                     0     0     0     1];

            self.T0_2 = [cos((pi*theta1)/180),  0, -sin((pi*theta1)/180),  0;
                    sin((pi*theta1)/180),  0,  cos((pi*theta1)/180),  0;
                                       0, -1,                     0, 95;      
                                       0,  0,                     0,  1];

            self.T0_3 = [cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180), -cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180), -sin((pi*theta1)/180), 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180);
                    sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180), -sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180),  cos((pi*theta1)/180), 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180);
                                        -sin((pi*(theta2 - 90))/180),                      -cos((pi*(theta2 - 90))/180),                     0,                 95 - 100*sin((pi*(theta2 - 90))/180);
                                                                   0,                                                 0,                     0,                                                    1];

            self.T0_4 = [ cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180), -sin((pi*theta1)/180), 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                     sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),  cos((pi*theta1)/180), 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                                                             - cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),                                             sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180),                     0,                                                           95 - 100*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - 100*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180) - 100*sin((pi*(theta2 - 90))/180);
                                                                                                                                                                               0,                                                                                                                                                             0,                     0,                                                                                                                                                                                                                          1];

            % assign position value to each joint for plotting
            self.joint1XYZ = [self.T0_1(1,4) self.T0_1(2,4) self.T0_1(3,4)];
            self.joint2XYZ = [self.T0_2(1,4) self.T0_2(2,4) self.T0_2(3,4)];
            self.joint3XYZ = [self.T0_3(1,4) self.T0_3(2,4) self.T0_3(3,4)];
            self.joint4XYZ = [self.T0_4(1,4) self.T0_4(2,4) self.T0_4(3,4)];

            % getting position vectors
            self.P0_1 = self.T0_1(1:3, 4);
            self.P0_2 = self.T0_2(1:3, 4);
            self.P0_3 = self.T0_3(1:3, 4);
            self.P0_4 = self.T0_4(1:3, 4);

            % store joint cordinates into xyz matrices for inputting into
            % plot3() func
            self.linkX = [0, self.joint1XYZ(1), self.joint2XYZ(1), self.joint3XYZ(1), self.joint4XYZ(1)];
            self.linkY = [0, self.joint1XYZ(2), self.joint2XYZ(2), self.joint3XYZ(2), self.joint4XYZ(2)];
            self.linkZ = [0, self.joint1XYZ(3), self.joint2XYZ(3), self.joint3XYZ(3), self.joint4XYZ(3)];

            end
        
    end
end
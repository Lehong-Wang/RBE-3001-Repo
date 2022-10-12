

classdef Matrix

    properties
    end

    methods (Static)

%         function [dh2mat, dh2fk, fk3001] = dh_matrix
%             dh2mat = @dh2mat;
%             dh2fk = @dh2fk;
%             fk3001 = @fk3001;
% 
% 
%         end


        function self = Matrix()
        end






        %%%%%%%%%%%%%%%%%%%%%%%%%%% Forward Kinamatics %%%%%%%%%%%%%%%%%%%%%%%%%%%%



        % takes in a 1x4 array corresponding to a row of the DH parameter table for 
        % given link. Then generates the associated intermediate transformation and returns
        % a corresponding symbolic 4x4 homogeneous transformation matrix. 
        function T = dh2mat(dh_parameter)
            dh_cell = num2cell(dh_parameter);
            [theta, d, a, alpha] = dh_cell{:};
            st = sin(theta);
            ct = cos(theta);
            sa = sin(alpha);
            ca = cos(alpha);

            T = [   ct  -st*ca  st*sa   a*ct;
                    st  ct*ca   -ct*sa  a*st;
                    0   sa      ca      d;
                    0   0       0       1;];
        end



        % takes in an nx4 array corresponding to the n rows of  the  full  
        % DH  parameter  table.  Then  generates  a  corresponding  symbolic  4x4 
        % homogeneous  transformation  matrix  for  the  composite  transformation. 
        function T = dh2fk(dh_array)
            % init a 2D matrix, which will later be 3D
            t_matrix_array = zeros(4);
            T = eye(4);

            for i = (1:height(dh_array))
                dh_row = dh_array(i,:);
%                 a = Matrix.dh2mat(dh_row);
                % make it 3D and store trans matrix
                t_matrix_array(:,:,i) = Matrix.dh2mat(dh_row);
            end
            %     disp(t_matrix_array);

            matrix_dim = size(t_matrix_array);
            if length(matrix_dim) == 2
                depth = 1;
            else
                depth = matrix_dim(3);
            end

            for i = (1:depth)
                T = T * t_matrix_array(:,:,i);
            end

        end

        % takes in 3 joint in degrees
        % return a 4*4 homogeneous transformation matrix  
        % representing the position and orientation of the tip frame with respect to the base frame 
        function T = fk3001(q1, q2, q3)

%             pos = arrayfun(@(x) deg2rad(x), joint_config);
            pos = [deg2rad(q1) deg2rad(q2) deg2rad(q3)];
            dh_tab = zeros(1,4);
            % base to joint 1
            dh_tab(1,:) = [0 55 0 0];

            if isempty(pos)
                T = Matrix.dh2fk(dh_tab);
            else
                for i = (1:length(pos))
                    % enter the robot structure
                    if i == 1
                        joint_dh = [pos(1) 40 0 -pi/2];
                    elseif i == 2
                        joint_dh = [pos(2)-pi/2 0 100 0];
                    elseif i == 3
                        joint_dh = [pos(3)+pi/2 0 100 0];
                    else
                        joint_dh = [pos(i) 0 100 0];
                    end
                    % first row is T_0_1
                    dh_tab(i+1,:) = joint_dh;
                end
    %             disp(dh_tab);
                T = Matrix.dh2fk(dh_tab);
            end
        end




        % take a joint configuration (3*1 matrix of degrees)
        % return a 4*4 Transformation Matrix

        function T = hard_code(theta1, theta2, theta3)
%             q1 = deg2rad(theta1)
%             q2 = deg2rad(theta2)
%             q3 = deg2rad(theta3)
%             ct1 = cos(q1);
%             st1 = sin(q1);
%             ct2 = cos(q2);
%             st2 = sin(q2);
%             ct3 = cos(q3);
%             st3 = sin(q3);
%             returnFM = [ ct1*st2*st3 - ct1*-ct2*ct3,    - ct1*st2*ct3 - ct1*st3*-ct2,       -st1,   100*ct1*st2 + 100*ct1*st2*st3 - 100*ct1*-ct2*ct3;
%                          st1*st2*st3 - st1*-ct2*ct3,    - st1*st2*ct3 - st1*st3*-ct2,       ct1,    100*st1*st2 + 100*st1*st2*st3 - 100*st1*-ct2*ct3;
%                          - st2*ct3 - st3*-ct2,          -ct2*ct3 - st2*st3,                 0,      95 - 100*st2*ct3 - 100*st3*-ct2 - 100*-ct2;
%                          0,                             0,                                  0,      1];
%             
%             

            T = [ cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180), -sin((pi*theta1)/180), 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                  sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),  cos((pi*theta1)/180), 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                                                          - cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),                                             sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180),                     0,                                                           95 - 100*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - 100*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180) - 100*sin((pi*(theta2 - 90))/180);
                                                                                                                                                                            0,                                                                                                                                                             0,                     0,                                                                                                                                                                                                                          1];

            
%             T = returnFM;
        end





        %%%%%%%%%%%%%%%%%%%%%%%%%%% Inverse Kinamatics %%%%%%%%%%%%%%%%%%%%%%%%%%%%






        
        
        % given x,y,z coordinate, check if it is in the workspace of arm
        % this is a percise calculation, 
        %   and is calculated by folding the first joint, then the second,
        %   expand the first, then the second.
        % Calculations invalves vertical sclicing (derivative)
        function in_work_space = check_in_work_space(x, y, z)
        
            if z<0
                in_work_space = false;
                error("ERROR: Z axis less than 0");
            end
            % outer sphere formed by arm length
            % all things below are already bounded by this
            if (x^2 + y^2 + (z-95)^2) > 200^2
                in_work_space = false;
                disp([x,y,z]);
                error('ERROR: NOT in work space! \nArm not long enough');
            end
            % bottom part, limited by base joint 
            % x > 0 and dig out 50 mm for base joint
            if (z >= 0 && z <= 95)
                if x<0
                    in_work_space = false;
                    disp([x,y,z]);
                    error("ERROR: NOT in work space! \nJoint 1 limitation [90, -90] (bottom)");
                end
                if x^2+y^2 < 50^2
                    in_work_space = false;
                    disp([x,y,z]);
                    error("ERROR: NOT in work space! \nMay hit base joint");
                end
                in_work_space = true;
                return
            end
            % middle part, limited by joint 2 -45 deg range
            % x > 0 and dig out link 3
            if (z > 95 && z <= 95+100*2^0.5)
                if x<0
                    in_work_space = false;
                    disp([x,y,z]);
                    error("ERROR: NOT in work space! \nJoint 1 limitation [90, -90] (middle)");
                end
                if sqrt(x^2+y^2) < sqrt(100^2-(z-195)^2)-50*2^0.5
                    in_work_space = false;
                    disp([x,y,z]);
                    error("ERROR: NOT in work space! \nJoint 2 limitation [90, -45] (middle)");
                end
                in_work_space = true;
                return
            end
            % top part, limited by joint 2 -45 deg range
            % x > 0 all good, x < 0 dig out top of link 3
            if (z > 95+100*2^0.5 && z <= 195+50*2^0.5)
                disp(sqrt(x^2+y^2))
                if x<0 && sqrt(x^2+y^2) > 50*2^0.5-sqrt(100^2-(z-195)^2) && sqrt(x^2+y^2) < 50*2^0.5+sqrt(100^2-(z-195)^2)
                   in_work_space = false;
                   disp([x,y,z]);
                   error("ERROR: NOT in work space! \nJoint 2 limitation [90, -45] (top)");
                end 
                in_work_space = true;
                return
            end
            % top top part
            % all good
            if (z > 195+50*2^0.5)
                in_work_space = true;
                return
            end
        
        end
        
        


        % take in x,y,z position
        % output 1*3 matrix of joint angles in degrees
        function T = ik3001(x,y,z)
            try
                Matrix.check_in_work_space(x,y,z);
            catch exception
                getReport(exception);
            end

            q1 = atan2(y, x);   % should be atan2(y, +-x), but x>0 (for simplicity)
            l = sqrt(x^2 + y^2 + (z-95)^2);     % length from joint2 to tip
            cos_theta = (2 * 100^2 - (x^2 + y^2 + (z-95)^2)) / (2 * 100^2);  % angle between joint3-joint2 and joint3-tip
            s3 = -cos_theta;        % q3 = theta + pi/2
            c3 = sqrt(1 - s3^2);     % should be +-sqrt, but -pi/2 < q3 < pi/2, cos(q3) > 0
            q3 = atan2(s3, c3);     
            
            alpha = atan2(z-95, sqrt(x^2+y^2));  % angle between line joint1-tip and horizontal line 
            beta = (pi/2 - q3) / 2;      % another angle of the triangle, angle between line joint2-tip and line joint2-joint3
            q2 = pi/2 - alpha - beta;    % q2 + alpha + beta = pi/2

            T = [rad2deg(q1) rad2deg(q2) -rad2deg(q3)];  % direction of q3 is inverse (up is minus)
        end
















    end
end


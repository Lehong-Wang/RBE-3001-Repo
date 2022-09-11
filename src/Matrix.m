

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

        % dh2mat([1,2,3,4]);

        function self = Matrix()
        end


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




        function T = dh2fk(dh_array)
            % init a 2D matrix, which will later be 3D
            t_matrix_array = zeros(4);
            T = eye(4);

            for i = (1:height(dh_array))
                dh_row = dh_array(i,:);
                a = Matrix.dh2mat(dh_row);
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


        function T = fk3001(joint_config)
            pos = joint_config;
            dh_tab = zeros(1,4);
            % base to joint 1
            dh_tab(1,:) = [0 55 0 0];

            if length(pos) == 0
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

    end
end


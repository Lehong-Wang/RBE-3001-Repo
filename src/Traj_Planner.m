

classdef Traj_Planner

    properties
    end

    methods (Static)



        % solve parameters with time, pos, velocity
        function A = cubic_traj(t0, tf, q0, qf, v0, vf)
            M = [1 t0 t0^2 t0^3;
                 0 1 2*t0^2 3*t0^3;
                 1 tf tf^2 tf^3;
                 0 1 2*tf^2 3*tf^3;];
            B = [q0; v0; qf; vf;];

            A = inv(M) * B;

        end




        % target is 3*1 joint degree
        function param_mat = solve_parameter_velocity(start_time, end_time, start_tar, end_tar)
        
            start_v = 0;
            end_v = 0;  
            param_mat = zeros(0);
            % calculate parameters with cubic_traj
            for j = (1:3)
                parameters = Traj_Planner.cubic_traj( ...
                    start_time, end_time, ...
                    start_tar(j), end_tar(j), ...
                    start_v, end_v ...
                    );
                param_mat(j,:) = parameters;
            end

        end

        % % target is 3*1 position
        % function param_mat = solve_position_velocity(start_time, end_time, start_tar, end_tar)
        %     % param_mat = zeros(0);
        %     % use inverse kinamatic to solve for joint degrees
        %     start_deg = Matrix.ik3001(start_tar(1), start_tar(2), start_tar(3));
        %     end_deg = Matrix.ik3001(end_tar(1), end_tar(2), end_tar(3));
        %     param_mat = Traj_Planner.solve_degree_velocity(start_time, end_time, start_deg, end_deg);
        % end



        % take the solved parameters and time
        % compute the angles each joint should be at this time
        % return 1*3 matrix of joint angle in degrees
        % can stick into servo_jp
        function target = get_angle_target_angle(param_mat, t)

            t_vector = [1; t; t^2; t^3];
            target = param_mat * t_vector;
        end

        % when solving in task space (with position)
        % convert into joint angles with inverse kinamatics
        function target = get_angle_target_pos(param_mat, t)

            t_vector = [1; t; t^2; t^3];
            target = param_mat * t_vector;
            target = Matrix.ik3001(target(1), target(2), target(3));
        end

        % calculate full x,y,z target angle from y parameter and func    
        function target = get_angle_target_func(param_y, func, t)

            t_vector = [1; t; t^2; t^3];
            target_y = param_y * t_vector;
            target = func(target_y);
            target = Matrix.ik3001(target(1), target(2), target(3));
        end


        % given the start and end of y, and time
        % calculate parameters for y ONLY
        function param_mat = solve_parameter_function(start_time, end_time, start_y, end_y)
            start_v = 0;
            end_v = 0;  
            param_mat = zeros(0);
            % calculate parameters with cubic_traj
            parameters = Traj_Planner.cubic_traj( ...
                start_time, end_time, ...
                start_y, end_y, ...
                start_v, end_v ...
                );

            param_mat(1,:) = parameters;
            
        end


        % these are functions that define a trajctory
        % x,z are expressed in y
        % the functions return [x y z]
        % NOTE: y should have corrspond to one unique x and unique z
        %       trajectory with constant y will not work
        function pos = straight_line(y)
            x = y;
            z = y;
            pos = [x y z];
        end

        function pos = verticle_half_circle(y)
            x = 60;
            z = sqrt(150^2 - y^2) + 20;
            pos = [x y z];
        end

        function pos = fun_stuff(y)
            x = 60;
            if -150 <= y && y < -100
                z = 2 * sqrt(25^2 - (y+125)^2) + 20;
            elseif -100 <= y && y < -50
                z = 2 * sqrt(25^2 - (y+75)^2) + 20;
            elseif -50 <= y && y < 50
                z = -9/125 * y^2 + 200;
            elseif 50 <= y && y < 100
                z = 2 * sqrt(25^2 - (y-75)^2) + 20;
            elseif 100 <= y && y < 150
                z = 2 * sqrt(25^2 - (y-125)^2) + 20;
            else
                z = 20;
            end
            pos = [x y z];
        end







    end

end


























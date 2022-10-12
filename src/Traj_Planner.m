

classdef Traj_Planner

    properties (Constant)
        % 20 degree rotate around tip z axis
        pen_T = [cos(pi/5) -sin(pi/5) 0 0;
                 sin(pi/5) cos(pi/5)  0 0;
                 0         0          1 0;
                 0         0          0 1;];
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



        function pos = draw_heart_upper(y)
            global end_effector_T
            disp("End Effector T:");
            disp(end_effector_T);
            pen_length = 35;

            if -45 <= y && y < 0
                x = sqrt(23^2 - (y+22)^2) + 130;
            elseif 0 <= y && y < 45
                x = sqrt(23^2 - (y-22)^2) + 130;
            else
                x = 130;
            end
            
            if size(end_effector_T,1) == 4
                e_e_T = end_effector_T * Traj_Planner.pen_T
                z_factor = e_e_T(3,1)
                z = - pen_length * z_factor
            else
                z = 25
            end
            pos = [x y z];
        end

        function pos = draw_heart_lower(y)
            global end_effector_T;
            pen_length = 25;

%             z = 30 - 8 + 8/45 * abs(y);
            if -45 <= y && y < 0
                x = -1*y + 85;
            elseif 0 <= y && y < 45
                x = 1*y + 85;
            else
                x = 130;
            end

            if size(end_effector_T,1) == 4
                e_e_T = end_effector_T
                e_e_T = end_effector_T * Traj_Planner.pen_T
                z_factor = e_e_T(3,1)
                z = - pen_length * z_factor
            else
                z = 25
            end
            pos = [x y z];
        end






        function plot_plan(fun, start_y, end_y)
            x_vector = zeros(0);
            y_vector = zeros(0);
            z_vector = zeros(0);
            for i = 1:1000
                y = (end_y - start_y) * i / 1000 + start_y;
                ret_pos = fun(y);
                x_vector(i) = ret_pos(1);
                y_vector(i) = ret_pos(2);
                z_vector(i) = ret_pos(3);
            end
            plot3(x_vector, y_vector, z_vector, "b",'LineWidth',2);

        end


        function update_plot(x,y,z)

        end




    end

end


























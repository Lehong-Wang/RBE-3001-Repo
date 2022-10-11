classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962;
        SERV_ID = 1848; 
%         matrix = Matrix();
    end
%     matrix = Matrix();

    methods






        % ######################################## Lab 5 Functions ########################################







        
        function pick_up_balls(self, ball_coords, drop_coord)
            disp(ball_coords);
            self.run_trajectory([100;0;195], 1);
            for i = 1 : size(ball_coords, 1)
                self.openGripper();
                target_coord = [ball_coords(i,1) ball_coords(i,1); 
                                ball_coords(i,2) ball_coords(i,2); 
                                50 10];
                disp(target_coord);
                self.run_trajectory(target_coord, 2);
                pause(1);
                self.closeGripper();
                pause(0.5);
                self.run_trajectory([100;0;195], 1);
                drop_target_coord = [drop_coord(1) drop_coord(1); 
                                drop_coord(2) drop_coord(2); 
                                50 10];
                self.run_trajectory(drop_target_coord, 1);
                pause(1);
                self.openGripper();
                self.run_trajectory([drop_coord(1); drop_coord(2); 100], 1);


            end
            self.run_trajectory([100;0;195], 1);
        end








% ######################################## Lab 3 Functions ########################################

        % take in 3*n matrix of targets to reach (currently 3 joint in degrees)
        % take in time in seconds
        function record_mat = run_trajectory(self, target_matrix, time)
            if size(target_matrix, 1) ~= 3
                error("Target Matrix should be 3*n");
            end
            record_mat = zeros(0);

            current_trans = self.measured_cp();
            current_pos = current_trans(1:3, 4);
            target_matrix = [current_pos target_matrix];
%             target_matrix = target_matrix * 1.02;

            tic
            % parse out target pairs
            for i = (1:size(target_matrix, 2)-1)

                start_tar = target_matrix(:,i);
                end_tar = target_matrix(:,i+1);
                start_time = toc;
                end_time = start_time + time;

                param_mat = Traj_Planner.solve_parameter_velocity(start_time, end_time, start_tar, end_tar);

                % for each start-end
                while toc < end_time
                    t = toc;
                    target = Traj_Planner.get_angle_target_pos(param_mat, t);
                    self.servo_jp(target);
                    % save graph data
                    recording = self.save_pos_data(t);
                    % recording = self.save_pos_plan(target, t);
                    record_mat = [record_mat; recording];
 
                end
                

            end
%             disp(record_mat);
        end


        % get the position data and format into [time x y z]
        function recording = save_pos_data(self, t)
            pos = self.measured_cp();
            recording = [t pos(1,4) pos(2,4) pos(3,4)];
        end

        % get the position plan and format into [time x y z]
        function recording = save_pos_plan(self, target, t)

            pos = Matrix.hard_code(target(1), target(2), target(3));
            recording = [t pos(1,4) pos(2,4) pos(3,4)];
        end
        

        % take in a function handle that calculates x,z with y
        %   some function handles are defined in Traj_Planner
        %   they all take y and return [x y z] 
        % NOTE: y should have corrspond to one unique x and unique z
        %       trajectory with constant y will not work
        % also take in start and end of y pos, and total time
        function record_mat = run_function_trajectory(self, func, start_y, end_y, time, do_real_time_plot)
            global end_effector_T
            end_effector_T = zeros(0)
            refresh_count = 0;


            if do_real_time_plot
                hold on
                Traj_Planner.plot_plan(func, start_y, end_y);

                X = nan(1);
                Y = nan(1);
                Z = nan(1);
                assignin('base','base_plot_x',X);
                assignin('base','base_plot_y',Y);
                assignin('base','base_plot_z',Z);

                plot_line = plot3(X,Y,Z, "r",'LineWidth',3);
                axis([-100 200 -200 200 0 300]);
                view(-30,45);



                plot_line.XDataSource = 'base_plot_x';
                plot_line.YDataSource = 'base_plot_y';
                plot_line.ZDataSource = 'base_plot_z';

                pause;
                
            end

            tic

            start_time = toc;
            end_time = start_time + time;
            record_mat = zeros(0);
            record_plan = zeros(0);
            record_pos = zeros(0);

            % solve parameters for y
            param_y = Traj_Planner.solve_parameter_function(start_time, end_time, start_y, end_y);
            while toc < end_time
                t = toc;     
                refresh_count = refresh_count + 1;
                % calculate full x,y,z target angle           
                target = Traj_Planner.get_angle_target_func(param_y, func, t);
                self.servo_jp(target);
                % save graph data
                recording = self.save_pos_plan(target, t);
                record_plan = [record_plan; recording];
                recording = self.save_pos_data(t);
                record_pos = [record_pos; recording];
                % save end effector transformation matrix to workspace
                e_e_T = Matrix.hard_code(target(1), target(2), target(3));
                end_effector_T = e_e_T;
%                 assign('base', 'end_effector_T', e_e_T);

                if do_real_time_plot && refresh_count > 20
                    refresh_count = 0;
                    assignin("base", 'base_plot_x', record_pos(100:end,2));
                    assignin("base", 'base_plot_y', record_pos(100:end,3));
                    assignin("base", 'base_plot_z', record_pos(100:end,4));
                    refreshdata
                    drawnow
                end

            end
            record_mat(:,:,1) = record_plan;
            record_mat(:,:,2) = record_pos;

        end

        % plot the recorded data
        function plot_record(self, record_mat)
            hold on
            for i = (1:size(record_mat,3))
                % time_vector = record_mat(:,1,i);
                x_vector = record_mat(:,2,i);
                y_vector = record_mat(:,3,i);
                z_vector = record_mat(:,4,i);
                plot3(x_vector, y_vector, z_vector);
                axis([-100 200 -200 200 0 300]);

            end
        end




% ######################################## Lab 2 Functions ########################################

        % return transformation matrix of base to tip
        function T = measured_cp(self)
            status_tab = self.measured_js(1,0);
            pos = status_tab(1,:);
            % turn degree to radians
            % pos_rad = arrayfun(@(x) deg2rad(x), pos);
            T = Matrix.fk3001(pos(1), pos(2), pos(3));
            % disp(pos);
        end

        % takes data from setpoint_js() and returns a 4x4 homogeneous transformation 
        % matrix based upon the current joint set point positions in degrees.

           
        %  takes  data  from  goal_js()  and  returns  a  4x4  homogeneous  transformation 
        % matrix based upon the commanded end of motion joint set point positions in degrees.  

        function T = setpoint_cp(self)
            set_tab = self.setpoint_js();
            set_rad = arrayfun(@(x) deg2rad(x), set_tab);
            T = Matrix.fk3001(set_rad);
%             disp(set_rad);
        end


        function T = goal_cp(self)
            goal_tab = self.goal_js();
            goal_rad = arrayfun(@(x) deg2rad(x), goal_tab);
            T = Matrix.fk3001(goal_rad);
%             disp(goal_rad);
        end

        % return all transformation matrix of the joints
        % helper for ploting
        % should be 4*4*n matrix (n=4)
        function T = get_all_trans_mat(self)
            T_0 = zeros(4);
            status_tab = self.measured_js(1,0);
            joint_angle = status_tab(1,:);
            % turn degree to radians
            pos_rad = arrayfun(@(x) deg2rad(x), joint_angle);
            T_1 = Matrix.fk3001(zeros(1,3));
            T_2 = Matrix.fk3001(pos_rad(1));
            T_3 = Matrix.fk3001(pos_rad(1:2));
            T_4 = Matrix.fk3001(pos_rad(1:3));

            T(:,:,1) = T_0;
            T(:,:,2) = T_1;
            T(:,:,3) = T_2;
            T(:,:,4) = T_3;
            T(:,:,5) = T_4;


        end









% ######################################## Lab 1 Functions ########################################


        % Returns  a  1x3  array  that  contains  the  end-of-motion  joint  
        % setpoint  positions  in degrees.
	    function packet = goal_js(self)
            packet = zeros(1,3, 'single');      % Initalize the matrix to zeros
            SERVER_ID_READ = 1848; % Set Motor Setpoints with Time (Interpolation) packet ID
            returnPacket = self.read(SERVER_ID_READ); % Read from Set Motor Setpoints with Time (Interpolation) packet
            % Set corresponding packet data to correct matrix row
            packet(1,1) = returnPacket(3);
            packet(1,2) = returnPacket(4);
            packet(1,3) = returnPacket(5);
            disp(packet);
        end

        % Which takes  two  boolean  values, named  GETPOS  and GETVEL. Only  
        % return the  results  for the requested data, and set the rest to
        % zero.
        % Which returns a 2x3 array that contains current joint positions in degrees (1st row) and/or 
        % current joint velocities (2nd row).
	    function out = measured_js(self, GETPOS, GETVEL)
            out = zeros(2,3,'single');  % Initalize matrix to zeros
            if GETPOS   % If we want the position
                pos = self.read(1910); % Read from Positions and Setpoint packet
                % Set corresponding packet data to the matrix
                out(1,1) = pos(3);
                out(1,2) = pos(5);
                out(1,3) = pos(7);
            end 
            
            if GETVEL   % If we want the velocity
                pos = self.read(1822); % Read from Velocity data packet
                % Set corresponding packet data to the matrix
                out(2,1) = pos(3);
                out(2,2) = pos(6);
                out(2,3) = pos(9);
            end
%             disp(out);    % Display the data
        end

        % takes a 1x3 array of joint values and an interpolation time in ms to get there 
        function interpolate_jp(self, targets, time)
            packet = zeros(15, 1, 'single');        % Initalize matrix to zeros
            packet(1) = time; % time in ms
            packet(2) = 0; % linear interpolation
            packet(3) = targets(1); % First link
            packet(4) = targets(2); % Second link
            packet(5) = targets(3); % Third link
            % Write the packet
            self.write(self.SERV_ID, packet);
            return
        end
        
        % takes a 1x3 array of joint values in degrees to be sent directly to the actuators and 
        % bypasses interpolation 
 	    function servo_jp(self, targets)
            packet = zeros(15,1,'single');      % Initalize matrix to zeros
            % Set corresponding packet data to correct matrix row
            packet(1) = 0;
            packet(2) = 0;
            packet(3) = targets(1); % First link
            packet(4) = targets(2); % Second link
            packet(5) = targets(3); % Third link
            % Write the packet
            self.write(self.SERV_ID, packet);
            return
        end
        
        % Returns  a  1x3  array  that  contains  current  joint  set  point  positions  in  degrees.  
        function packet = setpoint_js(self)
            packet = zeros(1, 3, 'single');     % Initalize matrix to zeros
            SERVER_ID_READ = 1848; % Setpoint packet ID
            returnPacket = self.read(SERVER_ID_READ); % Read from Postions and Setpoint packet
            packet(1) = returnPacket(3);
            packet(2) = returnPacket(4);
            packet(3) = returnPacket(5);
%             disp("Packet HERE");
%             disp(packet); % Display the data
%             disp("END");
        end





% ######################################## Default Functions ########################################



        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
            self.myHIDSimplePacketComs=dev;
            self.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(self, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(self, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(self, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(self.GRIPPER_ID);
                self.myHIDSimplePacketComs.writeBytes(intid, ds, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(self)
            self.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(self)
            self.writeGripper(0);
        end
        
    end
end

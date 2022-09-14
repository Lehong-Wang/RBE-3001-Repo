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
    
% ######################################## Lab 3 Functions ########################################
        % takes a 3x1 task space position vector (i.e. x, y, z components of pBaseTip that 
        % is the robot end-effector’s position w.r.t the base frame) as the input and returns a set
        % of corresponding joint angles (i.e. q1, q2, q3) that would make the robot’s end-effector 
        % move to that target position. 
        function T = ik3001()
        end
        
        % Takes two arguments, trajectory coefficients and the total maount
        % of time the trajector should take
        function T = run_trajectory(coeff, duration)
        end


% ######################################## Lab 2 Functions ########################################

        % return transformation matrix of base to tip
        function T = measured_cp(self)
            status_tab = self.measured_js(1,0);
            pos = status_tab(1,:);
            % turn degree to radians
            pos_rad = arrayfun(@(x) deg2rad(x), pos);
            T = Matrix.fk3001(pos_rad);
            disp(pos);
        end

        % takes data from setpoint_js() and returns a 4x4 homogeneous transformation 
        % matrix based upon the current joint set point positions in degrees.
        function T = setpoint_cp(self)
            set_tab = self.setpoint_js();
            set_rad = arrayfun(@(x) deg2rad(x), set_tab);
            T = Matrix.fk3001(set_rad);
%             disp(set_rad);
        end

        %  takes  data  from  goal_js()  and  returns  a  4x4  homogeneous  transformation 
        % matrix based upon the commanded end of motion joint set point positions in degrees.  
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
            T_1 = Matrix.fk3001(zeros(0));
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
            returnPacket = self.read(SERVER_ID_READ) % Read from Postions and Setpoint packet
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

classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
        curr_goal = zeros(1,3)
    end
    
    methods
        
        function servo_jp(self, SERV_ID, targets)
            packet = zeros(15,1,'single');
            packet(1) = 0;
            packet(2) = 0;
            packet(3) = targets(1); % First link
            packet(4) = targets(2); % Second link
            packet(5) = targets(3); % Third link
            %self.curr_goal = [packet(3),packet(4),packet(5)];
            % Write the packet
            self.write(SERV_ID, packet);
            return
        end

        function interpolate_jp(self, SERV_ID, targets, time)
            packet = zeros(15, 1, 'single');
            packet(1) = time; % time in ms
            packet(2) = 0; % linear interpolation
            packet(3) = targets(1); % First link
            packet(4) = targets(2); % Second link
            packet(5) = targets(3); % Third link
            %self.curr_goal = [packet(3),packet(4),packet(5)];
            % Write the packet
            self.write(SERV_ID, packet);
            return
        end

        function out = measured_js(self, GETPOS, GETVEL)
            out = zeros(2,3,'single');
            if GETPOS
                pos = self.read(1910);
                out(1,1) = pos(3);
                out(1,2) = pos(5);
                out(1,3) = pos(7);
            end 
            
            if GETVEL
                pos = self.read(1822);
%                  out(1,2) = pos(3);
%                 out(2,2) = pos(6);
%                out(3,2) = pos(9);
                out(2,1) = pos(3);
                out(2,2) = pos(6);
                out(2,3) = pos(9);
            end
%             disp(out);
        end

        function packet = setpoint_js(self)
            packet = zeros(1, 3, 'single');
                SERVER_ID_READ =1910;
                returnPacket = self.read(SERVER_ID_READ);
                packet(1,1) = returnPacket(3);
                packet(1,2) = returnPacket(4);
                packet(1,3) = returnPacket(5);
            disp(packet);
        end

        function packet = goal_js(self)
            packet = zeros(1,3, 'single');
            SERVER_ID_READ = 1848;
            returnPacket = self.read(SERVER_ID_READ);
            packet(1,1) = returnPacket(3);
            packet(1,2) = returnPacket(4);
            packet(1,3) = returnPacket(5);
          disp(packet);
        end

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

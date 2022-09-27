classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        jointSetpointGoal;
        SERV_ID = 1848;
        SERVER_ID_READ =1910;
        VEL_ID = 1822;
        GRIPPER_ID = 1962
        timeStamps = [0];
        actualJointAngle = [];
        jointAngleByCal = [];
        posArray = [];
    end
    
    methods
        
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

      
        % input array that contains angle value for each joint
        function servo_jp(packet, positionArray)
            packet.write(packet.SERV_ID, [0; 0; positionArray.'; zeros(15, 1)] );
            packet.jointSetpointGoal = positionArray;
        end
       
        function interpolate_jp(packet, positionArray, interpolateTime)
            packet.write(packet.SERV_ID, [interpolateTime; 0; positionArray.'; zeros(15, 1)] );
            packet.jointSetpointGoal = positionArray;
        end
% measured_js funtion
% takes two boolean GETPOS and GETVEL and 
% return a 2x3 array contains current joint positio and velocities
        function returnArray =  measured_js(packet, GETPOS, GETVEL)
            position = zeros(1,3);
            velocity = zeros(1,3);
            posData = packet.read(packet.SERVER_ID_READ);
            velData = packet.read(packet.VEL_ID);
            
            if GETPOS == 1
                position = [posData(3) posData(5) posData(7)];
            end
            
            if GETVEL == 1
                velocity = [velData(3) velData(6) velData(9)];
            end

            returnArray = [position; velocity];
        end
        
        
% return a 1x3 array contains current joint set point position in degrees
% before using the function, it needs to pause for a little while to waite
% the new setPoint data been written into the packet, or it will return
% invalid value 
        function returnSetpoint = setpoint_js(packet)
            setpointData = packet.read(packet.SERVER_ID_READ);
            returnSetpoint = [setpointData(2) setpointData(4) setpointData(6)];
        end

        function returnGoal = goal_js(packet)
             returnGoal = packet.jointSetpointGoal;
        end

        %--------------lab2-------------------------
        % input: a 1*4 array corresponding to a row of the DH parameter
        % table 
        % return: a symbolic 4*4 homogeneous transformation
        % matrix corresponding to that specific row 
        function returnTM = dh2mat(packet, inputArray)
            theta = inputArray(1);
            d = inputArray(2);
            a = inputArray(3);
            alpha = inputArray(4);

            tRotZ = [cos(theta), -sin(theta), 0, 0;
                     sin(theta),  cos(theta), 0, 0;
                              0,           0, 1, 0;
                              0,           0, 0, 1];
            
            tTransZ = [1,0,0,0;
                       0,1,0,0;
                       0,0,1,d;
                       0,0,0,1];

            tTransX = [1,0,0,a;
                       0,1,0,0;
                       0,0,1,0;
                       0,0,0,1];

            tRotX = [1,          0,           0, 0;
                     0, cos(alpha), -sin(alpha), 0;
                     0, sin(alpha),  cos(alpha), 0;
                     0,          0,           0, 1];

            returnTM = tRotZ * tTransZ * tTransX * tRotX;
        end

        function returnFKM = dh2fk(packet, inputMatrix, targetFrameIndex)
            % inputMatrix: corresponding to the n rows of the full DH
            % parameter table 
            % returnFKM = T0_4(FK matrix) =  T0_1 * T1_2 * T2_3 * T3_4
            index = 1;
            returnFKM = eye(4);
            while index <= targetFrameIndex
                currentTM = packet.dh2mat(inputMatrix(index,:));
                returnFKM = returnFKM * currentTM;
                index = index+1;
            end
            returnFKM = returnFKM;
        end

        function returnFM = fk3001(packet, jointAnglesArray)
            % jointAnglesArray = [theta1, theta2, theta3]
            theta1 = jointAnglesArray(1);
            theta2 = jointAnglesArray(2);
            theta3 = jointAnglesArray(3);
            %{
            L0 = 55; % Unit: mm
            L1 = 40; % Unit: mm
            L2 = 100; % Unit: mm
            L3 = 100; % Unit: mm
            inputMatrix = [         0, L0,  0,     0;
                               theta1, L1,  0, -pi/2;
                          theta2-pi/2,  0, L2,     0;
                          theta3+pi/2,  0, L3,     0;];
            returnFM = packet.dh2fk(inputMatrix, 4);
            %}
            returnFM = [ cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180), -sin((pi*theta1)/180), 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*cos((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*cos((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                         sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180), - sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - sin((pi*theta1)/180)*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),  cos((pi*theta1)/180), 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180) + 100*sin((pi*theta1)/180)*cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180) - 100*sin((pi*theta1)/180)*sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180);
                                                                 - cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180),                                             sin((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - cos((pi*(theta2 - 90))/180)*cos((pi*(theta3 + 90))/180),                     0,                                                           95 - 100*cos((pi*(theta2 - 90))/180)*sin((pi*(theta3 + 90))/180) - 100*cos((pi*(theta3 + 90))/180)*sin((pi*(theta2 - 90))/180) - 100*sin((pi*(theta2 - 90))/180);
                                                                                                                                                                                   0,                                                                                                                                                             0,                     0,                                                                                                                                                                                                                          1];
        end

        function returnJS = measured_cp(packet)
            data = packet.measured_js(1, 0);
            theta1 = data(1,1);
            theta2 = data(1,2);
            theta3 = data(1,3);

            returnJS = packet.fk3001([theta1 theta2 theta3]);
        end

        function returnSP = setpoint_cp(packet)
            setpointData = packet.setpoint_js();
            setTheta1 = setpointData(1);
            setTheta2 = setpointData(2);
            setTheta3 = setpointData(3);

            returnSP = packet.fk3001([setTheta1 setTheta2 setTheta3]);
        end

        function returnTM = goal_cp(packet)
            goalPointData = packet.goal_js();
            goalTheta1 = goalPointData(1);
            goalTheta2 = goalPointData(2);
            goalTheta3 = goalPointData(3);

            returnTM = packet.fk3001([goalTheta1 goalTheta2 goalTheta3]);
        end

        function returnT = ik3001(packet, positionArray)
            
            x = positionArray(1);
            y = positionArray(2);
            z = positionArray(3);
            
            % check if the given position is within the workspace 
            if z < 0 || z > 295
                error("Error: invalid z input");
            end
            if x > 180
                error("Error: invalid x input");
            end 
            if y > 200 || y < -200
                error("Error: invalid y input");
            end
       
       
            L0 = 55;
            L1 = 40;
            a1 = 100;
            a2 = 100;
            r = sqrt(x^2 + y^2);
            s = z - L1 - L0;
            alpha = atan2(s, r);

            % theta3:
            D3 = (a1^2+a2^2-(r^2+s^2))/(2*a1*a2);
            C3 = sqrt(1-D3^2);
            theta3 = pi/2 - atan2(C3, D3);

            % theta2: 
            l = realsqrt(r^2+s^2);
            D2 = (a1^2+r^2+s^2-a2^2)/(2*a1*l);
            C2 = sqrt(1-D2^2);
            beta = atan2(C2, D2);
            theta2 = pi/2 - alpha - beta;

            % theta1:
            D1 = x/r;
            C1 = sqrt(1-D1^2);
            %theta1 = atan2(C1, D1);
            theta1 = atan2(y, x);

            theta1 = rad2deg(theta1);
            theta2 = rad2deg(theta2);
            theta3 = rad2deg(theta3);

            returnT = [theta1; theta2; theta3];
 
        end

        % command the robot arm to go from one point to the other point
        % based on the input trajectory coefficients between two points 
        % (3x4 matrix)
        % the robot arm is moved along the triangle trajectory in advance
        % to get the time between each vertice; which can be input into the
        % cubic_traj() to get trajectory Coefficients and the totalTime;
        % bassically it alows the robot to go exat same trajectory as the
        % previous one with calculated coefficient
        function run_trajectory(self, t0, t, ifJointSpace)

            xa0 = trajectoryCoefficients(1,1);
            xa1 = trajectoryCoefficients(1,2);
            xa2 = trajectoryCoefficients(1,3);
            xa3 = trajectoryCoefficients(1,4);
            ya0 = trajectoryCoefficients(2,1);
            ya1 = trajectoryCoefficients(2,2);
            ya2 = trajectoryCoefficients(2,3);
            ya3 = trajectoryCoefficients(2,4);
            za0 = trajectoryCoefficients(3,1);
            za1 = trajectoryCoefficients(3,2);
            za2 = trajectoryCoefficients(3,3);
            za3 = trajectoryCoefficients(3,4);
            lastTimestamps = self.timeStamps(end);

            tStart = tic;
            t = toc(tStart);
            while t < totalTime+lastTimestamps
                t = toc(tStart);
                self.timeStamps = [self.timeStamps;t+lastTimestamps];
                t = t+lastTimestamps;
                currentJointPoseX = xa0 + xa1*t + xa2*t^2 + xa3*t^3;
                currentJointPoseY = ya0 + ya1*t + ya2*t^2 + ya3*t^3;
                currentJointPoseZ = za0 + za1*t + za2*t^2 + za3*t^3;
                currentJointPose = [currentJointPoseX, currentJointPoseY, currentJointPoseZ];

                if ~ifJointSpace
                    currentJointPose = transpose(self.ik3001(currentJointPose));
                end
                
                self.servo_jp(currentJointPose); % actuate the robot arm to the target value
                actualJs = self.measured_js(1, 0); % measure the joint angle value, return a 2x3 matrix
                self.actualJointAngle = [self.actualJointAngle; actualJs(1,:)]; % store the first row of actualJs to actualJointAngle
                self.jointAngleByCal = [self.jointAngleByCal; currentJointPose];

                actualPos = self.fk3001(actualJs(1,:));
                self.posArray = [self.posArray; transpose(actualPos(1:3, 4))];

                
            end
            
        end 


        
        


        
    end
end

clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
robot = Robot(myHIDSimplePacketComs); 
traj = Traj_Planner();
try
  SERV_ID = 1848;            % we will be talking to server ID 1848 on
                           % the Nucleo
  SERVER_ID_READ =1910;% ID of the read packet
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 60
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');
  
  
  J = robot.jacob3001([0 90 -90])
  disp(det(J(1:3,:)));
  
  
  
   Pos1 = [50,55,75];
   Pos2 = [75 -60 35];
   Pos3 = [140 60 90];
  
% Linear Trajectory 

i = 1; % counts iterations
prevEndpoint = [-500; -500; -500; -500]; %initialize variable to impossible values so it never overlaps
AV = robot.measured_js(1,1); 
fkAngle = transpose(AV(1, :));
qVelocity = AV(2,:);
endpoint = robot.fk3001(fkAngle)* [0; 0; 0; 1];

tic
while toc < 7
    
    if ~(robot.finished_movement(transpose(endpoint(1:3,1)), Pos2)) && (toc <= 2)
      
        x12traj = traj.linear_traj(Pos1(1,1), Pos2(1,1), 0, 2, toc);  
        y12traj = traj.linear_traj(Pos1(1,2), Pos2(1,2), 0, 2, toc);
        z12traj = traj.linear_traj(Pos1(1,3), Pos2(1,3), 0, 2, toc);
        robot.interpolate_jp(robot.ik3001([x12traj, y12traj, z12traj]),1000);
      
    elseif ~(robot.finished_movement(transpose(endpoint(1:3,1)), Pos3)) && toc > 2 && toc < 4

        x23traj = traj.linear_traj(Pos2(1,1), Pos3(1,1), 2, 4, toc);
        y23traj = traj.linear_traj(Pos2(1,2), Pos3(1,2), 2, 4, toc);
        z23traj = traj.linear_traj(Pos2(1,3), Pos3(1,3), 2, 4, toc);
        robot.interpolate_jp(robot.ik3001([x23traj, y23traj, z23traj]),1000);
            
    elseif ~(robot.finished_movement(transpose(endpoint(1:3,1)), Pos1)) && toc > 4 && toc < 6
      
        x31traj = traj.linear_traj(Pos3(1,1), Pos1(1,1), 4, 6, toc);
        y31traj = traj.linear_traj(Pos3(1,2), Pos1(1,2), 4, 6, toc);
        z31traj = traj.linear_traj(Pos3(1,3), Pos1(1,3), 4, 6, toc);  
        robot.interpolate_jp(robot.ik3001([x31traj, y31traj, z31traj]),1000);
   
    end
    
    AV = robot.measured_js(1,1);
    fkAngle = transpose(AV(1, :));
    qVelocity = transpose(AV(2,:));
    endpoint = robot.fk3001(fkAngle)* [0; 0; 0; 1];
    ee_jacob = robot.jacob3001(fkAngle);
    ee_velocity = ee_jacob(1:3,1:3)*qVelocity;
    robot.plot_arm(fkAngle,ee_velocity)
    
    if(prevEndpoint(1,1) ~= endpoint(1,1) || prevEndpoint(2,1) ~= endpoint(2,1) || prevEndpoint(3,1) ~= endpoint(3,1) && 0 ~= endpoint(1,1) && 0 ~= endpoint(2,1) && 0 ~= endpoint(3,1))
        lin_traj_m(i, 1) = toc;
        lin_traj_m(i, 2) = AV(2,1);
        lin_traj_m(i, 3) = AV(2,2);
        lin_traj_m(i, 4) = AV(2,3);

        lin_traj_m(i,5) = ee_velocity(1,1);
        lin_traj_m(i,6) = ee_velocity(2,1);
        lin_traj_m(i,7) = ee_velocity(3,1);
        lin_traj_m(i,8) = sqrt(ee_velocity(1,1)^2 + ee_velocity(2,1)^2 + ee_velocity(3,1)^2); %Calculates Scalar Velocity
        i = i+1;
    end

    prevEndpoint = endpoint;
    
end

writematrix(lin_traj_m,'linear_velocity_data.csv');
filename = 'linear_velocity_data.csv';

traj_data = csvread(filename);
time = traj_data(:,1);

% Linear Velocities
xVel = traj_data(:,5);
yVel = traj_data(:,6);
zVel = traj_data(:,7);

%Angular Velocities
xAngVel = traj_data(:,2);
yAngVel = traj_data(:,3);
zAngVel = traj_data(:,4);

%Scalar Velocity
ScalarVel = traj_data(:,8);

subplot(3,2,1)
plot(time,xVel); 
hold on
plot(time,yVel);
plot(time,zVel);
hold off

title("End Effector Linear Velocity");
xlabel('Time(s)') ;
ylabel('Linear Velocity(mm/s)'); 
legend('X Velocity', 'Y Velocity', 'Z Velocity');

subplot(3,2,2)
plot(time,xAngVel); 
hold on
plot(time,yAngVel);
plot(time,zAngVel);
hold off

title("End Effector Angular Velocity");
xlabel('Time(s)') ;
ylabel(' Angular Velocity(deg/s)'); 
legend('X Angular Velocity', 'Y Angular Velocity', 'Z Angular Velocity');
  
subplot(3,2,3)
plot(time,ScalarVel); 

title("End Effector Scalar Linear Velocity");
xlabel('Time(s)') ;
ylabel(' Scalar Linear Velocity(deg/s)'); 
legend('Scalar Linear Velocity');
  
  catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

% tozc
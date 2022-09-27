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
  
% Linear Trajectory 
Pos1 = [50,55,75];
Pos2 = [100 0 295];
Ang1 = robot.ik3001(Pos1);
Ang2 = [0 0 -90];

robot.interpolate_jp(robot.ik3001(Pos1),1000);
pause(1)

i = 1; % counts iterations
prevEndpoint = [-500; -500; -500; -500]; %initialize variable to impossible values so it never overlaps
AV = robot.measured_js(1,1); 
fkAngle = transpose(AV(1, :));
qVelocity = AV(2,:);
endpoint = robot.fk3001(fkAngle)* [0; 0; 0; 1];
eStop = 0;
ee_jacob = robot.jacob3001(fkAngle);
detJ = det(ee_jacob(1:3,:))
x12traj = -500;
y12traj = -500;
z12traj = -500;

%generate trajectory array
xtip_array = nan(1);
ytip_array = nan(1);
ztip_array = nan(1);

tic
while toc < 6 && eStop == 0;
    T = robot.measured_cp();
    xtip_array(length(xtip_array)+1) = T(1,4);
    ytip_array(length(ytip_array)+1) = T(2,4);
    ztip_array(length(ztip_array)+1) = T(3,4);

    if abs(detJ) < 2000
        eStop = 1;
         disp("Singularity Detected")
        
    end
    

    if ~(robot.finished_movement(robot.fk3001([x12traj y12traj z12traj]), Ang2)) && eStop == 0 && toc <= 5
            
        if(eStop == 0)    
        x12traj = traj.cubic_poly_traj(0,5,0,0,Ang1(1,1), Ang2(1,1), toc);  
        y12traj = traj.cubic_poly_traj(0,5,0,0, Ang1(1,2), Ang2(1,2), toc);
        z12traj = traj.cubic_poly_traj(0,5,0,0, Ang1(1,3), Ang2(1,3), toc);
        robot.interpolate_jp([x12traj, y12traj, z12traj],100);

        end
        
    end
    
AV = robot.measured_js(1,1);
    fkAngle = transpose(AV(1, :));
    qVelocity = transpose(AV(2,:));
    endpoint = robot.fk3001(fkAngle)* [0; 0; 0; 1];
    ee_jacob = robot.jacob3001(fkAngle);
    ee_velocity = ee_jacob(1:3,1:3)*qVelocity;
    robot.plot_arm(fkAngle,ee_velocity)

     if (robot.finished_movement(transpose(fkAngle), Ang2)) && eStop == 0 
        
        if (detJ > 2000)
         eStop = 1;
        end
        
    end
    
    if eStop == 1
        msg = 'Error occurred.';
        disp("Singularity Detected")

    end
    
    AV = robot.measured_js(1,1);
    fkAngle = transpose(AV(1, :))
    qVelocity = transpose(AV(2,:));
    endpoint = robot.fk3001(fkAngle)* [0; 0; 0; 1];
    ee_velocity = ee_jacob(1:3,1:3)*qVelocity;
    robot.plot_arm(fkAngle,ee_velocity);
    ee_jacob = robot.jacob3001(fkAngle);
    detJ = det(ee_jacob(1:3,:))
    
    if(prevEndpoint(1,1) ~= endpoint(1,1) || prevEndpoint(2,1) ~= endpoint(2,1) || prevEndpoint(3,1) ~= endpoint(3,1) && 0 ~= endpoint(1,1) && 0 ~= endpoint(2,1) && 0 ~= endpoint(3,1))
        M(i, 1) = toc;
        M(i, 2) = detJ;
      
        i = i+1;
    end

    prevEndpoint = endpoint;
    
end

%plot trajectory
plot3(xtip_array, ytip_array, ztip_array);
axis([-200 200 -200 200 0 400]);
writematrix(M,'DetJ_data.csv');
filename = 'DetJ_data.csv';
motionData = csvread(filename);

time = motionData(:,1);
DetJ = motionData(:,2);

plot(time,DetJ);

title("Determinant vs Time");
xlabel('Time(ms)') ;
ylabel('Determinant'); 

  
  catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
robot.shutdown()

% toc
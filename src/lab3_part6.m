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
model2 = Model2();
traj = Traj_Planner();
%--------------------------------------------------------------------------

% calculate the trajectory coefficient based on position
% arm get to P1 in advance of trajectory
robot.interpolate_jp([-37.68, 51.58, -42.13], 1000)
pause(2)

% P1-P2
TP1_2_1 = traj.cubic_traj(0, 2, 0, 0, 140.0769, 145.0102);
TP1_2_2 = traj.cubic_traj(0, 2, 0, 0, -108.1856, 1.8223);
TP1_2_3 = traj.cubic_traj(0, 2, 0, 0, 140.7234, 84.8718);
TPC1 = [TP1_2_1; TP1_2_2; TP1_2_3];
robot.run_trajectory(TPC1, 2, 0);
% P2-P3
TP2_3_1 = traj.cubic_traj(2, 4, 0, 0, 145.0102, 100);
TP2_3_2 = traj.cubic_traj(2, 4, 0, 0, 1.8223, 132.7);
TP2_3_3 = traj.cubic_traj(2, 4, 0, 0, 84.8718, 149.1);
TPC2 = [TP2_3_1; TP2_3_2; TP2_3_3];
robot.run_trajectory(TPC2, 2, 0);
% P3-P1
TP3_1_1 = traj.cubic_traj(4, 6, 0, 0, 100, 140.0769);
TP3_1_2 = traj.cubic_traj(4, 6, 0, 0, 132.7, -108.1856);
TP3_1_3 = traj.cubic_traj(4, 6, 0, 0, 149.1, 140.7234);
TPC3 = [TP3_1_1; TP3_1_2; TP3_1_3];
robot.run_trajectory(TPC3, 2, 0);
%--------------------------------------------------------------------------

% plotting end effector position vs. time
i = 1;
actualJointAngle = robot.actualJointAngle;
eePosArray = [];
row_num = size(actualJointAngle);
for i = (1 : row_num(1))
    currentJA = actualJointAngle(i,:);
    currentPos = robot.fk3001(currentJA);
    X = currentPos(1,4);
    Y = currentPos(2,4);
    Z = currentPos(3,4);
    eePosArray = [eePosArray; X, Y, Z];
    
end

% End Effector position vs. Time
figure
timeStamps = robot.timeStamps(2:end);
plot(timeStamps, eePosArray(:,1))
hold on
plot(timeStamps, eePosArray(:,2))
plot(timeStamps, eePosArray(:,3))
xlabel("Time(s)");
ylabel("End effector position(mm)")
title("End Effector position vs. Time")
legend("X axis","Y axis", "Z axis")
%--------------------------------------------------------------------------

figure
% use for loop to get the time boundary of 0-2, 2-4, 4-6
for i = (1 : size(timeStamps))
    currentTime = timeStamps(i);
    if timeStamps(i) <2 && timeStamps(i+1) >2
        indexOf2 = i+1;
    end
    if timeStamps(i) <4 && timeStamps(i+1) >4
        indexOf4 = i+1;
    end
end

% get the timearray of each sec
timeArray0_2 = timeStamps(1:indexOf2);
timeArray2_4 = timeStamps(indexOf2:indexOf4);
timeArray4_6 = timeStamps(indexOf4:end);
t0_2 = timeArray0_2;
t2_4 = timeArray2_4;
t4_6 = timeArray4_6;

% functions for sec1, from 0s to 2s
q0_2X = TPC1(1,2) + 2*TPC1(1,3)*t0_2 + 3*TPC1(1,4)*t0_2.^2;
q0_2Y = TPC1(2,2) + 2*TPC1(2,3)*t0_2 + 3*TPC1(2,4)*t0_2.^2;
q0_2Z = TPC1(3,2) + 2*TPC1(3,3)*t0_2 + 3*TPC1(3,4)*t0_2.^2;
% functions for sec2, from 2s to 4s
q2_4X = TPC2(1,2) + 2*TPC2(1,3)*t2_4 + 3*TPC2(1,4)*t2_4.^2;
q2_4Y = TPC2(2,2) + 2*TPC2(2,3)*t2_4 + 3*TPC2(2,4)*t2_4.^2;
q2_4Z = TPC2(3,2) + 2*TPC2(3,3)*t2_4 + 3*TPC2(3,4)*t2_4.^2;
% functions for sec3, from 4s to 6s
q4_6X = TPC3(1,2) + 2*TPC3(1,3)*t4_6 + 3*TPC3(1,4)*t4_6.^2;
q4_6Y = TPC3(2,2) + 2*TPC3(2,3)*t4_6 + 3*TPC3(2,4)*t4_6.^2;
q4_6Z = TPC3(3,2) + 2*TPC3(3,3)*t4_6 + 3*TPC3(3,4)*t4_6.^2;

plot(timeArray0_2, q0_2X, 'r')
hold on
plot(timeArray0_2, q0_2Y, 'g')
plot(timeArray0_2, q0_2Z, 'b')

plot(timeArray2_4, q2_4X, 'r')
plot(timeArray2_4, q2_4Y, 'g')
plot(timeArray2_4, q2_4Z, 'b')

plot(timeArray4_6, q4_6X, 'r')
plot(timeArray4_6, q4_6Y, 'g')
plot(timeArray4_6, q4_6Z, 'b')

title("Velocity along each axis vs. Time")
xlabel("Time(s)");
ylabel("velocity along each axis(mm/s)");
legend("X axis", "Y axis", "Z axis");
%--------------------------------------------------------------------------

% plotting for the accelaration

% functions for sec1, from 0s to 2s
q0_2Xa = 2*TPC1(1,3) + 6*TPC1(1,4)*t0_2;
q0_2Ya = 2*TPC1(2,3) + 6*TPC1(2,4)*t0_2;
q0_2Za = 2*TPC1(3,3) + 6*TPC1(3,4)*t0_2;
% functions for sec2, from 2s to 4s
q2_4Xa = 2*TPC2(1,3) + 6*TPC2(1,4)*t2_4;
q2_4Ya = 2*TPC2(2,3) + 6*TPC2(2,4)*t2_4;
q2_4Za = 2*TPC2(3,3) + 6*TPC2(3,4)*t2_4;
% functions for sec3, from 4s to 6s
q4_6Xa = 2*TPC3(1,3) + 6*TPC3(1,4)*t4_6;
q4_6Ya = 2*TPC3(2,3) + 6*TPC3(2,4)*t4_6;
q4_6Za = 2*TPC3(3,3) + 6*TPC3(3,4)*t4_6;

figure
plot(timeArray0_2, q0_2Xa, 'r')
hold on
plot(timeArray0_2, q0_2Ya, 'g')
plot(timeArray0_2, q0_2Za, 'b')

plot(timeArray2_4, q2_4Xa, 'r')
plot(timeArray2_4, q2_4Ya, 'g')
plot(timeArray2_4, q2_4Za, 'b')

plot(timeArray4_6, q4_6Xa, 'r')
plot(timeArray4_6, q4_6Ya, 'g')
plot(timeArray4_6, q4_6Za, 'b')

title("Accelerations along each axis vs. Time")
xlabel("Time(s)");
ylabel("Accelerations along each axis(mm/s^2)");
legend("X axis", "Y axis", "Z axis");
%--------------------------------------------------------------------------

% plot 3d trajectory
figure
plot3(robot.posArray(:,1),robot.posArray(:,2),robot.posArray(:,3))
axis([100 180 -100 100 80 150])
title("3D Position Trajectory");
xlabel('X-Position');
ylabel('Y-Position');
zlabel('Z-Position');  

%--------------------------------------------------------------------------
% store posArray into a csv file
writematrix(robot.posArray, 'task_space_posArray.csv')




% Clear up memory upon termination
robot.shutdown()

%toc
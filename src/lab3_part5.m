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

%{ 
% Code to get the time duration for each edge of the triangle 
robot.interpolate_jp([52.991 42.8602 -31.7898], 1000)
pause(5)

tStart = tic;
startTime = toc(tStart);
while true
currentAngle = robot.measured_js(1, 0);
currentX = currentAngle(1,1);
robot.servo_jp([-37.68 51.58 -42.13])
if currentX <= -37.6 
    endTime = toc(tStart);
    disp(endTime)
end
end 
%}
%--------------------------------------------------------------------------

% apply run_trajectory func to actuate robot arm follow the triangle
% P1-P2
% arm get to P1 in advance of trajectory
robot.interpolate_jp([-37.68, 51.58, -42.13], 1000)
pause(2)

% calculate the trajectory coefficient
T1_2_1 = traj.cubic_traj(0, 2, 0, 0, -37.68, 0.72);
T1_2_2 = traj.cubic_traj(0, 2, 0, 0, 51.58, 50.62);
T1_2_3 = traj.cubic_traj(0, 2, 0, 0, -42.13, -3.25);
TC1 = [T1_2_1; T1_2_2; T1_2_3];

robot.run_trajectory(TC1, 2);

% P2-P3
% calculate the trajectory coefficient
T2_3_1 = traj.cubic_traj(0, 2, 0, 0, 0.72, 52.9991);
T2_3_2 = traj.cubic_traj(0, 2, 0, 0, 50.62, 42.8602);
T2_3_3 = traj.cubic_traj(0, 2, 0, 0, -3.25, -31.7898);
TC2 = [T2_3_1; T2_3_2; T2_3_3];

robot.run_trajectory(TC2, 2);

% P3-P1
% calculate the trajectory coefficient
T3_1_1 = traj.cubic_traj(0, 2, 0, 0, 52.9991, -37.68);
T3_1_2 = traj.cubic_traj(0, 2, 0, 0, 42.8602, 51.58);
T3_1_3 = traj.cubic_traj(0, 2, 0, 0, -31.7898, -42.13);
TC3 = [T3_1_1; T3_1_2; T3_1_3];

robot.run_trajectory(TC3, 2);
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

% plotting velociy along each axis vs. time
% calculate the trajectory coefficient based on position
TP1_2_1 = traj.cubic_traj(0, 2, 0, 0, 140.0769, 145.0102);
TP1_2_2 = traj.cubic_traj(0, 2, 0, 0, -108.1856, 1.8223);
TP1_2_3 = traj.cubic_traj(0, 2, 0, 0, 140.7234, 84.8718);
TPC1 = [TP1_2_1; TP1_2_2; TP1_2_3];
% P2-P3
% calculate the trajectory coefficient based on position
TP2_3_1 = traj.cubic_traj(2, 4, 0, 0, 145.0102, 100);
TP2_3_2 = traj.cubic_traj(2, 4, 0, 0, 1.8223, 132.7);
TP2_3_3 = traj.cubic_traj(2, 4, 0, 0, 84.8718, 149.1);
TPC2 = [TP2_3_1; TP2_3_2; TP2_3_3];
% P3-P1
% calculate the trajectory coefficient based on position
TP3_1_1 = traj.cubic_traj(4, 6, 0, 0, 100, 140.0769);
TP3_1_2 = traj.cubic_traj(4, 6, 0, 0, 132.7, -108.1856);
TP3_1_3 = traj.cubic_traj(4, 6, 0, 0, 149.1, 140.7234);
TPC3 = [TP3_1_1; TP3_1_2; TP3_1_3];

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



% Clear up memory upon termination
robot.shutdown()

%toc
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

% apply run_trajectory func to actuate robot arm follow the triangle
% arm get to P1 in advance of trajectory
robot.interpolate_jp([47.73, 37.69, 44.72], 1000);
%%
disp(robot.ik3001([63 -44 32]));
disp(robot.fk3001([-34.9309 69.1375 30.417]));
%%
%    Point1 = [50,55,75];
%    Point2 = [50 0 35];
%    Point3 = [140 60 90];
   %robot.servo_jp([50 55 75]);

   pause(2)

% P1-P2

T1_2_1 = traj.cubic_traj(0, 2, 0, 0, 47.73, 0.0);
T1_2_2 = traj.cubic_traj(0, 2, 0, 0, 37.69, 73.18);
T1_2_3 = traj.cubic_traj(0, 2, 0, 0, 44.72, 44.02);
TC1 = [T1_2_1; T1_2_2; T1_2_3];

robot.run_trajectory(TC1, 2, 1);

% P2-P3

T2_3_1 = traj.cubic_traj(2, 4, 0, 0, 0.0, 23.19);
T2_3_2 = traj.cubic_traj(2, 4, 0, 0, 73.18, 51.52);
T2_3_3 = traj.cubic_traj(2, 4, 0, 0, 44.02, -9.28);
TC2 = [T2_3_1; T2_3_2; T2_3_3];

robot.run_trajectory(TC2, 2, 1);

% P3-P1

T3_1_1 = traj.cubic_traj(4, 6, 0, 0, 23.19, 47.73);
T3_1_2 = traj.cubic_traj(4, 6, 0, 0, 51.52, 37.69);
T3_1_3 = traj.cubic_traj(4, 6, 0, 0, -9.28, 44.72);
TC3 = [T3_1_1; T3_1_2; T3_1_3];

robot.run_trajectory(TC3, 2, 1);


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
subplot(2,2,1)
timeStamps = robot.timeStamps(2:end);
plot(timeStamps, eePosArray(:,1))
hold on
plot(timeStamps, eePosArray(:,2))
plot(timeStamps, eePosArray(:,3))
xlabel("Time(s)");
ylabel("End effector position(mm)")
title("End Effector position vs. Time")
legend("X axis","Y axis", "Z axis")


% plotting velociy along each axis vs. time
%P1-P2
TP1_2_1 = traj.cubic_traj(0, 2, 0, 0, 50.0078, 50.0121);
TP1_2_2 = traj.cubic_traj(0, 2, 0, 0, 55.0157, 0);
TP1_2_3 = traj.cubic_traj(0, 2, 0, 0, 75.0092, 34.995);
TPC1 = [TP1_2_1; TP1_2_2; TP1_2_3];
% P2-P3

TP2_3_1 = traj.cubic_traj(2, 4, 0, 0, 50.0121, 140.0096);
TP2_3_2 = traj.cubic_traj(2, 4, 0, 0, 0, 59.9793);
TP2_3_3 = traj.cubic_traj(2, 4, 0, 0, 34.995, 90.0004);
TPC2 = [TP2_3_1; TP2_3_2; TP2_3_3];
% P3-P1

TP3_1_1 = traj.cubic_traj(4, 6, 0, 0, 140.0096, 50.0078);
TP3_1_2 = traj.cubic_traj(4, 6, 0, 0, 59.9793, 55.0157);
TP3_1_3 = traj.cubic_traj(4, 6, 0, 0, 90.0004, 75.0092);
TPC3 = [TP3_1_1; TP3_1_2; TP3_1_3];

subplot(2,2,2)

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

%plotting Velocity
v_x_1 = traj.takeDeri(eePosArray(:,1), timeStamps);
v_x_2 = traj.takeDeri(eePosArray(:,2), timeStamps);
v_x_3 = traj.takeDeri(eePosArray(:,3), timeStamps);

plot(timeStamps, v_x_1)
hold on
plot(timeStamps, v_x_2)
hold on
plot(timeStamps, v_x_3)
hold on

title("Velocity along each axis vs. Time")
xlabel("Time(s)");
ylabel("velocity along each axis(mm/s)");
axis([0 6 -5000 5000])
legend("X axis", "Y axis", "Z axis");



% plotting for the accelaration


a_x_1 = traj.takeDeri(v_x_1, timeStamps);
a_x_2 = traj.takeDeri(v_x_2, timeStamps);
a_x_3 = traj.takeDeri(v_x_3, timeStamps);

hold on
subplot(2,2,3)
plot(timeStamps, a_x_1)
hold on
plot(timeStamps, a_x_2)
hold on
plot(timeStamps, a_x_3)

title("Accelerations along each axis vs. Time")
xlabel("Time(s)");
ylabel("Accelerations along each axis(mm/s^2)");
axis([0 6 -4000000 4000000])
legend("X axis", "Y axis", "Z axis");


% plot 3d trajectory
figure
plot3(robot.posArray(:,1),robot.posArray(:,2),robot.posArray(:,3))
axis([0 180 -100 100 0 200])
title("3D Position Trajectory");
xlabel('X-Position');
ylabel('Y-Position');
zlabel('Z-Position');

writematrix(robot.posArray, 'joint_space_posArray.csv')

% Clear up memory upon termination
robot.shutdown()

%toc
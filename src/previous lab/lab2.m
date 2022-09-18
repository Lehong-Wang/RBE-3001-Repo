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
% Active arm plot test
%{
robot.interpolate_jp([90 30 30], 10000)

while true
disp('measured_cp:')
robot.measured_cp()
disp('setpoint_cp:')
robot.setpoint_cp()
disp('goal_cp:')
robot.goal_cp()
pause(1)
end
%}

robot.interpolate_jp([0 0 0], 1000)
pause(1.5)


% Part 8 Motion Planning in joint space
  ifReach = 0;
  trajectory1 = [];
  timeStamps1 = [];
  position1 = [];
  robot.interpolate_jp([0 15 0], 1000)
  tStart1_1 = tic;
  while( ifReach < 1)
      currentAng = robot.measured_js(1, 0);
      tCurrent1 = toc(tStart1_1);
      positionData = robot.measured_cp();
      position1 = [position1; transpose(positionData(1:3,4))];
      trajectory1 = [trajectory1; currentAng(1,:)];
      timeStamps1 = [timeStamps1; tCurrent1];
      if currentAng(1,2) >= 13
          ifReach = 1;

      end
  end

  ifReach = 0;
  trajectory2 = [];
  timeStamps2 = [];
  position2 = [];
  robot.interpolate_jp([0 45 -40], 1000)
  while( ifReach < 1)
      currentAng = robot.measured_js(1, 0);
      tCurrent2 = toc(tStart1_1);
      positionData = robot.measured_cp();
      position2 = [position2; transpose(positionData(1:3,4))];
      trajectory2 = [trajectory2; currentAng(1,:)];
      timeStamps2 = [timeStamps2; tCurrent2];
      if currentAng(1,2) >= 43
          ifReach = 1;

      end
  end


  ifReach = 0;
  trajectory3 = [];
  timeStamps3 = [];
  position3 = [];
  robot.interpolate_jp([0 50 40], 1000)
  while( ifReach < 1)
      currentAng = robot.measured_js(1, 0);
      tCurrent3 = toc(tStart1_1);
      positionData = robot.measured_cp();
      position3 = [position3; transpose(positionData(1:3,4))];
      trajectory3 = [trajectory3; currentAng(1,:)];
      timeStamps3 = [timeStamps3; tCurrent3];
      if currentAng(1,2) >= 48
          ifReach = 1;

      end
  end

  ifReach = 0;
  trajectory4 = [];
  timeStamps4 = [];
  position4 = [];
  robot.interpolate_jp([0 15 0], 1000)
  while( ifReach < 1)
      currentAng = robot.measured_js(1, 0);
      tCurrent4 = toc(tStart1_1);  
      positionData = robot.measured_cp();
      position4 = [position4; transpose(positionData(1:3,4))];
      trajectory4 = [trajectory4; currentAng(1,:)];
      timeStamps4 = [timeStamps4; tCurrent4];
      if currentAng(1,2) <= 17
          ifReach = 1;
      end
  end


  timeStamps = [timeStamps1; timeStamps2; timeStamps3; timeStamps4];
  trajectory = [trajectory1; trajectory2; trajectory3; trajectory4];
  position = [position1; position2; position3; position4];
  motionT = [timeStamps trajectory position];
  writematrix(motionT, 'xz_joint_position.csv')

% Clear up memory upon termination
robot.shutdown()

%toc

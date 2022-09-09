% This  Model  class  should  reference  your Robot class for the kinematics calculations
classdef Model

    properties
    end

    methods (Static)

        % takes a 3x1 array of joint values and plots a stick model of 
        % the arm showing all frames, joints, and links.  
        function T = plot_arm(jointValues)
            L0 = 55;
            L1 = 40;
            L2 = 100;
            L3 = 100;
            % use joint values to DH parameters function
            % use dh2mat to get all intermediate matrices
            % use XYZkine example to convert to usable vectors
            %vectors = Matrix.dh2fk()
            %vectors = Matrix.dh2fk()
            plot3(vectors(1,:), vectors(2,:), vectors(3,:), '-o', 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', [0.5,0.5,0.5]);
            grid on;
            title("Manipulator Visualizer")
            xlabel('X Axis');
            ylabel('Y Axis');
            zlabel('Z Axis');
            axis([-20 20 0 20 0 20]);
            x = rotate3d;
            x.Enable = 'on';
        end
    end
end
% This  Model  class  should  reference  your Robot class for the kinematics calculations
classdef Model

    properties
    end

    methods (Static)

        % takes a 3x1 array of joint values and plots a stick model of 
        % the arm showing all frames, joints, and links.  
        function T = plot_arm(self)
            L0 = 55;
            L1 = 40;
            L2 = 100;
            L3 = 100;
        end
    end
end
classdef Traj_Planner

    properties
    end

    methods (Static)
        
        % solves for a cubic (3rd order) polynomial trajectory between two viapoints. It should take in desired starting and ending times t0 and tf (in seconds), starting and ending 
        % velocities, and starting and ending positions. It should output a 4-by-1 array containing the coefficients
        % 𝑎𝑖, i = 0,1,2,3 of the polynomial. 
        function T = cubic_traj()
        end
        
        % solves for a  quintic polynomial trajectory (5th order) between two set-points. 
        % It should take in desired starting and ending times t0 and tf (in seconds), 
        % starting and ending velocities, starting and ending accelerations, and starting 
        % and ending positions. It should output a 6-by-1 array containing the coefficients
        % 𝑎𝑖, i = 0,1,2,3,4,5 of the polynomial. 
        function T = quintic_traj()
        end
    end
end
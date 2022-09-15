classdef Traj_Planner

    properties
    end

    methods (Static)
        
        % solves for a cubic (3rd order) polynomial trajectory between two viapoints. It should take in desired starting and ending times t0 and tf (in seconds), starting and ending 
        % velocities, and starting and ending positions. It should output a 4-by-1 array containing the coefficients
        % 𝑎𝑖, i = 0,1,2,3 of the polynomial. 
        function T = cubic_traj(t0, tf, v0, vf, q0, qf)
            matrix = [ 1    t0  t0^2    t0^3;
                       0    1   2*t0    3*t0^2;
                       1    tf  tf^2    tf^3;
                       0    1   2*tf    3*tf^2; ]
        end
        
        % solves for a  quintic polynomial trajectory (5th order) between two set-points. 
        % It should take in desired starting and ending times t0 and tf (in seconds), 
        % starting and ending velocities, starting and ending accelerations, and starting 
        % and ending positions. It should output a 6-by-1 array containing the coefficients
        % 𝑎𝑖, i = 0,1,2,3,4,5 of the polynomial. 
        function T = quintic_traj(t0, tf, v0, vf, q0, qf, a0, af)
             matrix = [ 1   t0  t0^2   t0^3     t0^4        t0^5;
                        0   1   2*t0   3*t0^2   4*t0^3      5*t0^4;
                        0   0   2      6*t0     12*t0^2     20*t0^3;
                        1   tf  tf^2   tf^3     tf^4        tf^5;
                        0   1   2*tf   3*tf^2   4*tf^3      5*tf^4;
                        0   0   2      6*tf     12*tf^2     20*tf^3; ]
        end
    end
end
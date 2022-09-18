classdef Traj_Planner < handle
    
    properties
        robot;
    end

    methods
        function self = Traj_Planner()
        end
        
        % Solves for a cubic(3rd order) polynomial trajectory between two
        % via-pionts 
        % Input: 
        function T = cubic_traj(self, t0, tf, vel0, velf, q0, qf)
            M = [1,  t0,  t0^2,    t0^3;
                 0,   1,  2*t0,  3*t0^2;
                 1,  tf,  tf^2,    tf^3;
                 0,   1,  2*tf,  3*tf^2];
            Given = [q0; vel0; qf; velf];
            T = transpose(inv(M)*Given);
        end
        
    end
end
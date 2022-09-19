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

        function a = quintic_traj(self, posi, posf, ti, tf, ai, af, vi, vf)
            M = [1 ti ti^2 ti^3 ti^4 ti^5;
                0 1 2*ti 3*(ti^2) 4*(ti^3) 5*(ti^4);
                0 0 2 6*ti 12*(ti^2) 20*(ti^3);
                1 tf tf^2 tf^3 tf^4 tf^5;
                0 1 2*tf 3*(tf^2) 4*(tf^3) 5*(tf^4);
                0 0 2 6*tf 12*(tf^2) 20*(tf^3)]
           
            givenVals = [posi; vi; ai; posf; vf; af];
           
            a = M\givenVals;
        end
        
    end
end
classdef Traj_Planner

    methods (Static)

        
        % Solves for a cubic(3rd order) polynomial trajectory between two
        % via-pionts 
        % Input: 
        function T = cubic_traj(t0, tf, vel0, velf, q0, qf)
            M = [1,  t0,  t0^2,    t0^3;
                 0,   1,  2*t0,  3*t0^2;
                 1,  tf,  tf^2,    tf^3;
                 0,   1,  2*tf,  3*tf^2];
            Given = [q0; vel0; qf; velf];
            T = transpose(inv(M)*Given);
        end

        function a = quintic_traj(posi, posf, ti, tf, ai, af, vi, vf)
            M = [1 ti ti^2 ti^3 ti^4 ti^5;
                0 1 2*ti 3*(ti^2) 4*(ti^3) 5*(ti^4);
                0 0 2 6*ti 12*(ti^2) 20*(ti^3);
                1 tf tf^2 tf^3 tf^4 tf^5;
                0 1 2*tf 3*(tf^2) 4*(tf^3) 5*(tf^4);
                0 0 2 6*tf 12*(tf^2) 20*(tf^3)]
           
            givenVals = [posi; vi; ai; posf; vf; af];
           
            a = M\givenVals;
        end


        % for q0, qf, v0, vf, take in 3*1 / 1*3 matrix
        function T = get_cub_coef(t0, tf, q0, qf, v0, vf)
            T_x = Traj_Planner.cubic_traj(t0, tf, v0(1), vf(1), q0(1), qf(1))
            T_y = Traj_Planner.cubic_traj(t0, tf, v0(2), vf(2), q0(2), qf(2))
            T_Z = Traj_Planner.cubic_traj(t0, tf, v0(3), vf(3), q0(3), qf(3))
            T = [T_x; T_y; T_z]
        end
        
    end
end
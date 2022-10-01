

classdef Traj_Planner

    properties
    end

    methods (Static)




        function A = cubic_traj(t0, tf, q0, qf, v0, vf)
            M = [1 t0 t0^2 t0^3;
                 0 1 2*t0^2 3*t0^3;
                 1 tf tf^2 tf^3;
                 0 1 2*tf^2 3*tf^3;]
            B = [q0; v0; qf; vf;]

            A = inv(M) * B

%             M(3,:) * A


        end



































    end

end


























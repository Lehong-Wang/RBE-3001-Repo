
% function untitled()
% end


% dynamic([4 5 6], c=8)
% universal_run_traj(0,0,0,1,1)
% 
% function dynamic(a, b, c)
%     arguments
%         a (3,1) = 1
%         b = 2
%         c = 3
%     end
%     disp(a);
%     disp(b)
%     disp(c)
% end

t0 = 0;
tf = 2;
q0 = [1 2 3];
qf = [1 2 3];
v0 = [1 2 3];
vf = [1 2 3];
a0 = [1 2 3];
af = [1 2 3];

universal_run_traj(0, t0, tf, q0, qf, v0, vf)

function universal_run_traj(self, t0, tf, q0, qf, v0, vf, a0, af)
    arguments
        self
        t0
        tf
        q0 (3,1)
        qf (3,1)
        v0 (3,1) = NaN
        vf (3,1) = NaN
        a0 (3,1) = NaN
        af (3,1) = NaN
    end
%     disp(nargin)
        if nargin ~= 5 && nargin~= 7 && nargin ~= 9
            error("Input not valid");
        end
        M = [1 t0 t0^2 t0^3 t0^4 t0^5;
             0 1 2*t0 3*(t0^2) 4*(t0^3) 5*(t0^4);
             0 0 2 6*t0 12*(t0^2) 20*(t0^3);
             1 tf tf^2 tf^3 tf^4 tf^5;
             0 1 2*tf 3*(tf^2) 4*(tf^3) 5*(tf^4);
             0 0 2 6*tf 12*(tf^2) 20*(tf^3)];
        T = zeros(0);
        for i = (1:3)
            
            arg_val = [q0(i); qf(i); v0(i); vf(i); a0(i); af(i)];
            T(i,:) = M\arg_val;
        end

        disp(T);


end

























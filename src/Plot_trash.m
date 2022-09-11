

classdef Plot_trash

    properties 
        current_plot = plot3(1,2,3);
    end


    methods (Static)

        % pos_mat should be formated as 
        % [ link_0_x   link_0_y    link_0_z
        %   link_1_x    link_1_y    link_1_z
        %   ...
        function plot_pos (pos_mat)

%             try
%                 delete(plot_line);
%                 delete(plot_dot);
%             catch exception
%             end
            clf;
            X = zeros(1);
            Y = zeros(1);
            Z = zeros(1);
%             disp(Plot.current_plot);
            for i = (1:width(pos_mat))
                X(1,i) = pos_mat(i,1);
                Y(1,i) = pos_mat(i,2);
                Z(1,i) = pos_mat(i,3);
            end
            hold on
            plot_line = plot3(X,Y,Z);
            plot_dot = plot3(X,Y,Z,'.');
%             hold off
        end

        
%         function 




    end



end






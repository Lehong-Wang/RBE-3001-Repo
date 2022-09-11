% 
% 
% 
% 
% % plot_x = zeros(1,1);
% % plot_y = zeros(1,1);
% % plot_z = zeros(1,1);
% 
% classdef Plot
% 
% 
%     properties
%         plot_line = plot3(1,2,3);
% %         plot_x = zeros(1,1);
% %         plot_y = zeros(1,1);
% %         plot_z = zeros(1,1);
% 
%     end
% 
%     methods
% 
% 
%         function self = Plot()
%             global plot_x plot_y plot_z
%             plot_x = zeros(1,1);
%             plot_y = zeros(1,1);
%             plot_z = zeros(1,1);
% 
% %             X = plot_x
% %             Y = plot_y
% %             Z = plot_z
% 
%             self.plot_line = plot3(plot_x, plot_y, plot_z);
%             grid on
%             self.plot_line.XDataSource = 'plot_x';
%             self.plot_line.YDataSource = 'plot_y';
%             self.plot_line.ZDataSource = 'plot_z';        
%         end
% 
% 
%         % pos_mat should be formated as 
%         % [ link_0_x   link_0_y    link_0_z
%         %   link_1_x    link_1_y    link_1_z
%         %   ...
% 
%         function plot_pos(self, pos_mat)
%             global plot_x plot_y plot_z
% 
%             for i = (1:height(pos_mat))
%                 plot_x(1,i) = pos_mat(i,1);
%                 plot_y(1,i) = pos_mat(i,2);
%                 plot_z(1,i) = pos_mat(i,3);
%             end
% 
%             refreshdata
%             drawnow
% 
%         end
% 
%     end
% 
% 
% end
% 
% 







function [plot_pos] = Plot
    plot_pos = @plot_pos;
%     global X Y Z

end






% pos_mat should be formated as
% [ link_0_x   link_0_y    link_0_z
%   link_1_x    link_1_y    link_1_z
%   ...
function plot_pos (pos_mat)

    % clf;
    
    %             disp(Plot.current_plot);

    X = zeros(1);
    Y = zeros(1);
    Z = zeros(1);

    for i = (1:width(pos_mat))
        X(1,i) = pos_mat(i,1);
        Y(1,i) = pos_mat(i,2);
        Z(1,i) = pos_mat(i,3);
    end
    assignin('base','base_plot_x',X);
    assignin('base','base_plot_y',Y);
    assignin('base','base_plot_z',Z);

    hold on
    plot_line = plot3(X,Y,Z);
%     plot_dot = plot3(X,Y,Z,'.');
    assignin('base','base_plot_line',plot_line);
%     assignin('base','base_plot_dot',plot_dot);

    base_plot_line.XDataSource = 'base_plot_x'
    base_plot_line.YDataSource = 'base_plot_y'
    base_plot_line.ZDataSource = 'base_plot_z'
    %             hold off
end



















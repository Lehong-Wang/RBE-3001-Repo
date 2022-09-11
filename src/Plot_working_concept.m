

function [plot_pos, make_plot] = Plot_working_concept
    plot_pos = @plot_pos
    make_plot = @make_plot
end
    

function make_plot
%     X = [1 2];
%     Y = [3 4];
%     Z = [5 6];
    X = zeros(1,2);
    Y = zeros(1,2);
    Z = zeros(1,2);
    assignin('base','base_plot_x',X);
    assignin('base','base_plot_y',Y);
    assignin('base','base_plot_z',Z);

    plot_line = plot3(X,Y,Z);
    axis([0 10 0 10 0 10])
    
    plot_line.XDataSource = 'base_plot_x';
    plot_line.YDataSource = 'base_plot_y';
    plot_line.XDataSource = 'base_plot_z';
    
    assignin('base','base_plot_line',plot_line);
end


function plot_pos(x,y,z)

    assignin('base','base_plot_x',x);
    assignin('base','base_plot_y',y);
    assignin('base','base_plot_z',z);
    refreshdata
%     drawnow
end


% 
% function 
% robot = Robot(dev)

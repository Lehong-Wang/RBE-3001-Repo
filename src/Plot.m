

function [make_plot, plot_pos, plot_arm] = Plot
    make_plot = @make_plot
    plot_pos = @plot_pos
    plot_arm = @plot_arm
end


function make_plot
    X = zeros(1,2);
    Y = zeros(1,2);
    Z = zeros(1,2);
    assignin('base','base_plot_x',X);
    assignin('base','base_plot_y',Y);
    assignin('base','base_plot_z',Z);

    hold on
    plot_line = plot3(X,Y,Z);
    plot_dot = plot3(X,Y,Z,'.');
    axis([-100 200 -200 200 0 300]);
    view(-30,45);
    grid on
    
    plot_line.XDataSource = 'base_plot_x';
    plot_line.YDataSource = 'base_plot_y';
    plot_line.ZDataSource = 'base_plot_z';

    plot_dot.XDataSource = 'base_plot_x';
    plot_dot.YDataSource = 'base_plot_y';
    plot_dot.ZDataSource = 'base_plot_z';

    assignin('base','base_plot_line',plot_line);
end


% input the xyz positon of joints
% expect a n*3 matrix, n should be 5 (5 frames)
function plot_pos(joint_pos)
    X = zeros(1);
    Y = zeros(1);
    Z = zeros(1);
    for i = (1:height(joint_pos))
        X(1,i) = joint_pos(i,1);
        Y(1,i) = joint_pos(i,2);
        Z(1,i) = joint_pos(i,3);
    end

    assignin("base", 'base_plot_x', X);
    assignin("base", 'base_plot_y', Y);
    assignin("base", 'base_plot_z', Z);
    refreshdata
    drawnow
    

end



% input the transformation matrix of each joint respect to base
% expect a 4*4*n matrix, n should be 4 (4 joints)

function plot_arm(trans_mat)
    joint_pos = zeros(1,3);
    % for each transfromation matrix
    mat_size = size(trans_mat);
    for i = (1:mat_size(3))
        mat = trans_mat(:,:,i);
        pos_col = mat(1:3,4);
        pos = pos_col.';
        % first pos is base
        joint_pos(i,:) = pos;
    end
    disp(joint_pos);

    plot_pos(joint_pos);



end




















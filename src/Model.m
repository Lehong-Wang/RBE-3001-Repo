

function [make_plot, plot_pos, plot_axis, plot_arm] = Model
    make_plot = @make_plot
    plot_pos = @plot_pos
    plot_axis = @plot_axis
    plot_arm = @plot_arm
end


function make_plot
    X = zeros(1,2);
    Y = zeros(1,2);
    Z = zeros(1,2);
    U_x = zeros(1,2);
    V_x = zeros(1,2);
    W_x = zeros(1,2);
    U_y = zeros(1,2);
    V_y = zeros(1,2);
    W_y = zeros(1,2);
    U_z = zeros(1,2);
    V_z = zeros(1,2);
    W_z = zeros(1,2);

    assignin('base','base_plot_x',X);
    assignin('base','base_plot_y',Y);
    assignin('base','base_plot_z',Z);

    assignin('base','base_plot_ux',U_x);
    assignin('base','base_plot_vx',V_x);
    assignin('base','base_plot_wx',W_x);

    assignin('base','base_plot_uy',U_y);
    assignin('base','base_plot_vy',V_y);
    assignin('base','base_plot_wy',W_y);

    assignin('base','base_plot_uz',U_z);
    assignin('base','base_plot_vz',V_z);
    assignin('base','base_plot_wz',W_z);

    hold on
    plot_line = plot3(X,Y,Z,'MarkerSize', 30);
    plot_dot = plot3(X,Y,Z,'.');
    plot_axis_x = quiver3(X,Y,Z,U_x,V_x,W_x,.2);
    plot_axis_y = quiver3(X,Y,Z,U_y,V_y,W_y,.2);
    plot_axis_z = quiver3(X,Y,Z,U_z,V_z,W_z,.2);

%     plot_frame_0 = 
    axis([-100 200 -200 200 0 300]);
    view(-30,45);
    grid on
    axis on
    xlabel("--- X axis -->");
    ylabel("--- Y axis -->");
    zlabel("--- Z axis -->");

    
    plot_line.XDataSource = 'base_plot_x';
    plot_line.YDataSource = 'base_plot_y';
    plot_line.ZDataSource = 'base_plot_z';

    plot_dot.XDataSource = 'base_plot_x';
    plot_dot.YDataSource = 'base_plot_y';
    plot_dot.ZDataSource = 'base_plot_z';

    plot_axis_x.XDataSource = 'base_plot_x';
    plot_axis_x.YDataSource = 'base_plot_y';
    plot_axis_x.ZDataSource = 'base_plot_z';
    plot_axis_x.UDataSource = 'base_plot_ux'
    plot_axis_x.VDataSource = 'base_plot_vx';
    plot_axis_x.WDataSource = 'base_plot_wx';

    plot_axis_y.XDataSource = 'base_plot_x';
    plot_axis_y.YDataSource = 'base_plot_y';
    plot_axis_y.ZDataSource = 'base_plot_z';
    plot_axis_y.UDataSource = 'base_plot_uy'
    plot_axis_y.VDataSource = 'base_plot_vy';
    plot_axis_y.WDataSource = 'base_plot_wy';

    plot_axis_z.XDataSource = 'base_plot_x';
    plot_axis_z.YDataSource = 'base_plot_y';
    plot_axis_z.ZDataSource = 'base_plot_z';
    plot_axis_z.UDataSource = 'base_plot_uz'
    plot_axis_z.VDataSource = 'base_plot_vz';
    plot_axis_z.WDataSource = 'base_plot_wz';
%     assignin('base','base_plot_line',plot_line);

    plot_workspace();

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

% input rotation matrix for each joint
% expect a 3*3*n matrix (n = 5)
function plot_axis(joint_rot)
    U_x = zeros(1);
    V_x = zeros(1);
    W_x = zeros(1);

    U_y = zeros(1);
    V_y = zeros(1);
    W_y = zeros(1);

    U_z = zeros(1);
    V_z = zeros(1);
    W_z = zeros(1);

    x_axis = [1; 0; 0];
    y_axis = [0; 1; 0];
    z_axis = [0; 0; 1];

    mat_size = size(joint_rot);

    for i = (1:mat_size(3))
        rot_mat = joint_rot(:,:,i);
        rot_x_axis = rot_mat * x_axis;
        rot_y_axis = rot_mat * y_axis;
        rot_z_axis = rot_mat * z_axis;

        U_x(1,i) = rot_x_axis(1);
        V_x(1,i) = rot_x_axis(2);
        W_x(1,i) = rot_x_axis(3);

        U_y(1,i) = rot_y_axis(1);
        V_y(1,i) = rot_y_axis(2);
        W_y(1,i) = rot_y_axis(3);

        U_z(1,i) = rot_z_axis(1);
        V_z(1,i) = rot_z_axis(2);
        W_z(1,i) = rot_z_axis(3);
    end
    

    assignin('base','base_plot_ux',U_x);
    assignin('base','base_plot_vx',V_x);
    assignin('base','base_plot_wx',W_x);

    assignin('base','base_plot_uy',U_y);
    assignin('base','base_plot_vy',V_y);
    assignin('base','base_plot_wy',W_y);

    assignin('base','base_plot_uz',U_z);
    assignin('base','base_plot_vz',V_z);
    assignin('base','base_plot_wz',W_z);

    refreshdata
    drawnow
end



% input the transformation matrix of each joint respect to base
% expect a 4*4*n matrix, n should be 4 (4 joints)

function plot_arm(trans_mat)
    joint_pos = zeros(1,3);
    joint_rot = zeros(3,3);
    % for each transfromation matrix
    mat_size = size(trans_mat);
    for i = (1:mat_size(3))
        mat = trans_mat(:,:,i);
        pos_col = mat(1:3,4);
        pos = pos_col.';
        % first pos is base
        joint_pos(i,:) = pos;

        rot_mat = mat(1:3,1:3);
        joint_rot(:,:,i) = rot_mat;

    end
%     disp(joint_pos);
%     disp(joint_rot);

    plot_pos(joint_pos);
    plot_axis(joint_rot);



end


function plot_workspace()

    [x,y,z] = sphere;      %# Makes a 21-by-21 point sphere
    x = x(5:end,:);       %# Keep top 11 x points
    y = y(5:end,:);       %# Keep top 11 y points
    z = z(5:end,:);       %# Keep top 11 z points
    r = 200;                 %# A radius value
    surf(r.*x,r.*y,r.*z+95);  %# Plot the surface
%     axis equal;            %# Make the scaling on the x, y, and z axes equal
end




















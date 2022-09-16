

clc
clear

check_in_work_space(-20,40,6)





function in_work_space = check_in_work_space(x, y, z)

    if z<0
        in_work_space = false;
        error("ERROR: Z axis less than 0");
    end
    % outer sphere formed by arm length
    if (x^2 + y^2 + (z-95)^2) > 200^2
        in_work_space = false;
        error('ERROR: NOT in work space! \nArm not long enough');
    end
    % 
    if (z >= 0 && z <= 95)
        if x<0
            in_work_space = false;
            error("ERROR: NOT in work space! \nJoint 1 limitation [90, -90] (bottom)");
        end
        if x^2+y^2 < 50^2
            in_work_space = false;
            error("ERROR: NOT in work space! \nMay hit base joint");
        end
        in_work_space = true;
        return
    end
    % middle part, limited by joint 2 -45 deg range
    if (z > 95 && z <= 95+100*2^0.5)
        if x<0
            in_work_space = false;
            error("ERROR: NOT in work space! \nJoint 1 limitation [90, -90] (middle)");
        end
        if sqrt(x^2+y^2) < sqrt(100^2-(z-195)^2)-50*2^0.5
            in_work_space = false;
            error("ERROR: NOT in work space! \nJoint 2 limitation [90, -45] (middle)");
        end
        in_work_space = true;
        return
    end

    if (z > 95+100*2^0.5 && z <= 195+50*2^0.5)
        disp(sqrt(x^2+y^2))
        if x<0 && sqrt(x^2+y^2) > 50*2^0.5-sqrt(100^2-(z-195)^2) && sqrt(x^2+y^2) < 50*2^0.5+sqrt(100^2-(z-195)^2)
           in_work_space = false;
            error("ERROR: NOT in work space! \nJoint 2 limitation [90, -45] (top)");
        end 
        in_work_space = true;
        return
    end

    if (z > 195+50*2^0.5)
        in_work_space = true;
        return
    end





end



function q = inverseKinematics(T)
%inverseKinematics calculates the inverse kinematics of reRACKer for a
%given end effector position
%
%q = inverseKinematics(T) where q is output row vector of joint variable
%values in the form q = [Ɵ1*, d2*, d3*], and T is input homogenous
%transformation matrix in the form  T   =   nx sx ax dx
%                                           ny sy ay dy
%                                           nz sz az dz
%                                           0  0  0  1
%
%developed by Riley Black
    
    %extracting relevant information into variables for readability
    dx = T(1,4);
    dy = T(2,4);
    dz = T(3,4);

    %assigning manipulator constants
    L1 = 15;
    
    %calculating d2
    d2 = sqrt((dx*dx)+(dy*dy));
    
    %calculating d3
    d3 = dz - L1;
    
    %calculating t1
    s1 = -dx/d2;
    c1 = dy/d2;
    t1 = atan2d(s1, c1);
    
    %constructing q row vector
    q = [t1, d2, d3];
end
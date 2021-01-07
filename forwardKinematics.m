function [T01, T02, T03] = forwardKinematics(q)
%forwardKinematics calculates the intermediate and final forward kinematics 
%matricies of reRACKer in a given configuration
%
%[T01, T02, T03] = forwardKinematics(q) where q is the input row vector of 
%joint variables in the form q = [Ɵ1*, d2*, d3*], and T01, T02, and T03 are
%the output homogenous transformation matricies (HTMs) relating joint 1, 2,
%and 3 to the base frame, respectivly, taking the following form:
%T  =   nx sx ax dx
%       ny sy ay dy
%       nz sz az dz
%       0  0  0  1
%
%developed by Riley Black

    %extracting relevant information into variables for readability
    t1 = q(1);
    d2 = q(2);
    d3 = q(3);
    
    %assigning manipulator constants
    L1 = 15;
    
    %calculating T01
    A1 =   [cosd(t1), 0, -sind(t1), 0;
            sind(t1), 0, cosd(t1), 0;
            0, -1, 0, L1;
            0, 0, 0, 1];
    T01 = A1;
        
    %calculating T02
    A2 =   [1, 0, 0, 0;
            0, 0, -1, 0;
            0, 1, 0, d2;
            0, 0, 0, 1];
    T02 = A1*A2;
    
    %calculating T03
    A3 =   [1, 0, 0, 0;
            0, 1, 0, 0;
            0, 0, 1, d3;
            0, 0, 0, 1];
    T03 = T02*A3;
end

function [J01, J02, J03] = jacobian(q)
%jacobian calculates the intermediate and final jacobian matrices of
%reRACKer in a given configuration
%
%[J01, J02, J03] = jacobian(q) where q is the input row vector of joint 
%variables in the form q = [Ɵ1*, d2*, d3*], and J01, J02, and J03 are the
%output jacobian matricies for the origins of frame 1, 2, and 3,
%respectively.
%
%developed by Riley Black
    
    %extracting relevant information into variables for readability
    t1 = q(1);
    d2 = q(2);
%     d3 = q(3);
    
    %constructing empty containers to hold jacobians 
    J01 = zeros(3);
    J02 = zeros(3);
    J03 = zeros(3);
    
    %calculating J01
    %all zeros
    
    %calculating J02
    J02(1,1) = -d2*cosd(t1);
    J02(2,1) = -d2*sind(t1);
    J02(1,2) = -sind(t1);
    J02(2,2) = cosd(t1);
    
    %calculating J03
    J03(1,1) = -d2*cosd(t1);
    J03(2,1) = -d2*sind(t1);
    J03(1,2) = -sind(t1);
    J03(2,2) = cosd(t1);
    J03(3,3) = 1;
end
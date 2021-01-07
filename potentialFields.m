function T = potentialFields(qs, qf, cueballPos, chalkPos, rackPos)
%potentialFields calculates the total configuration space joint torque
%acting on reRACKer in a given scenario
%
%T = potentialFields(qs, qf, cueballPos, chalkPos, rackPos) where T
%is the outputted total configuration space joint torque in the form
%T = [T1, T2, T3]', qs is the inputted current manipulator configuration,
%qf is the inputted final manipulator configuration, cueballPos is the
%inputted cueball obstacle centroid position, chalkPos is the inputted
%chalk obstacle centroid position, and rackPos is the inputted rack
%obstacle centroid position. These manipulator configurations are 
%expressed in the form q = [Ɵ1*, d2*, d3*], and the positions are expressed
%in the form Pos = [x, y, z] as coordinates relative to robot base.
%
%developed by Riley Black

    %calculating intermediate HTMs of current/final configurations
    [Ts01, Ts02, Ts03] = forwardKinematics(qs);                             %current position
    [Tf01, Tf02, Tf03] = forwardKinematics(qf);                             %final position
    
    %extracting relevant information into variables for readability
    O1s = Ts01(1:3, 4);                                                     %origin 1 at current configuration
    O2s = Ts02(1:3, 4);                                                     %origin 2 at current configuration
    O3s = Ts03(1:3, 4);                                                     %origin 3 at current configuration
    O1f = Tf01(1:3, 4);                                                     %origin 1 at final configuration
    O2f = Tf02(1:3, 4);                                                     %origin 2 at final configuration
    O3f = Tf03(1:3, 4);                                                     %origin 3 at final configuration
    
    %calculating attractive forces
    [Fatt1, Fatt2, Fatt3] = attractiveForces(O1s, O2s, O3s, O1f, O2f, O3f);
    
    %calculating repulsive forces
    [Frep1, Frep2, Frep3] = repulsiveForces(O1s, O2s, O3s, cueballPos, chalkPos, rackPos);

    %mapping workspace forces
    T = forceMapping(qs, Fatt1, Fatt2, Fatt3, Frep1, Frep2, Frep3);
end

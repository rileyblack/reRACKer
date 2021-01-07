function T = forceMapping(qs, Fatt1, Fatt2, Fatt3, Frep1, Frep2, Frep3)
%forceMapping maps workspace forces to configuration space forces for
%reRACKer in a given scenario
%
%T = forceMapping(qs, Fatt1, Fatt2, Fatt3, Frep1, Frep2, Frep3) where T is
%the outputted total configuration space joint torque produced by potential
%fields, qs is the inputted current configuration in the form 
%qs =[Ɵ1*, d2*, d3*], Fatt1, Fatt2, and Fatt3 are the inputted attractive
%forces acting on joints 1, 2, and 3, respectively, and Frep1, Frep2, and
%Frep3 are the inputted repulsive forces acting on joints 1, 2, and 3,
%respectively.
%
%developed by Riley Black

    %calculating intermediate/final jacobians of current configuration
    [J01, J02, J03] = jacobian(qs);

    %calculating configuration space torques of current configuration
    Tatt1 = (transpose(J01))*Fatt1;                                         %origin 1 attractive torque
    Tatt2 = (transpose(J02))*Fatt2;                                         %origin 2 attractive torque
    Tatt3 = (transpose(J03))*Fatt3;                                         %origin 3 attractive torque
    Trep1 = (transpose(J01))*Frep1;                                         %origin 1 repulsive torque
    Trep2 = (transpose(J02))*Frep2;                                         %origin 2 repulsive torque
    Trep3 = (transpose(J03))*Frep3;                                         %origin 3 repulsive torque

    %calculating total configuration space joint torque at current position
    T = Tatt1 + Tatt2 + Tatt3 + Trep1 + Trep2 + Trep3;
end
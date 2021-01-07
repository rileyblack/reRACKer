function [Fatt1, Fatt2, Fatt3] = attractiveForces(O1s, O2s, O3s, O1f, O2f, O3f)
%attractiveForces calculates the attractive forces acting at each joint of
%reRACKer in a given scenario
%
%[Fatt1, Fatt2, Fatt3] = attractiveForces(O1s, O2s, O3s, O1f, O2f, O3f)
%where Fatt1, Fatt2, and Fatt3 are the outputted attractive forces acting
%on joints 1, 2, and 3, respectively, O1s, O2s, and O3s are the inputted
%current positions of origin 1, 2, and 3, respectfully, and O1f, O2f, and 
%O3f are the inputted final positions of origin 1, 2, and 3, respectfully.
%All above positions are expressed as coordinates relative to robot base in
%the form Pos = [x, y, z], and that this function implements the hybrid
%attractive field.
%
%developed by Riley Black

    %defining attractive force parameters
    Y = [1, 1, 1];                                                          %zetas [origin 1, origin 2, origin 3]
    d = 10;                                                                 %hybrid transition distance

    %calculating parabolic attractive forces by default for each joint 
    Fatt1 = -Y(1)*(O1s - O1f);
    Fatt2 = -Y(2)*(O2s - O2f);
    Fatt3 = -Y(3)*(O3s - O3f);
    
    %determining origin distances from current position to final position
    d1 = norm(O1s - O1f);
    d2 = norm(O2s - O2f);
    d3 = norm(O3s - O3f);
    
    %determining if parabolic or conic forces will be used for joint 1
    if(d1 > d)                                                              %checking if farther than hybrid transition distance  
        Fatt1 = (d*Fatt1)/d1;                                               %calculating conic attractive forces by modifying parabolic force
    end
    
    %determining if parabolic or conic forces will be used for joint 2
    if(d2 > d)                                                              %checking if farther than hybrid transition distance 
        Fatt2 = (d*Fatt2)/d2;                                               %calculating conic attractive forces by modifying parabolic force
    end
    
    %determining if parabolic or conic forces will be used for joint 3
    if(d3 > d)                                                              %checking if farther than hybrid transition distance 
        Fatt3 = (d*Fatt3)/d3;                                               %calculating conic attractive forces by modifying parabolic force
    end
end
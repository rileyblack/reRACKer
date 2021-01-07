function [Frep1, Frep2, Frep3] = repulsiveForces(O1s, O2s, O3s, cueballPos, chalkPos, rackPos)
%repulsiveForces calculates the repulsive forces acting at each joint of
%reRACKer in a given scenario
%
%[Frep1, Frep2, Frep3] = repulsiveForces(O1s, O2s, O3s, cueballPos, chalkPos, rackPos)
%where Frep1, Frep2, and Frep3 are the outputted repulsive forces acting
%on joints 1, 2, and 3, respectively, O1s, O2s, and O3s are the inputted
%current positions of origin 1, 2, and 3, respectfully, cueballPos is the
%inputted cueball obstacle centroid position, chalkPos is the inputted
%chalk obstacle centroid position, and rackPos is the inputted rack
%obstacle centroid position. Note all above positions are expressed as
%coordinates relative to robot base in the form Pos = [x, y, z].
%
%developed by Riley Black
    
    %defining spherically approximated obstacles' radius
    radii = [1.125, 3.5, 6.75];                                             %[cueball, chalk box, triangle rack]
    
    %defining repulsive force parameters
    N = [1000, 1000, 1000];                                                 %etas [origin 1, origin 2, origin 3]
    p0 = [1.125, 3.5, 6.75];                                                %radii of influence [cueball, chalk box, triangle rack]

    %converting current origin positions to iteratable form
    Os = [O1s, O2s, O3s];
    
    %constructing empty container to hold Frep = [Frep1, Frep2, Frep3]
    Frep = zeros(3);
    
    %calculating repulsive force on each origin in current configuration
    for i = 1:3                                                             %calculating distance from current origin to each obstacle sphere
        pcueball = norm(Os(1:3, i) - cueballPos') - radii(1);               %cueball distance
        pchalk = norm(Os(1:3, i) - chalkPos') - radii(2);                   %chalk distance
        prack = norm(Os(1:3, i) - rackPos') - radii(3);                     %rack distance

        %comparing distances to obstacles' region of convergence
        if (((pcueball <= p0(1))&&(pchalk <= p0(2)))||((pcueball <= p0(1))&&(prack <= p0(3)))||((prack <= p0(3))&&(pchalk <= p0(2))))
            disp('Two or more obstacles have an overlapping region of convergence.')
        elseif(pcueball <= p0(1))                                           %checking if origin within cueball ROI 
            Frep(1:3, i) = N(i)*((1/pcueball)-(1/p0(1)))*(1/(pcueball*pcueball))*((Os(1:3, i)-cueballPos')/pcueball);
        elseif(pchalk <= p0(2))                                             %checking if origin within chalk box ROI 
            Frep(1:3, i) = N(i)*((1/pchalk)-(1/p0(2)))*(1/(pchalk*pchalk))*((Os(1:3, i)-chalkPos')/pchalk);
        elseif(prack <= p0(3))                                              %checking if origin within rack ROI 
            Frep(1:3, i) = N(i)*((1/prack)-(1/p0(3)))*(1/(prack*prack))*((Os(1:3, i)-rackPos')/prack);
        end
    end
    
    %extracting result into return values
    Frep1 = Frep(1:3, 1);
    Frep2 = Frep(1:3, 2);
    Frep3 = Frep(1:3, 3);
end
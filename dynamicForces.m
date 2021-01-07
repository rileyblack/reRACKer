function T = dynamicForces(q, q_O, q_OO)
%dynamicForces calculates the torques acting at the joints of reRACKer
%given joint positions, velocities, and accelerations by evaluating the
%dynamic equations of motion derived using the Newton-Euler method

%T = dynamicForces(q, q_O, q_OO) where q is an input 3x1 column vector of
%joint positions in the form [T1, D2, D3]', q_O is an input 3x1 column
%vector of joint velocities in the form [T1_O, D2_O, D3_O]', q_OO is an
%input 3x1 column vector of joint accelerations in the form [T1_OO, D2_OO,
%D3_OO]', and T is an output 3x1 column vector of joint torques required to
%produce the requested motion in the form [t1, t2, t3]'
%
%developed by Riley Black


    %----------------------------------------------------------------------
    %extracting joint inputs for readability
    %----------------------------------------------------------------------

    %extracting joint positions
    T1 = q(1);                                                              %revolute joint 1 position
    D2 = q(2);                                                              %prismatic joint 2 position
    D3 = q(3);                                                              %prismatic joint 3 position

    %extracting joint velocities
    T1_O = q_O(1);                                                          %revolute joint 1 velocity
    D2_O = q_O(2);                                                          %prismatic joint 2 velocity
    D3_O = q_O(3);                                                          %prismatic joint 3 velocity

    %extracting joint accelerations
    T1_OO = q_OO(1);                                                        %revolute joint 1 acceleration
    D2_OO = q_OO(2);                                                        %prismatic joint 2 acceleration
    D3_OO = q_OO(3);                                                        %prismatic joint 3 acceleration


    %----------------------------------------------------------------------
    %defining manipulator physical parameters
    %----------------------------------------------------------------------

    %defining physical manipulator link lengths of SolidWorks Model
    L1 = 55;                                                                %length of link 1 in inches
    L2 = 65;                                                                %length of link 2 in inches
    L3 = 32;                                                                %length of link 3 in inches

    %defining physical manipulator link masses of SolidWorks Model
    m1 = 69.05;                                                             %mass of link 1 in pounds
    m2 = 51.33;                                                             %mass of link 2 in pounds
    m3 = 18.73;                                                             %mass of link 3 in pounds

    %defining physical manipulator link CoMs of SolidWorks Model
    swCoM1 = [0,  24.86, 0];                                                %SolidWorks link 1 CoM WRT part [x, y, z] frame
    swCoM2 = [33.51, 0, 0];                                                 %SolidWorks link 2 CoM WRT part [x, y, z] frame
    swCoM3 = [0, 16, 0];                                                    %SolidWorks link 3 CoM WRT part [x, y, z] frame

    %defining physical manipulator link inertias of SolidWorks Model
    swI1 = [24407.21, 1976.07, 23918.19];                                   %SolidWorks link 1 inertia WRT assembly [x, y, z] frame
    swI2 = [848.56,  21959.25, 21658.21];                                   %SolidWorks link 2 inertia WRT assembly [x, y, z] frame
    swI3 = [2028.91, 164.39, 1976.88];                                      %SolidWorks link 3 inertia WRT assembly [x, y, z] frame

    %defining physical manipulator link inertia matrices (converted from SolidWorks frames to DH frames)
    I1 = [swI1(3), 0, 0; 0, swI1(2), 0; 0, 0, swI1(1)];                     %inertia matrix of link 1 WRT DH frame 1 (with frame 1 translated to link 1 CoM) 
    I2 = [swI2(3), 0, 0; 0, swI2(1), 0; 0, 0, swI2(2)];                     %inertia matrix of link 2 WRT DH frame 2 (with frame 2 translated to link 2 CoM)
    I3 = [swI3(3), 0, 0; 0, swI3(1), 0; 0, 0, swI3(2)];                     %inertia matrix of link 3 WRT DH frame 3 (with frame 3 translated to link 3 CoM)

    %defining gravity in IPS units
    g = 386.09;


    %----------------------------------------------------------------------
    %constructing terms used in equations
    %----------------------------------------------------------------------

    %constructing inertia values
    I1yy = I1(2, 2);
    I2zz = I2(3, 3);
    I3zz = I3(3, 3);

    %constructing length scalars to link i CoM WRT joint i+1
    Lc1 = L1 - swCoM1(2);                                                   %length from joint 2 to link 1 CoM
    Lc2 = L2 - swCoM2(1);                                                   %length from joint 3 to link 2 CoM
    Lc3 = L3 - swCoM3(2);                                                   %length from end effector to link 3 CoM


    %----------------------------------------------------------------------
    %calculating joint torques using dynamic equations
    %----------------------------------------------------------------------
    
    t3 = (m3*D3_OO) + (m3*g);
    t2 = ((m2+m3)*D2_OO)+(((Lc2*m2)-(L1*m2)-(L1*m3))*(T1_O*T1_O));
    t1 = (((L1*L1*m1)+(L1*L1*m2)+(L1*L1*m3)+(Lc1*Lc1*m1)-(2*L1*Lc1*m1)...
        -(L1*Lc1*m2)-(L1*Lc1*m3)-(L1*Lc2*m2)+(L1*Lc2*m3)+(Lc1*Lc2*m2)...
        +I1yy+I2zz+I3zz)*(T1_OO))+(((L1*m2)+(L1*m3)-(Lc1*m2)-(Lc1*m3)...
        +(Lc2*m3))*(2*D2_O*T1_O));

    %formatting return
    T = [double(t1), double(t2), double(t3)]';
end

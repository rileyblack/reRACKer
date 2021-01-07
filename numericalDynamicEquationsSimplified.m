%developed by Riley Black

%preparing MATLAB workspace
close all
clear all
clc

%--------------------------------------------------------------------------
%defining symbolic variables for derivation of dynamic equations
%--------------------------------------------------------------------------

%defining joint position variables
T1 = sym('T1', 'real');                                                     %revolute joint 1 position
D2 = sym('D2', 'real');                                                     %prismatic joint 2 position
D3 = sym('D3', 'real');                                                     %prismatic joint 3 position

%defining joint velocity variables
T1_O = sym('T1_O', 'real');                                                 %revolute joint 1 velocity
D2_O = sym('D2_O', 'real');                                                 %prismatic joint 2 velocity
D3_O = sym('D3_O', 'real');                                                 %prismatic joint 3 velocity

%defining joint acceleration variables
T1_OO = sym('T1_OO', 'real');                                               %revolute joint 1 acceleration
D2_OO = sym('D2_OO', 'real');                                               %prismatic joint 2 acceleration
D3_OO = sym('D3_OO', 'real');                                               %prismatic joint 3 acceleration


%--------------------------------------------------------------------------
%defining numeric values of manipulator physical parameters
%--------------------------------------------------------------------------

%defining physical manipulator link lengths in SolidWorks Model
L1 = 55;                                                                    %length of link 1 in inches
L2 = 65;                                                                    %length of link 2 in inches
L3 = 32;                                                                    %length of link 3 in inches

%defining physical manipulator link masses in SolidWorks Model
m1 = 69.05;                                                                 %mass of link 1 in pounds
m2 = 51.33;                                                                 %mass of link 2 in pounds
m3 = 18.73;                                                                 %mass of link 3 in pounds

%defining physical manipulator link CoMs in SolidWorks Model
swCoM1 = [0,  24.86, 0];                                                    %SolidWorks link 1 CoM WRT part [x, y, z] frame
swCoM2 = [33.51, 0, 0];                                                     %SolidWorks link 2 CoM WRT part [x, y, z] frame
swCoM3 = [0, 16, 0];                                                        %SolidWorks link 3 CoM WRT part [x, y, z] frame

%defining physical manipulator link inertias in SolidWorks Model
swI1 = [24407.21, 1976.07, 23918.19];                                       %SolidWorks link 1 inertia WRT assembly [x, y, z] frame
swI2 = [848.56,  21959.25, 21658.21];                                       %SolidWorks link 2 inertia WRT assembly [x, y, z] frame
swI3 = [2028.91, 164.39, 1976.88];                                          %SolidWorks link 3 inertia WRT assembly [x, y, z] frame

%defining physical manipulator link inertia matrices (converted from SolidWorks frames to DH frames)
I1 = [swI1(3), 0, 0; 0, swI1(2), 0; 0, 0, swI1(1)];                         %inertia matrix of link 1 WRT DH frame 1 (with frame 1 translated to link 1 CoM) 
I2 = [swI2(3), 0, 0; 0, swI2(1), 0; 0, 0, swI2(2)];                         %inertia matrix of link 2 WRT DH frame 2 (with frame 2 translated to link 2 CoM)
I3 = [swI3(3), 0, 0; 0, swI3(1), 0; 0, 0, swI3(2)];                         %inertia matrix of link 3 WRT DH frame 3 (with frame 3 translated to link 3 CoM)

%defining gravity in IPS units
g = 386.09;


%--------------------------------------------------------------------------
%constructing terms used in equations
%--------------------------------------------------------------------------

%constructing symbolic joint vectors
q = [T1, D2, D3];                                                           %joint position vector
q_O = [T1_O, D2_O, D3_O];                                                   %joint velocity vector
q_OO = [T1_OO, D2_OO, D3_OO];                                               %joint acceleration vector

%constructing inertia values that are actually used in dynamic equations
I1yy = I1(2, 2);
I2zz = I2(3, 3);
I3zz = I3(3, 3);

%constructing length scalars to link i CoM WRT joint i+1
Lc1 = L1 - swCoM1(2);                                                       %length from joint 2 to link 1 CoM
Lc2 = L2 - swCoM2(1);                                                       %length from joint 3 to link 2 CoM
Lc3 = L3 - swCoM3(2);                                                       %length from end effector to link 3 CoM


%--------------------------------------------------------------------------
%calculating dynamic equations
%--------------------------------------------------------------------------

%original MATLAB long form
Tt3 = m3*(D3_OO + g);
Tt2 = m2*(D2_OO - L1*T1_O^2 + Lc2*T1_O^2) + m3*(- L1*T1_O^2 + D2_OO);
Tt1 = (L1 - Lc1)*(m2*(2*D2_O*T1_O + L1*T1_OO - Lc2*T1_OO)...
    + m3*(2*D2_O*T1_O + L1*T1_OO) + T1_OO*m1*(L1 - Lc1)) + I1yy*T1_OO ...
    + I2zz*T1_OO + I3zz*T1_OO + Lc2*m3*(2*D2_O*T1_O + L1*T1_OO);

%rearranged form T = D(q)q_OO + C(q, q_O)q_O + g
Tt3s = (m3*D3_OO) + (m3*g);
Tt2s = ((m2+m3)*D2_OO)+(((Lc2*m2)-(L1*m2)-(L1*m3))*(T1_O*T1_O));
Tt1s = (((L1*L1*m1)+(L1*L1*m2)+(L1*L1*m3)+(Lc1*Lc1*m1)-(2*L1*Lc1*m1)...
    -(L1*Lc1*m2)-(L1*Lc1*m3)-(L1*Lc2*m2)+(L1*Lc2*m3)+(Lc1*Lc2*m2)+I1yy...
    +I2zz+I3zz)*(T1_OO))+(((L1*m2)+(L1*m3)-(Lc1*m2)-(Lc1*m3)+(Lc2*m3))...
    *(2*D2_O*T1_O));

%converting to rational constants instead of fraction constants
Tt3 = vpa(Tt3,5);
Tt2 = vpa(Tt2,5);
Tt1 = vpa(Tt1,5);
Tt3s = vpa(Tt3s,5);
Tt2s = vpa(Tt2s,5);
Tt1s = vpa(Tt1s,5);
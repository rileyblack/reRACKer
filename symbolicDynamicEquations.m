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

%defining manipulator physical parameters
L1 = sym('L1', 'real');                                                     %length of link 1 in inches
L2 = sym('L2', 'real');                                                     %length of link 2 in inches
L3 = sym('L3', 'real');                                                     %length of link 3 in inches
m1 = sym('m1', 'real');                                                     %mass of link 1 in pounds
m2 = sym('m2', 'real');                                                     %mass of link 2 in pounds
m3 = sym('m3', 'real');                                                     %mass of link 3 in pounds
I1xx = sym('I1xx', 'real');                                                 %inertia of link 1 WRT DH frame 1 x-axis (with frame 1 translated to link 1 CoM)
I1yy = sym('I1yy', 'real');                                                 %inertia of link 1 WRT DH frame 1 y-axis (with frame 1 translated to link 1 CoM)
I1zz = sym('I1zz', 'real');                                                 %inertia of link 1 WRT DH frame 1 z-axis (with frame 1 translated to link 1 CoM) 
I2xx = sym('I2xx', 'real');                                                 %inertia of link 2 WRT DH frame 2 x-axis (with frame 2 translated to link 2 CoM)
I2yy = sym('I2yy', 'real');                                                 %inertia of link 2 WRT DH frame 2 y-axis (with frame 2 translated to link 2 CoM)
I2zz = sym('I2zz', 'real');                                                 %inertia of link 2 WRT DH frame 2 z-axis (with frame 2 translated to link 2 CoM)
I3xx = sym('I3xx', 'real');                                                 %inertia of link 2 WRT DH frame 3 x-axis (with frame 3 translated to link 3 CoM)
I3yy = sym('I3yy', 'real');                                                 %inertia of link 2 WRT DH frame 3 y-axis (with frame 3 translated to link 3 CoM)
I3zz = sym('I3zz', 'real');                                                 %inertia of link 2 WRT DH frame 3 z-axis (with frame 3 translated to link 3 CoM)
Lc1 = sym('Lc1', 'real');                                                   %length from O1 to link 1 CoM
Lc2 = sym('Lc2', 'real');                                                   %length from O2 to link 2 CoM
Lc3 = sym('Lc3', 'real');                                                   %length from O3 to link 3 CoM
g = sym('g', 'real');                                                       %gravity 


%--------------------------------------------------------------------------
%constructing manipulator variables used in algorithm
%--------------------------------------------------------------------------

%constructing joint vectors
q = [T1, D2, D3];                                                           %joint position vector
q_O = [T1_O, D2_O, D3_O];                                                   %joint velocity vector
q_OO = [T1_OO, D2_OO, D3_OO];                                               %joint acceleration vector

%constructing position vectors of key manipulator locations
r1c1 = [0, 0, L1 - Lc1]';                                                   %distance vector to link 1 CoM WRT joint 1 (to link 1 CoM WRT DH frame 0)
r2c2 = [0, 0, D2 - Lc2]';                                                   %distance vector to link 2 CoM WRT joint 2 (to link 2 CoM WRT DH frame 1)
r3c3 = [0, 0, D3 - Lc3]';                                                   %distance vector to link 3 CoM WRT joint 3 (to link 3 CoM WRT DH frame 2)
r2c1 = [0, Lc1, 0]';                                                        %distance vector to link 1 CoM WRT joint 1 (to link 1 CoM WRT DH frame 1)
r3c2 = [0, -Lc2, 0]';                                                       %distance vector to link 2 CoM WRT joint 2 (to link 2 CoM WRT DH frame 2)
r4c3 = [0, 0, -Lc3]';                                                       %distance vector to link 3 CoM WRT joint 3 (to link 3 CoM WRT DH frame 3)
r12 = [0, 0, L1]';                                                          %distance vector to joint 2 WRT joint 1 (to DH frame 1 WRT DH frame 0)
r23 = [0, 0, D2]';                                                          %distance vector to joint 3 WRT joint 2 (to DH frame 2 WRT DH frame 1)
r34 = [0, 0, D3]';                                                          %distance vector to joint 4 WRT joint 3 (to DH frame 3 WRT DH frame 2)

%constructing inertia matrices of links
I1 = [I1xx, 0, 0;                                                           %link 1 inertia matrix
      0, I1yy, 0;
      0, 0, I1zz];
I2 = [I2xx, 0, 0;                                                           %link 2 inertia matrix
      0, I2yy, 0;
      0, 0, I2zz];
I3 = [I3xx, 0, 0;                                                           %link 3 inertia matrix
      0, I3yy, 0;
      0, 0, I3zz];

%constructing transformation matrices of frames
A1 = [cos(T1), 0, -sin(T1), 0;                                              %relating frame 0-->frame 1
      sin(T1), 0, cos(T1), 0;
      0, -1, 0, L1;
      0, 0, 0, 1];  
A2 = [1, 0, 0, 0;                                                           %relating frame 1-->frame 2
      0, 0, -1, 0;
      0, 1, 0, D2;
      0, 0, 0, 1];
A3 = [1, 0, 0, 0;                                                           %relating frame 2-->frame 3
      0, 1, 0, 0;
      0, 0, 1, D3;
      0, 0, 0, 1];
T01 = A1;                                                                   %relating frame 0-->frame 1
T02 = A1*A2;                                                                %relating frame 0-->frame 2
T03 = T02*A3;                                                               %relating frame 0-->frame 3

%constructing rotation matrices of frames  
R01 = A1(1:3, 1:3);                                                         %relating frame 0-->frame 1
R12 = A2(1:3, 1:3);                                                         %relating frame 1-->frame 2
R23 = A3(1:3, 1:3);                                                         %relating frame 2-->frame 3
R34 = zeros(3);                                                             %no fourth frame but rotation matrix definition still needed
% R01 = T01(1:3, 1:3);                                                      %relating frame 0-->frame 1
R02 = T02(1:3, 1:3);                                                        %relating frame 0-->frame 2
R03 = T03(1:3, 1:3);                                                        %relating frame 0-->frame 3

%constructing z0 vector by definition
z0 = [0, 0, 1]';

%constructing initial conditions by definition
w0 = [0, 0, 0]';                                                            %angular velocity of frame 0 WRT frame 0
alpha0 = [0, 0, 0]';                                                        %angular acceleration of frame 0 WRT frame 0
ac0 = [0, 0, 0]';                                                           %linear acceleration of link 0 CoM
ae0 = [0, 0, g]';                                                           %linear acceleration of link 0 end (joint 1)
                                                                            %NOTE: +z acceleration to account for gravity
                  
%constructing boundary conditions by definition
f4 = [0, 0, 0]';                                                            %force exerted on link 4 by link 3
T4 = [0, 0, 0]';                                                            %torque exerted on link 4 by link 3


%--------------------------------------------------------------------------
%calculating forward recursion
%--------------------------------------------------------------------------

%calculating link 1 values
w1 = ((R01')*w0)+((R01')*z0*q_O(1));                                        %angular velocity of frame 1 WRT frame 0
alpha1 = ((R01')*alpha0)+((R01')*z0*q_OO(1))+cross(w1, ((R01')*z0*q_O(1))); %angular acceleration of frame 1 WRT frame 0
ae1 = ((R01')*ae0)+cross(alpha1, r12)+cross(w1, cross(w1, r12));            %linear acceleration of link 1 CoM
ac1 = ((R01')*ae0)+cross(alpha1, r1c1)+cross(w1, cross(w1, r1c1));          %linear acceleration of link 1 end (joint 2)

%calculating link 2 values
w2 = (R12')*w1;                                                             %angular velocity of frame 2 WRT frame 0
alpha2 = (R12')*alpha1;                                                     %angular acceleration of frame 2 WRT frame 0
ae2 = ((R12')*(ae1+(q_OO(2)*z0)))+cross(2*q_O(2)*w2, (R12')*z0)...          %linear acceleration of link 2 CoM
    +cross(alpha2, r23)+cross(w2, cross(w2, r23)); 
ac2 = ae2+cross(alpha2, r3c2)+cross(w2, cross(w2, r3c2));                   %linear acceleration of link 2 end (joint 3)

%calculating link 3 values
w3 = (R23')*w2;                                                             %angular velocity of frame 3 WRT frame 0
alpha3 = (R23')*alpha2;                                                     %angular acceleration of frame 3 WRT frame 0
ae3 = ((R23')*(ae2+(q_OO(3)*z0)))+cross(2*q_O(3)*w3, (R23')*z0)...          %linear acceleration of link 3 CoM
    +cross(alpha3, r34)+cross(w3, cross(w3, r34)); 
ac3 = ae3+cross(alpha3, r4c3)+cross(w3, cross(w3, r4c3));                   %linear acceleration of link 3 end (end effector)


%--------------------------------------------------------------------------
%calculating backward recursion
%--------------------------------------------------------------------------

%calculating link 1 values and equation
f3 = (R34*f4)+(m3*ac3);                                                     %force exerted on link 3 by link 2 (gravity already accounted for)
T3 = (R34*T4)-cross(f3, r3c3)+cross((R34*f4), r4c3)+(I3*alpha3)...          %torque exerted on link 3 by link 2
    +cross(w3, I3*w3); 
Tt3 = (f3')*(R23')*z0;                                                      %actual torque acting at prismatic joint 3

%calculating link 2 values and equation
f2 = (R23*f3)+(m2*ac2);                                                     %force exerted on link 2 by link 1 (gravity already accounted for)
T2 = (R23*T3)-cross(f2, r2c2)+cross((R23*f3), r3c2)+(I2*alpha2)...          %torque exerted on link 2 by link 1
    +cross(w2, I2*w2); 
Tt2 = (f2')*(R12')*z0;                                                      %actual torque acting at prismatic joint 2

%calculating link 3 values and equation
f1 = (R12*f2)+(m1*ac1);                                                     %force exerted on link 1 by link 0 (gravity already accounted for)
T1 = (R12*T2)-cross(f1, r1c1)+cross((R12*f2), r2c1)+(I1*alpha1)...          %torque exerted on link 1 by link 0
    +cross(w1, I1*w1);
Tt1 = (T1')*(R01')*z0;                                                      %actual torque acting at rotary joint 1
%developed by Riley Black

%preparing MATLAB workspace
close all
clear all
clc

%--------------------------------------------------------------------------
%calculating path
%--------------------------------------------------------------------------

%defining gradient descent parameters
e = 1;                                                                      %region of convergence
a = 0.5;                                                                    %step size

%defining key x, y, z global positions relative to robot base
qsPos = [-1, 45, 74];                                                       %initial end-effector position
qfPos = [0, -30, 76.875];                                                   %final end-effector position
cueballPos = [0, -25, 76.875];                                              %cueball obstacle centroid
chalkPos = [-15, 30, 75.5];                                                 %chalk box obstacle centroid
rackPos = [-10, 0, 76];                                                     %racking triangle obstacle centroid

%defining arbitrary rotation matricies for start/final position
Rs = eye(4);                                                                %4x4 matrix with ones along diagonal and zeros elsewhere
Rf = eye(4);                                                                %4x4 matrix with ones along diagonal and zeros elsewhere

%constructing preliminary HTM for start/final position
Ts = Rs;                                                                    %start 4x4 rotation matrix
Ts(1:3, 4) = qsPos';                                                        %filling x, y, and z start position
Tf = Rf;                                                                    %final 4x4 rotation matrix
Tf(1:3, 4) = qfPos';                                                        %filling x, y, and z final position

%calculating start/final configurations using inverse kinematics
qs = inverseKinematics(Ts);                                                 %start manipulator configuration
qf = inverseKinematics(Tf);                                                 %final manipulator configuration

%implementing gradient descent algorithm
i = 1;                                                                      %iteration number
configpath(1:3, i) = qs';                                                   %adding starting configuration to configuration path
while (true)
    qcurr = configpath(1:3, i)';                                            %extracting current configuration from latest path entry
    [~, ~, T03] = forwardKinematics(qcurr);                                 %calculating current HTM from current configuration
    qcurrPos = T03(1:3, 4);                                                 %extracting current end effector position in current configuration
    endeffpath(1:3, i) = qcurrPos;                                          %adding current end effector position to end effector path
    distance = norm(qcurrPos - qfPos');                                     %calculating current configuration distance to goal configuration
    if(distance < e)                                                        %checking if within region of convergence
        break
    end
    T = potentialFields(qcurr, qf, cueballPos, chalkPos, rackPos);          %calculating joint torques at current configuration 
    Tdir = T/norm(T);                                                       %calculating normalized step direction
    configpath(1:3, i+1) = configpath(1:3, i) + (a*Tdir);                   %calculating next configuration and adding to path
    i = i + 1;
end


%--------------------------------------------------------------------------
%plotting scenario
%--------------------------------------------------------------------------

%defining useful plotting parameters
xmin = -50;                                                                 %minimum x value to be plotted based on proposed workspace 
xmax = 50;                                                                  %maximum x value to be plotted based on proposed workspace
ymin = -50;                                                                 %minimum y value to be plotted based on proposed workspace
ymax = 50;                                                                  %maximum y value to be plotted based on proposed workspace
zmin = 60;                                                                  %minimum z value to be plotted based on proposed workspace
zmax = 78;                                                                  %maximum z value to be plotted based on proposed workspace
p01 = 2.25;                                                                 %cueball radius of influence default (from origin)
p02 = 7;                                                                    %chalk radius of influence default (from origin)
p03 = 13.5;                                                                 %rack radius of influence default (from origin)
cueballsize = 1.125;                                                        %cueball physical radius (from origin)
chalksize = 3.5;                                                            %chalk physical radius (from origin)
racksize = 6.75;                                                            %rack physical radius (from origin)
textoffset = 3;                                                             %offset so point labels are not directly on point

%plotting end effector path
pathplot = scatter3(endeffpath(1, :), endeffpath(2, :), endeffpath(3, :), '.');
hold on
%plotting start point
startPoint = scatter3(qsPos(1), qsPos(2), qsPos(3), 500, 'g*');
text(qsPos(1)-textoffset, qsPos(2), qsPos(3)-2, 'Start', 'Fontsize', 20)
hold on
%plotting end point
scatter3(qfPos(1), qfPos(2), qfPos(3), 500, 'g*');
text(qfPos(1)+textoffset, qfPos(2), qfPos(3), 'Finish', 'Fontsize', 20)
hold on
%plotting cueball obstacle point
obstacle = scatter3(cueballPos(1), cueballPos(2), cueballPos(3), 200, 'r*');
text(cueballPos(1)-(3*textoffset), cueballPos(2)-(2*textoffset), cueballPos(3), 'Cueball', 'Fontsize', 15)
hold on
%plotting chalk obstacle point
scatter3(chalkPos(1), chalkPos(2), chalkPos(3), 200, 'r*');
text(chalkPos(1)-(7*textoffset), chalkPos(2)-(3*textoffset), chalkPos(3), 'Chalk', 'Fontsize', 15)
hold on
%plotting rack obstacle point
scatter3(rackPos(1), rackPos(2), rackPos(3), 200, 'r*');
text(rackPos(1)-(5*textoffset), rackPos(2)-(3*textoffset), rackPos(3), 'Rack', 'Fontsize', 15)
hold on
%plotting cueball obstacle radius of influence
[x1,y1,z1] = sphere;
x1 = x1*p01;
y1 = y1*p01;
z1 = z1*p01;
ROI = surf(x1+cueballPos(1),y1+cueballPos(2),z1+cueballPos(3), 'FaceColor', 'none','EdgeColor',uint8([255,192,203]));
hold on
%plotting chalk obstacle radius of influence
[x2,y2,z2] = sphere;
x2 = x2*p02;
y2 = y2*p02;
z2 = z2*p02;
surf(x2+chalkPos(1),y2+chalkPos(2),z2+chalkPos(3), 'FaceColor', 'none','EdgeColor',uint8([255,192,203]))
hold on
%plotting rack obstacle radius of influence
[x3,y3,z3] = sphere;
x3 = x3*p03;
y3 = y3*p03;
z3 = z3*p03;
surf(x3+rackPos(1),y3+rackPos(2),z3+rackPos(3), 'FaceColor', 'none','EdgeColor',uint8([255,192,203]))
hold on
%plotting cueball obstacle physical boundary
[x4,y4,z4] = sphere;
x4 = x4*cueballsize;
y4 = y4*cueballsize;
z4 = z4*cueballsize;
boundary = surf(x4+cueballPos(1),y4+cueballPos(2),z4+cueballPos(3), 'FaceColor', 'none','EdgeColor',uint8([255,150,150]));
hold on
%plotting chalk obstacle physical boundary
[x5,y5,z5] = sphere;
x5 = x5*chalksize;
y5 = y5*chalksize;
z5 = z5*chalksize;
surf(x5+chalkPos(1),y5+chalkPos(2),z5+chalkPos(3), 'FaceColor', 'none','EdgeColor',uint8([255,150,150]))
hold on
%plotting rack obstacle physical boundary
[x6,y6,z6] = sphere;
x6 = x6*racksize;
y6 = y6*racksize;
z6 = z6*racksize;
surf(x6+rackPos(1),y6+rackPos(2),z6+rackPos(3), 'FaceColor', 'none','EdgeColor',uint8([255,150,150]))
hold on
%plotting table as grey surface/plane at z = 78
[X,Y] = meshgrid(-25:1:25, -50:1:50);
Z = 78 + (X-X) + (Y-Y);
surf(X,Y,Z,'EdgeColor', 0.8*[1 1 1], 'FaceColor', 0.8*[1 1 1])
text(-20, -40, 77, 'Table', 'Fontsize', 15)
%formatting plot
xlim([xmin, xmax])
ylim([ymin, ymax])
zlim([zmin zmax])
%labelling axes
xlabel('x') 
ylabel('y')
zlabel('z')
%flipping z direction so table is at bottom
set(gca,'zdir','reverse')
%adding legend
legend([pathplot, startPoint, obstacle, boundary, ROI], {'Path', 'Start/End Point', 'Obstacle Centroid', 'Obstacle Boundary', 'Obstacle ROI'});
hold off
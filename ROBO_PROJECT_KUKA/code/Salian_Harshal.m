clc
clear all
close all

%Given D-H table
dh_cam = [0.34, 97.31*pi/180, 0, -pi/2;
      0, -4.70*pi/180, 0, pi/2;
      0.4, 162.57*pi/180, 0, pi/2;
      0, 103.53*pi/180, 0, -pi/2;
      0.4, -1.96*pi/180, 0, -pi/2;
      0 , -62.33*pi/180, 0, pi/2;
      0.126, 113.31*pi/180, 0 ,0];
  
%conversion of Joint angles from degrees to rad 
q(1) = deg2rad(97.31);
q(2) = deg2rad(-4.70);
q(3) = deg2rad(162.57);
q(4) = deg2rad(103.53);
q(5) = deg2rad(-1.96);
q(6) = deg2rad(-62.33);
q(7) = deg2rad(113.31);


%Final Transformation matrix from Base to End Effector of the robot
[t01_c,t02_c,t03_c,t04_c,t05_c,t06_c,t07_c,ja] = jacobian(dh_cam);
T0E = t07_c;

%d = 0.0431, theta = -90, a = 0.0662, alpha = 0
%transformation from end effector to the camera
TEC = [0 1 0 0; -1 0 0 -0.0662; 0 0 1 0.0431; 0 0 0 1];

% Transformation From camera to Aruco Maker
roll = deg2rad(174.1750404305652);
pitch = deg2rad(-17.3967534123935);
yaw = deg2rad(-1.9587388578232 );
RCA = eul2rotm([yaw, pitch, roll], "ZYX");
TC1 = [RCA; 0 0 0];
TC2 = [-0.205780720039398; -0.109793029482687; 0.561252115509121;1];
TCA = [TC1, TC2];

%transformation from Arucomarker to Target
TAT = [1 0 0 0.103975;
	0 1 0 -0.103975;
	0 0 1 0;
	0 0 0 1];

%Transformation from Base frame to Target
T0T = T0E * TEC * TCA * TAT;

%transformation matrix from end effector to obj
T_EObj = [0 -1 0 0.0455; -1 0 0 0; 0 0 -1 0.060; 0 0 0 1];

T_TObj = T0T * T_EObj;

R0E_last = T_TObj(1:3,1:3);
euler = rotm2eul(R0E_last,'zyz');
xpos = T_TObj(1,4);
ypos = T_TObj(2,4);
zpos = T_TObj(3,4);

%desired position
pd = [xpos; ypos; zpos];
phid = [euler(1); euler(2); euler(3)];

%inverse Kinematics
q1 = deg2rad([58.2686 75.3224 11.7968 45.9029 -22.1081 -31.2831 -42.3712]);

 steps = 10000;
 gain = 100;
 q = zeros(7, steps);
 e = zeros(6, steps);
 J = zeros(6,7);
 q(:,1) = q1';
 for i = 1:steps
	xe = frwd_kin(q(:,i));
	Ta1 = geo_2_ana(q(:,i));
    Ja = Jac(q(:,i), pd,Ta1);
    e(:,i) = [pd; phid]-xe;
    psinvjac = pinv(Ja);
    qdot = psinvjac*(gain*e(:,i));
    q(:,i+1) = q(:,i) + qdot*0.01;
    if (max(abs(e(1:3,i))) < 0.001 && max(abs(e(4:6,i))) < 0.0001)
        break;
    end
 end
 z = q(:,i);
 zdeg = rad2deg(z);
 for y = 1:7
     if zdeg(y)>(180)
         while zdeg(y)>(180)
             zdeg(y) = zdeg(y)-360;
         end
     elseif zdeg(y)<-180
         while zdeg(y)<-180
             zdeg(y) = zdeg(y)+360;
         end
     end
 end
 zrad= deg2rad(zdeg);
 Z = zrad';
 theta1 = Z(1);
 theta2 = Z(2);
 theta3 = Z(3);
 theta4 = Z(4);
 theta5 = Z(5);
 theta6 = Z(6);
 theta7 = Z(7);

% New_DH parameters
d = [0.340, 0, 0.4, 0, 0.4, 0, 0.126];
theta = [theta1, theta2, theta3, theta4, theta5, theta6, theta7];
a = [0, 0, 0, 0, 0, 0, 0];
alpha = [-pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2, 0];

T = eye(4);
% Transformation matrix initialization

for i = 1:numel(d)
    
    di = d(i);
    thetai = theta(i);
    ai = a(i);
    alphai = alpha(i);
    A = [cos(thetai) -sin(thetai)*cos(alphai) sin(thetai)*sin(alphai) ai*cos(thetai);
         sin(thetai) cos(thetai)*cos(alphai) -cos(thetai)*sin(alphai) ai*sin(thetai);
         0 sin(alphai) cos(alphai) di;
         0 0 0 1];
    T = T * A;
end

disp("Transformation Matrix:");
disp(T);

% Trajectory planning
fileID = fopen('Salian_Harshal.txt','w');
fmt = '%.8f %.8f %.8f %.8f %.8f %.8f %.8f\n';
Time = 10;  % total time in seconds
deltaT = 0.005;  % time step in seconds
nSteps = Time / deltaT; % should be 2000 (10 / 0.005)

q1 = q1'; % Make sure q1 is transposed if necessary
a0 = q1;
a1 = zeros(size(q1));  % assuming no initial velocity
a2 = 3 * (zrad - q1) / Time^2;
a3 = -2 * (zrad - q1) / Time^3;

for t = 0:deltaT:(Time-deltaT)  % Iterating from 0 to just below 10 seconds, total 2000 steps
    qt = a0 + a1*t + a2*t^2 + a3*t^3;
    fprintf(fileID, fmt, qt);
end
fclose(fileID);


fileID2 = fopen('Salian_Harshal_Velocity_Limits.txt', 'w');
fmt = '%.8f %.8f %.8f %.8f %.8f %.8f %.8f\n';

for t = 0:deltaT:(Time - deltaT)  % Looping from 0 to just below 10 seconds
    qt = a0 + a1*t + a2*t^2 + a3*t^3;
    qv = 3*a3*t^2 + 2*a2*t + a1;  % Velocity calculation
    qv = qv';  % Transpose if necessary to match your output format requirements
    fprintf(fileID2, fmt, qv);
end

fclose(fileID2);


  function p= frwd_kin(q)
    T01 = [cos(q(1)) 0 -sin(q(1)) 0; sin(q(1)) 0 cos(q(1)) 0; 0 -1 0 0.34; 0 0 0 1];
    T12 = [cos(q(2)) 0 sin(q(2)) 0; sin(q(2)) 0 -cos(q(2)) 0; 0 1 0 0; 0 0 0 1];
    T23 = [cos(q(3)) 0 sin(q(3)) 0; sin(q(3)) 0 -cos(q(3)) 0; 0 1 0 0.4; 0 0 0 1];
    T34 = [cos(q(4)) 0 -sin(q(4)) 0; sin(q(4)) 0 cos(q(4)) 0; 0 -1 0 0; 0 0 0 1];
    T45 = [cos(q(5)) 0 -sin(q(5)) 0; sin(q(5)) 0 cos(q(5)) 0; 0 -1 0 0.4; 0 0 0 1];
    T56 = [cos(q(6)) 0 sin(q(6)) 0; sin(q(6)) 0 -cos(q(6)) 0; 0 1 0 0; 0 0 0 1];
    T6E = [cos(q(7)) -sin(q(7)) 0 0; sin(q(7)) cos(q(7)) 0 0; 0 0 1 0.126; 0 0 0 1];
    T0E = T01*T12*T23*T34*T45*T56*T6E;
    x = T0E(1,4); y = T0E(2,4); z = T0E(3,4);
    R0E = T0E(1:3,1:3);
    eulerzyz = rotm2eul(R0E,'zyz');
    p= [x; y; z;eulerzyz(1);eulerzyz(2);eulerzyz(3)];

  end

%function for Ta matrix
 function Ta = geo_2_ana(q)
    T01 = [cos(q(1)) 0 -sin(q(1)) 0; sin(q(1)) 0 cos(q(1)) 0; 0 -1 0 0.34; 0 0 0 1];
    T12 = [cos(q(2)) 0 sin(q(2)) 0; sin(q(2)) 0 -cos(q(2)) 0; 0 1 0 0; 0 0 0 1];
    T23 = [cos(q(3)) 0 sin(q(3)) 0; sin(q(3)) 0 -cos(q(3)) 0; 0 1 0 0.4; 0 0 0 1];
    T34 = [cos(q(4)) 0 -sin(q(4)) 0; sin(q(4)) 0 cos(q(4)) 0; 0 -1 0 0; 0 0 0 1];
    T45 = [cos(q(5)) 0 -sin(q(5)) 0; sin(q(5)) 0 cos(q(5)) 0; 0 -1 0 0.4; 0 0 0 1];
    T56 = [cos(q(6)) 0 sin(q(6)) 0; sin(q(6)) 0 -cos(q(6)) 0; 0 1 0 0; 0 0 0 1];
    T6E = [cos(q(7)) -sin(q(7)) 0 0; sin(q(7)) cos(q(7)) 0 0; 0 0 1 0.126; 0 0 0 1];
    T0E = T01*T12*T23*T34*T45*T56*T6E;

   R0E = T0E(1:3,1:3);
   euler1 = rotm2eul(R0E,'zyz');
   Ta = [1 0 0 0 0 0;
       0 1 0 0 0 0;
       0 0 1 0 0 0;
       0 0 0 0 -sin(euler1(1)) cos(euler1(1))*sin(euler1(2));
       0 0 0 0 cos(euler1(1)) sin(euler1(1))*sin(euler1(2));
       0 0 0 1 0 cos(euler1(2))];
 end
 function J = Jac(q, pe,Ta)
   
    T01 = [cos(q(1)) 0 -sin(q(1)) 0; sin(q(1)) 0 cos(q(1)) 0; 0 -1 0 0.34; 0 0 0 1];
    T12 = [cos(q(2)) 0 sin(q(2)) 0; sin(q(2)) 0 -cos(q(2)) 0; 0 1 0 0; 0 0 0 1];
    T02 = T01*T12;
    T23 = [cos(q(3)) 0 sin(q(3)) 0; sin(q(3)) 0 -cos(q(3)) 0; 0 1 0 0.4; 0 0 0 1];
    T03 = T02*T23;
    T34 = [cos(q(4)) 0 -sin(q(4)) 0; sin(q(4)) 0 cos(q(4)) 0; 0 -1 0 0; 0 0 0 1];
    T04 = T03*T34;
    T45 = [cos(q(5)) 0 -sin(q(5)) 0; sin(q(5)) 0 cos(q(5)) 0; 0 -1 0 0.4; 0 0 0 1];
    T05 = T04*T45;
    T56 = [cos(q(6)) 0 sin(q(6)) 0; sin(q(6)) 0 -cos(q(6)) 0; 0 1 0 0; 0 0 0 1];
    T06 = T05*T56;
   
    z0 = [0 0 1]'; p0 = [0 0 0]';
    z1 = T01(1:3,3); p1 = T01(1:3,4);
    z2 = T02(1:3,3); p2 = T02(1:3,4);
    z3 = T03(1:3,3); p3 = T03(1:3,4);
    z4 = T04(1:3,3); p4 = T04(1:3,4);
    z5 = T05(1:3,3); p5 = T05(1:3,4);
    z6 = T06(1:3,3); p6 = T06(1:3,4);
    
    Jg = [cross(z0,pe-p0) cross(z1,pe-p1) cross(z2,pe-p2) cross(z3,pe-p3) cross(z4,pe-p4) cross(z5,pe-p5) cross(z6,pe-p6);
           z0 z1 z2 z3 z4 z5 z6];
       J = inv(Ta)*Jg;
end

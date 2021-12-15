%% Instruction
% The only section you need to modify is Section 5.
% This function is divided into multiple sections
%     1.Rotation Matrix                                 (dependency: rot3x3.m)
%     2.Homogeneous Transformation(rotation) Matrix     (dependency: rot.m)
%     3.Homogeneous Transformation(translation) Matrix  (dependency: trans.m)
%     4.DH Homogeneous Transformation Matrix            (dependency: DHmatrix.m)
%     5.Forward Kinematics                              (dependency: forwardKinematics.m)
%     6.Jacobian                                        (dependency: DHjacobian.m)
% 
% In order to execute each section, dependency files must be complete. 
% To run each section, you can click on 'Run Section' on EDITOR tab or press "Ctrl + Enter".
% Each section provides correct answers. Your result must match up with the correct answers.

%% 1.Rotation Matrix
theta1 = pi;
theta2 =0;
theta3 = pi/6;

R1 = rot3x3('x',theta1)
R2 = rot3x3('y',theta2)
R3 = rot3x3('z',theta3)

%Correct Outputs================
% R1 =
% 
%     1.0000         0         0
%          0    0.0000   -1.0000
%          0    1.0000    0.0000
% 
% R2 =
% 
%     0.0000         0   -1.0000
%          0    1.0000         0
%     1.0000         0    0.0000
%  
% R3 =
% 
%    -1.0000   -0.0000         0
%     0.0000   -1.0000         0
%          0         0    1.0000
% ==============================

%% 2.Homogeneous Transformation(rotation) Matrix
th = [pi/3; -pi/6; 3*pi/2];

Rot1 = rot('x',th(1))
Rot2 = rot('y',th(2))
Rot3 = rot('z',th(3))

%Correct Outputs==========================
% Rot1 =
% 
%      1     0     0     0
%      0     1     0     0
%      0     0     1     0
%      0     0     0     1
% 
% 
% Rot2 =
% 
%     0.0000         0    1.0000         0
%          0    1.0000         0         0
%    -1.0000         0    0.0000         0
%          0         0         0    1.0000
% 
% 
% Rot3 =
% 
%    -1.0000   -0.0000         0         0
%     0.0000   -1.0000         0         0
%          0         0    1.0000         0
%          0         0         0    1.0000
%=========================================

%% 3.DH Homogeneous Matrix
a = [10; 0; 0];
d = [5; 5; 0];
alp = [pi; -pi; pi/3];
th = [-pi/2; pi/6; pi/3];

DH1 = DHmatrix(a(1), d(1), alp(1), th(1))
DH2 = DHmatrix(a(2), d(2), alp(2), th(2))
DH3 = DHmatrix(a(3), d(3), alp(3), th(3))

% %Correct Outputs==========================
% DH1 =
% 
%     1.0000         0         0    1.0000
%          0    0.0000    1.0000         0
%          0   -1.0000    0.0000    3.0000
%          0         0         0    1.0000
% 
% DH2 =
% 
%     0.0000    1.0000    0.0000    0.0000
%     1.0000   -0.0000   -0.0000    2.0000
%          0    0.0000   -1.0000    2.0000
%          0         0         0    1.0000
% 
% DH3 =
% 
%    -1.0000   -0.0000    0.0000   -3.0000
%     0.0000   -0.0000    1.0000    0.0000
%          0    1.0000    0.0000    1.0000
%          0         0         0    1.0000
% ========================================

%% 4.Forward Kinematics
%Note===================================================================== 
%a, d, and alp are DH parameters for the robot shown in Figure 1.
%========================================================================= 

a = [0; 10; 10];
d = [10; 0; 0];
alp = [-pi/2; 0; 0];
th = [pi/4; -pi/4; pi/4];

FKin = forwardKinematics(a,d,alp,th);
plotManipulator(a,d,alp,th);

FKin01 = FKin(:,:,1)
FKin02 = FKin(:,:,2)
FKin03 = FKin(:,:,3)

%Correct Outputs==========================
% FKin01 =
% 
%     1.0000         0         0         0
%          0    0.0000    1.0000         0
%          0   -1.0000    0.0000   10.0000
%          0         0         0    1.0000
% 
% FKin02 =
% 
%     0.0000    1.0000         0    0.0000
%    -0.0000    0.0000    1.0000   -0.0000
%     1.0000   -0.0000    0.0000   20.0000
%          0         0         0    1.0000
% 
% FKin03 =
% 
%     1.0000         0         0   10.0000
%          0    0.0000    1.0000   -0.0000
%          0   -1.0000    0.0000   20.0000
%          0         0         0    1.0000
% ========================================

%% 5.Jacobian
a = [0; 10; 10];
d = [10; 0; 0];
alp = [-pi/2; 0; 0];
th = [pi/4; -pi/4; pi/4];
type = ['r';'r';'r'];

J = DHjacobian(a,d,alp,th,type);

%Correct Outputs=================
% J =
% 
%     0.0000   10.0000         0
%    10.0000    0.0000    0.0000
%          0  -10.0000  -10.0000
%          0         0         0
%          0    1.0000    1.0000
%     1.0000    0.0000    0.0000
%================================

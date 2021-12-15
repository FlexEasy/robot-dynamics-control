function [rtn] = DHmatrix(a,d,alp,th)
%% Instruction
%Parameters==================================================================================
%Inputs:
%   a: DH parameter a
%   d: DH parameter d
%   alp: DH parameter alpha in radian
%   th: DH parameter theta in radian
%Output:
%   rtn(rtn stands for return): 4X4 DH matrix
%============================================================================================

%% Calculation
    M_th=[cos(th),   -sin(th),   0,  0;
        -sin(th),    cos(th),   0,  0;
            0,          0,      1,  0;
            0,          0,      0,  1];
    M_d=[1,  0,  0,  0;
         0,  1,  0,  0;   
         0,  0,  1,  d;
         0,  0,  0,  1];
    M_a =[1,  0,  0,  a;
         0,  1,  0,  0;   
         0,  0,  1,  0;
         0,  0,  0,  1];
    M_alp=[1,    0,         0,      0;
           0,  cos(alp), -sin(alp), 0;
           0,  sin(alp), cos(alp),  0;
           0,    0,         0,      1];
       
    rtn =M_th*M_d*M_a*M_alp;
    
end
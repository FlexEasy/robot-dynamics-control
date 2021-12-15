function [rtn] = forwardKinematics(a,d,alp,th)
%% Instruction
%Parameters==================================================================================
%Inputs:
%   a: DH parameter a in vector form
%   d: DH parameter d in vector form
%   alp: DH parameter alpha in radian in vector form
%   th: DH parameter theta in radian in vector form
%Output:
%   rtn(rtn stands for return): 4X4 homogeneous transformation matrices 
%                               rtn(:,:,1) = H01, rtn(:,:,2) = H02, .......
%============================================================================================

%% Calculation
    dataSize = max(size(a));

    %Initialize the matrix
    matrix = zeros(4,4,dataSize);
    for i = 1: dataSize + 1
        matrix(1:4,1:4,i) = eye(4); 
    end

    for i = 1:dataSize
        matrix(:,:,i + 1) =  matrix(:,:,i)*DHmatrix(a(i),d(i),alp(i),th(i));
    end

    rtn = matrix(:,:,2:dataSize + 1);

end
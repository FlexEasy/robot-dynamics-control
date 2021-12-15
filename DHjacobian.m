function rtn = DHjacobian(a,d,alp,th,type)
%% Instruction
%Parameters===============================================================
%Inputs:
%   a: DH parameter a in vector form
%   d: DH parameter d in vector form
%   alp: DH parameter alpha in radian in vector form
%   th: DH parameter theta in radian in vector form
%   type: Joint type in a vector form 
%         ex) type = ['r'; 'p'; 'r']; where r or R: revolute joint
%                                           p or P: prismatic joint
%   
%Output:
%   rtn(rtn stands for return): 4X4 homogeneous transformation matrices 
%                               rtn(:,:,1) = H01, rtn(:,:,2) = H02, ......
%=========================================================================

%Note===================================================================== 
%Since MATLAB does NOT provide index 0, we need to add 1 to all subscripts
%========================================================================= 

%% Initialization
    dataSize = max(size(a));
    H = forwardKinematics(a,d,alp,th);
    %p is location of origin in base frame
    p = zeros(3,dataSize + 1);
    %z is first 3 rows of third column of H matrix
    z = zeros(3,dataSize + 1);
    z(1:3,1) = [0;0;1];
    rtn = zeros(6,dataSize);
    
%% Calculation 1
    for i=1:dataSize
        p(1:3,i + 1) = H(1:3,4,i);
        z(1:3,i + 1) = H(1:3,3,i);
    end    
    
    for i = 1:dataSize
        if( type(i) == 'r' || type(i) == 'R') %q = th
            rtn(1:3,i) = cross(z(1:3,i),p(1:3,datasize+1)-p(1:3,i));
            rtn(4:6,i) = z(1:3,i);
        elseif( type(i) == 'p' || type(i) == 'P' )% q = d 
            rtn(1:3,i) = z(1:3,i);
            rtn(4:6,i) = 0;
        end
    end
    
end
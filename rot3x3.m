
function [rtn] = rot3x3(axis, th)
%% Instruction
% Parameters====================================================================================
% Inputs:
%   axis: Rotation axis (Options: 'x', 'y', 'z')(Not case sensitive)
%   th: Rotation angle in radian
% Output:
%   rtn(rtn stands for return): 3X3 rotation matrix 
%==============================================================================================

% Note================================================================ 
% Make sure you put single quotation mark arround the variable 'axis' 
% when you call this function
%==================================================================== 

% Example==========================
% >> rot('x',pi)
% 
% ans =
% 
%     1.0000         0         0
%          0   -1.0000   -0.0000
%          0    0.0000   -1.0000
%=================================
%% Calculation
    if(axis == 'x' || axis == 'X')
        rtn = [ 1,        0,           0;
                0,     cos(th),    -sin(th)
                0,     sin(th),     cos(th)];
    end
    
    if(axis == 'y' || axis == 'Y')
        rtn = [cos(th),   0,   sin(th);
                0,         1,       0;
              -sin(th),   0,    cos(th)];
    end
    
    if(axis == 'z' || axis == 'Z')
        rtn = [cos(th),   -sin(th),   0;
               sin(th),    cos(th),    0;
                  0,        0,         1 ];
    end
    
end

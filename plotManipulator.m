function [] = plotManipulator(a,d,alpha,th)
%Note===================================================================== 
%Nothing to modify in this function
%========================================================================= 

%Parameters===========================================
%Inputs:
%   a: DH parameter a in vector form
%   d: DH parameter d in vector form
%   alpha: DH parameter alpha in radian in vector form
%   th: DH parameter theta in radian in vector form
%Output:
%   rtn: 3X1 end effector position
%=====================================================

%% Calculation
%Calculate data size
dataSize = max(size(a));

%Initialize
origins = zeros(3,dataSize + 1);

H = forwardKinematics(a,d,alpha,th);
for i = 1:dataSize
    origins(:,i + 1) = H(1:3,4,i); 
end

%% Plot
figure();
hold on;
grid on;
axis equal;

%Plot Joints
for i = 1:dataSize + 1
    plot3(origins(1,i), origins(2,i), origins(3,i), '*','LineWidth',20);
end

%Plot Links
for i = 1:dataSize
    plot3([origins(1,i), origins(1,i+1)], [origins(2,i), origins(2,i+1)], [origins(3,i),origins(3,i+1)],'LineWidth',5);
end

xlabel('x Position [m]');
ylabel('y Position [m]');
zlabel('z Position [m]');

end
function [ Cbn ] = EulerAngleToDCM( EA )
%   欧拉角转姿态矩阵DCM
%   EA 是欧拉角矩阵 3×1
%   注意：EA中角度单位为弧度rad
%   DCM 姿态矩阵 3×3
phi= EA(1,1);
theta= EA(2,1);
psi= EA(3,1);
Cbn(1,1)= cos(theta)*cos(psi);
Cbn(1,2)= -cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi);
Cbn(1,3)= sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);
Cbn(2,1)= cos(theta)*sin(psi);
Cbn(2,2)= cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi);
Cbn(2,3)= -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);
Cbn(3,1)= -sin(theta);
Cbn(3,2)= sin(phi)*cos(theta);
Cbn(3,3)= cos(phi)*cos(theta);
end


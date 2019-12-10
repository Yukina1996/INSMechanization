function [ Qbn ] = RotationVectorToQ( RV )
%   旋转矢量转化为姿态四元数
%	RV is for rotation vector:旋转矩阵 3×1
%   Qbn：从b系转换到n系的姿态四元数
mod= sqrt(RV(1,1)*RV(1,1)+RV(2,1)*RV(2,1)+RV(3,1)*RV(3,1));
Qbn(1,1)=cos(0.5*mod);
Qbn(2:4,1)=(sin(0.5*mod)/mod)*RV;
end


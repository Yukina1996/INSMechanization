function [ Cbn ] = RotationVectorToDCM( RV )
%   等效旋转矢量转换成姿态阵
%   RV is for Rotation Vector: 旋转矢量 3×1
%   Cbn: 方向余弦矩阵/姿态阵
%   mod:表示旋转矢量的范数/模
mod= sqrt(RV(1,1)*RV(1,1)+RV(2,1)*RV(2,1)+RV(3,1)*RV(3,1));
I= zeros(3,3);
I(1,1)=1;
I(2,2)=1;
I(3,3)=1;
asymRV= AntisymMatrix(RV);
Cbn= I+(sin(mod)/mod)*asymRV+((1-cos(mod))/(mod*mod))*asymRV*asymRV;
end


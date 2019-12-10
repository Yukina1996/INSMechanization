function [ Qbn ] = RotationVectorToQ( RV )
%   ��תʸ��ת��Ϊ��̬��Ԫ��
%	RV is for rotation vector:��ת���� 3��1
%   Qbn����bϵת����nϵ����̬��Ԫ��
mod= sqrt(RV(1,1)*RV(1,1)+RV(2,1)*RV(2,1)+RV(3,1)*RV(3,1));
Qbn(1,1)=cos(0.5*mod);
Qbn(2:4,1)=(sin(0.5*mod)/mod)*RV;
end


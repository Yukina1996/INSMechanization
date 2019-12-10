function [ Cbn ] = RotationVectorToDCM( RV )
%   ��Ч��תʸ��ת������̬��
%   RV is for Rotation Vector: ��תʸ�� 3��1
%   Cbn: �������Ҿ���/��̬��
%   mod:��ʾ��תʸ���ķ���/ģ
mod= sqrt(RV(1,1)*RV(1,1)+RV(2,1)*RV(2,1)+RV(3,1)*RV(3,1));
I= zeros(3,3);
I(1,1)=1;
I(2,2)=1;
I(3,3)=1;
asymRV= AntisymMatrix(RV);
Cbn= I+(sin(mod)/mod)*asymRV+((1-cos(mod))/(mod*mod))*asymRV*asymRV;
end


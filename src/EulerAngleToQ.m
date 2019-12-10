function [ Qbn ] = EulerAngleToQ( EA )
%   ŷ��������ת��̬��Ԫ��
%   EularAngle�ɺ���ǡ������ǡ��������ɣ�3��1
%   Qbn����bϵת��nϵ����̬��Ԫ��,4��1
phi= EA(1,1);
theta= EA(2,1);
psi= EA(3,1);
Qbn(1,1)= cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2);
Qbn(2,1)= sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2);
Qbn(3,1)= cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2);
Qbn(4,1)= cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2);
end


function [ result ] = Qmultiply( P,Q )
%	��̬��Ԫ���ĳ˷�
%   P��Q����Ԫ����result�����P*Q�Ľ��
%   P��Qά������4��1��
p0=P(1,1);
p1=P(2,1);
p2=P(3,1);
p3=P(4,1);

trans=[p0,-p1,-p2,-p3;...
    p1,p0,-p3,p2;...
    p2,p3,p0,-p1;...
    p3,-p2,p1,p0];
result=trans*Q;
end


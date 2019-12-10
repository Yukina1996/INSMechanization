function [ result ] = Qmultiply( P,Q )
%	姿态四元数的乘法
%   P，Q：四元数；result：存放P*Q的结果
%   P，Q维数均是4行1列
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

